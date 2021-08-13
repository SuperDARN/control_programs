/* normalsound.c
   ============
   Author: Dieter Andre
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <env.h>
#include <math.h>
#include <process.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iqdata.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"


#include "taskid.h"
#include "errlog.h"
#include "rmsg.h"
#include "radarshell.h"
#include "rmsgsnd.h"
#include "tsg.h"
#include "tsgtable.h"
#include "maketsg.h"
#include "global.h"
#include "setup.h"
#include "reopen.h"
#include "tmseq.h"
#include "build.h"
#include "sync.h"
#include "interface.h"
#include "hdw.h"

/*
 $Log: normalsound.c,v $
 Revision 2.9  2011/05/12 18:00:00  DAndre
 Conversion from 1.20 to 1.25
 These changes have been done earlier [in 1.20]:
 Store all ranges and consider in the scatter percentage.
 Change in snd file format.

 Revision 2.7  2008/03/15 00:47:25  code
 Added iqwrite.

 Revision 2.6  2008/03/14 17:47:55  code
 Added support for I&Q capture.

 Revision 2.5  2006/11/27 18:14:56  DAndre
 Added parameter nrang.
 Prevent freq switch before dwell time is up.

 $Log: normalsound.c,v $
 Revision 2.5  2006/07/12 15:50:19  code
 Added call to set up command line and limited data written to rawacf and fitacf.

 Revision 2.4  2006/04/13 18:14:36  barnes
 Incorporated Dieter Andre's bug fix.

 Revision 2.3  2006/02/07 20:55:52  barnes
 Simon Shepherd's modification to the lag table.

 Revision 2.2  2006/02/07 17:50:57  barnes
 Added Dieter Andre's improved error logging.

 Revision 2.1  2005/07/19 18:43:54  barnes
 First revision included in the ROS 1.08.


 Revision 2.0  2004/03/23 andre
 Initial revision from John Hughes program
 
*/

/*======================================================================*/
/* Program Description:							*/
/* This version of normalsound runs under ROS 1.08			*/
/* If you want to run fastsound just run normalsound -fast		*/
/*									*/
/* normalsound performs a scan through all 16 beams at 6s or 3s		*/
/* integration time. In the remaining time until the end of the minute	*/
/* it performs scans through a set of up to 12 frequencies and through	*/
/* all beams [even/odd]. This is used to determine the frequency which	*/
/* gives the most ionospheric scatter from time to time [default 15min]	*/
/*									*/
/* The file $SD_HDWPATH/sounder.dat shoudl contain the following values	*/
/* one per line:							*/
/* Time between frequency evaluations [min]				*/
/* Number of sounder frequencies					*/
/* The sounder frequencies [kHz]					*/
/*									*/
/* If this file does not exist, default values are used. This is not a	*/
/* good idea, as the program may try to sound at forbidden frequencies.	*/
/*									*/
/* The sounding data are writen to *.snd files, for each beam one	*/
/* header and a data record for each good [qflg=1] range.		*/
/* We have decided to do away with the internal compression, since we	*/
/* feel, that zipping the files afterwards gives the same result.	*/
/*									*/
/* The information used to determine the optimal frequency is kept in	*/
/* an internal circular buffer, that can hold 1 hour of data. This way	*/
/* we avoid having to read the files each 15 minutes or so.		*/
/*									*/
/* At the moment the whole buffer is used to calculate for each beam	*/
/* and each sounder frequency :						*/
/* ( #total_returns - #groundscatter_returns)/ #ranges			*/
/* These values are then averaged over all beams and if the maximum	*/
/* gives a sufficient improvement over the presently used one, the	*/
/* frequency is switched [ see find_optimal_freq for details]		*/
/*======================================================================*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: normalsound.c,v 2.9 2011/05/12 18:00:00 DAndre Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;

/* used by cutlass radars */
/* extern int lsfreq[], lfreq_range[]; */

/* Up to 1 hour of sounding data is stored in this structure */

/* 24* 30/ sounder_intt */
#define NUM_SND_DATA 360
#define SND_NBM 16
#define MAX_SND_FREQS 12

struct sounder_struct
  {
  double stime;
  char program_name[40];
  int site_id;
  int beam_num;
  int freq;
  int noise;
  int frange;
  int rsep;
  int nrang;
  float pwr[ MAX_RANGE];
  float vel[ MAX_RANGE];
  float width[ MAX_RANGE];
  float  AOA[ MAX_RANGE];
  int gsct[ MAX_RANGE];
  int qflg[ MAX_RANGE];
  };


int main(int argc,char *argv[]) {

  int ptab[8] = {0,14,22,24,27,31,42,43};

  int lags[LAG_SIZE][2] = {
    { 0, 0},    /*  0 */
    {42,43},    /*  1 */
    {22,24},    /*  2 */
    {24,27},    /*  3 */
    {27,31},    /*  4 */
    {22,27},    /*  5 */

    {24,31},    /*  7 */
    {14,22},    /*  8 */
    {22,31},    /*  9 */
    {14,24},    /* 10 */
    {31,42},    /* 11 */
    {31,43},    /* 12 */
    {14,27},    /* 13 */
    { 0,14},    /* 14 */
    {27,42},    /* 15 */
    {27,43},    /* 16 */
    {14,31},    /* 17 */
    {24,42},    /* 18 */
    {24,43},    /* 19 */
    {22,42},    /* 20 */
    {22,43},    /* 21 */
    { 0,22},    /* 22 */

    { 0,24},    /* 24 */

    {43,43}};   /* alternate lag-0  */



  char *sname=NULL;
  char *ename=NULL;
  char *sdname={SCHEDULER};
  char *edname={ERRLOG};
  char logtxt[1024];

  int n;
  pid_t sid;
  int exitpoll=0;
 
  int scnsc=120;
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char fast=0;
  unsigned char discretion=0;

  /* Variables for sounding */
  FILE *snd_dat;
  /* If the file $SD_HDWPATH/sounder.dat exists, the next three parameters are read from it */
  /* the file contains one integer value per line */
  int freq_dwell=15; /* after so many minutes a new optimal frequency is evaluated */
  int sounder_freqs_total=8;
  int sounder_freqs[ MAX_SND_FREQS]= { 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0 };
  time_t last_freq_search, t_now;
  int fw=0; // frequency weighting flag used in selecting the optimal freq
  int sounder_beams[]={0,2,4,6,8,10,12,14};
  int sounder_freq_count=0, sounder_beam_count=0;
  int sounder_beams_total=8, odd_beams=0;
  int sounder_freq;
  int sounder_beam_loop=1;
  int normal_intt=6;
  int fast_intt=3;
  int sounder_intt=2; 
  int do_new_freq_search=0;
  float sounder_time, time_needed=1.25;
  int cutlass=0;
  struct sounder_struct *sounder_data;
  int act_snd_rec= 0;
  
  snd_dat= fopen( getenv("SD_SND_TABLE"), "r");
  if( snd_dat != NULL ) {
    fscanf( snd_dat, "%d", &freq_dwell);
    fscanf( snd_dat, "%d", &sounder_freqs_total);
    if (sounder_freqs_total > 12) sounder_freqs_total= 12;
    for ( sounder_freq_count=0; sounder_freq_count < sounder_freqs_total; sounder_freq_count++ )
      fscanf( snd_dat, "%d", &sounder_freqs[ sounder_freq_count] );
    sounder_freq_count= 0;
    fclose( snd_dat);
  }

  
  sounder_data= ( struct sounder_struct *) calloc( sizeof( struct sounder_struct), NUM_SND_DATA);
  
  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 

  strncpy(combf,progid,80);
  OpsSetupCommand(argc,argv);
  OpsSetupRadar();
  OpsSetupShell();
   
  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
                  &sbm,&ebm,
                  &dfrq,&nfrq,
                  &dfrang,&nfrang,
                  &dmpinc,&nmpinc,
                  &frqrng,&xcnt);

  cp=155; /* normalsound */
  intsc= normal_intt;
  intus=0;
  mppul=8;
  mplgs=23;
  mpinc=1500;
  dmpinc=1500;
  nmpinc=1500;
  nrang=75;
  rsep=45;
  txpl=300;

  SiteStart();
  /* need to find out whether this is a CUTLASS-type radar */
  if( getenv("CUTLASS_ADDRESS")==NULL ) cutlass=0;
  else cutlass=1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt,"el",'t',&ename);
  OptionAdd(&opt,"sc",'t',&sname);

  OptionAdd( &opt, "di", 'x', &discretion);
  OptionAdd( &opt, "dt", 'i', &day);
  OptionAdd( &opt, "nt", 'i', &night);
  OptionAdd( &opt, "df", 'i', &dfrq);
  OptionAdd( &opt, "nf", 'i', &nfrq);
  OptionAdd( &opt,"dr",  'i', &dfrang);
  OptionAdd( &opt,"nr",  'i', &nfrang);
  OptionAdd( &opt, "dm", 'i', &dmpinc);
  OptionAdd( &opt, "nm", 'i', &nmpinc);
  OptionAdd( &opt, "sb", 'i', &sbm);
  OptionAdd( &opt, "eb", 'i', &ebm);
  OptionAdd( &opt, "xcf", 'i', &xcnt);
  OptionAdd( &opt, "dwell", 'i', &freq_dwell);
  OptionAdd( &opt, "nrang", 'i', &nrang);
  OptionAdd( &opt, "rsep", 'i', &rsep);
  OptionAdd(&opt,"fast", 'x', &fast);
 
  arg=OptionProcess(1,argc,argv,&opt,NULL);  

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);  

  SiteSetupHardware();

  if (fast) {
     cp= 157; /* fastsound */
     scnsc=60;
     scnus=0;
     intsc= fast_intt;
     intus=0;
  }
  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  if (fast) sprintf(progname,"normalsound (fast)");
  else sprintf(progname,"normalsound");

  OpsFitACFStart();
  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
  }

  /* only set frequency to their default on program start */
  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  time( &last_freq_search);
  if (OpsDayNight()==1) {
    stfrq=dfrq;
  } else {
    stfrq=nfrq;
  }

  do {

    if (SiteStartScan()==0) continue;
    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(tlist[n]);
        RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
      }
    }

    scan=1;
    ErrLog(errlog,progname,"Starting scan.");
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    skip=OpsFindSkip(scnsc,scnus);
    if (backward) {
      bmnum=sbm-skip;
      if (bmnum<ebm) bmnum=sbm;
    } else {
      bmnum=sbm+skip;
      if (bmnum>ebm) bmnum=sbm;
    }

    do {
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      if (OpsDayNight()==1) {
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        mpinc=nmpinc;
        frang=nfrang;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,
                      intsc,intus,hr,mt,sc,us);
      ErrLog(errlog,progname,logtxt);
      ErrLog(errlog,progname,"Setting beam.");
      SiteSetIntt(intsc,intus);
      SiteSetBeam(bmnum);

      ErrLog(errlog,progname,"Doing clear frequency search."); 
      if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
      SiteSetFreq(tfreq);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog,progname,logtxt);
      tsgid=SiteTimeSeq(ptab);
      nave=SiteIntegrate(lags);  
      if (nave < 0) { 
        sprintf( logtxt, "Integration error: %d", nave);
        ErrLog(errlog,progname, logtxt);
        spawnl( P_WAIT, "/home/radar/script/restart.radar", NULL); 
        exit( nave);
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog,progname,logtxt);

      OpsBuildPrm(&prm,ptab,lags);
      OpsBuildIQ(&iq);
      OpsBuildRaw(&raw);

      FitACF(&prm,&raw,&fblk,&fit);

      ErrLog(errlog,progname,"Sending messages."); 
      msg.num=0;
      msg.tsize=0;
      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prm, PRM_TYPE,0); 
      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);
      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);
      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &raw, RAW_TYPE,0);    
      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fit, FIT_TYPE,0);   
      RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,0);   
      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg);
  
      ErrLog(errlog,progname,"Polling for exit."); 
      exitpoll=RadarShell(sid,&rstable);
      if (exitpoll !=0) break;
      scan=0;
      if (bmnum==ebm) break;
      if (backward) bmnum--;
      else bmnum++;

    } while (1);
    ErrLog(errlog,progname,"Waiting for scan boundary."); 
  
    if (exitpoll==0) {
      /* In here comes the sounder code */
      /* see if it's time for a new freq search */
      time(&t_now);
      do_new_freq_search= ( freq_dwell>0 && freq_dwell<=((t_now-last_freq_search)/60.) );
      /* set the "sounder mode" scan variable */
      scan=-2;
      /* set the xcf variable to do cross-correlations (AOA) */
      xcf=1;
      /* we have time until the end of the minute to do sounding */
      /* minus a safety factor given in time_needed */
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      sounder_time= 60.0 - ( sc + us/ 1000000.0);
      /* we do not implement to no sounding mode here */
      /* do a new frequency search if it's time */
      if( do_new_freq_search && sounder_time>=5 ) {
	    do_new_freq_search=0;
	    stfrq= find_optimal_freq( stfrq, cutlass, fw, sounder_freqs, sounder_freqs_total, sounder_data, act_snd_rec);
        sprintf( logtxt,"New Opt Freq; %d\n", stfrq);
        ErrLog( errlog, progname, logtxt);
	    last_freq_search= t_now;
      }
      sounder_beam_loop= ( sounder_time-(float)sounder_intt > time_needed );
      while( sounder_beam_loop ) {
	    intsc= sounder_intt;
	    /* set the beam */
	    bmnum=sounder_beams[sounder_beam_count]+odd_beams;
	    /* sounder_freq will be an array of frequencies to step through */
	    if( !cutlass )
	      sounder_freq=sounder_freqs[sounder_freq_count];
/*
	else {
	  sounder_freq=lsfreq[sounder_freqs[sounder_freq_count]];
	  frqrng=lfreq_range[sounder_freqs[sounder_freq_count]];
	}
*/
	  /* the scanning code here */	
	  sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%d:%d:%d:%d)", bmnum, intsc,intus,hr,mt,sc,us);
	  ErrLog(errlog,progname,logtxt);
	  ErrLog(errlog,progname,"Setting SND beam.");
	  SiteSetIntt(intsc,intus);
	  SiteSetBeam(bmnum);
	  ErrLog( errlog, progname, "Doing SND clear frequency search."); 
	  if (SiteFCLR( sounder_freq, sounder_freq + frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
	  SiteSetFreq(tfreq);
/*
	  sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
	  ErrLog( errlog, progname, logtxt);
*/
	  tsgid= SiteTimeSeq(ptab);
	  nave= SiteIntegrate( lags);   
	  if (nave < 0) {
            sprintf( logtxt, "SND integration error: %d", nave);
	    ErrLog(errlog,progname, logtxt);
            spawnl( P_WAIT, "/home/radar/script/restart.radar", NULL);
            exit( nave);
	  }
	  sprintf(logtxt,"Number of SND sequences: %d",nave);
	  ErrLog(errlog,progname,logtxt);

          OpsBuildPrm(&prm,ptab,lags);
          OpsBuildIQ(&iq);
          OpsBuildRaw(&raw);
	  FitACF(&prm,&raw,&fblk,&fit);

	  ErrLog( errlog, progname, "Sending SND messages."); 
	  msg.num= 0;
	  msg.tsize= 0;
	  RMsgSndAdd( &msg, sizeof(struct RadarParm), (unsigned char *) &prm, PRM_TYPE, 0); 
	  RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);
	  RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);
	  RMsgSndAdd( &msg, sizeof(struct RawData), (unsigned char *) &raw, RAW_TYPE, 0);    
	  RMsgSndAdd( &msg, sizeof(struct FitData), (unsigned char *) &fit, FIT_TYPE, 0);   
	  RMsgSndAdd( &msg, strlen(progname)+1, progname, NME_TYPE, 0);
	  /* Only send these to echo_data; otherwise they get written to the data files */   
	  RMsgSndSend( tlist[ 0], &msg);

	  sprintf( logtxt, "SBC: %d  SFC: %d\n", sounder_beam_count, sounder_freq_count);
	  ErrLog( errlog, progname, logtxt);
	  /* save the sounding mode data */
	  write_sounding_record( progname, &prm, &fit, sounder_data, &act_snd_rec);
 
	  ErrLog( errlog, progname, "Polling SND for exit."); 
	  exitpoll=RadarShell(sid,&rstable);
	  if (exitpoll !=0) break;

	  /* check for the end of a beam loop */
      sounder_freq_count++;
	  if( sounder_freq_count >= sounder_freqs_total ) {
	    /* reset the freq counter and increment the beam counter */
	    sounder_freq_count=0;
	    sounder_beam_count++;
	    if( sounder_beam_count>=sounder_beams_total ) {
	      sounder_beam_count=0;
	      if( odd_beams==0 )
	        odd_beams=1;
	      else
	        odd_beams=0;
	      sounder_freq_count=0;
	    }
	  }
	  /* see if we have enough time for another go round */ 
	  TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);
	  sounder_time= 60.0 - ( sc + us/ 1000000.0);
	  sounder_beam_loop= ( sounder_time-(float)sounder_intt > time_needed );
      }
      /* now wait for the next normal_scan */
      intsc=normal_intt;
      if ( fast) intsc= fast_intt;
      OpsWaitBoundary(scnsc,scnus);
    }

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;   
} 
 

/********************** function write_sounding_record() ************************/
/* changed the data structure to use uncompressed values DAndre */

void write_sounding_record( char *progname, struct RadarParm *prm, struct FitData *fit, struct sounder_struct *sounder_data, int *act_snd_rec)
{
  int i;

  struct header_struct
	{
	long int stime;
	short int  site_id;
	short int beam_no;
	short int freq;
	short int noise;
	short int frange;
	short int rsep;
	short int nrang; /* Number of ranges sampled */
	short int gsct[ MAX_RANGE];
	short int qflg[ MAX_RANGE];
	char program_name[40];
	} header;

  struct data_struct
	{
	short int pwr;
	short int vel;
	short int width;
	short int AOA;
	} data;

  //char data_path[]={"/data/snd/"}, data_filename[50], filename[80];
  char data_path[100], data_filename[50], filename[80];
 

  int  good_ranges[ MAX_RANGE];

  char *snd_dir;
  FILE *out;

  struct sounder_struct *act_snd_data;


  /* set up the data directory */
  /* get the snd data dir */
  snd_dir= getenv("SD_SND_PATH");
  if( snd_dir==NULL )
    sprintf( data_path,"/data/snd/");
  else {
    memcpy( data_path,snd_dir,strlen(snd_dir));
    data_path[ strlen( snd_dir)]= '/';
    data_path[ strlen( snd_dir) + 1]= 0;
  }

  /* make up the filename */
  /* YYYYMMDDHH */
  sprintf( data_filename, "%04d%02d%02d%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, getenv("SD_RADARCODE"));
  /* finally make the filename */
  sprintf( filename, "%s%s.snd", data_path, data_filename);
  /* open the output file */
  out= fopen(filename,"a");
  if( out==NULL ) {
	/* crap. might as well go home */
	return;
  }

  /* make the header */

  header.stime= TimeYMDHMSToEpoch( prm->time.yr, prm->time.mo, prm->time.dy, prm->time.hr, prm->time.mt, prm->time.sc);
  header.site_id= prm->stid;
  header.beam_no= prm->bmnum;
  header.freq= prm->tfreq;
  header.noise= prm->noise.mean;
  header.frange= prm->frang;
  header.rsep= prm->rsep;
  header.nrang= prm->nrang;
  memcpy( header.program_name, progname, sizeof( header.program_name));
 /* zero out the gscat and qual bytes */
  for( i=0; i< MAX_RANGE; i++ ) {
    header.gsct[i]= 0;
    header.qflg[i]= 0;
    good_ranges[ i]= 0;
  }
  for( i=0; i< header.nrang; i++ ) {
    header.gsct[i]= fit->rng[i].gsct;
    header.qflg[i]= fit->rng[i].qflg;
    good_ranges[ i]= ( fit->rng[i].qflg == 1);
  }

  /* write out the header */
  fwrite( &header, sizeof( header), 1, out);

  /* scale the fit data into the char/shorts */
  for( i=0; i< header.nrang; i++ ) {
    /* only do the good ranges */
    if( good_ranges[i] ) {
      /* do the power */
      data.pwr= fit->rng[i].p_l;
      /* do the velocity */
      data.vel= fit->rng[i].v;
      /* do the AOA */
      data.AOA= fit->elv[i].normal;
      /* do the width */
      data.width= fit->rng[i].w_l;
      /* write out the data structure */
      fwrite( &data, sizeof( data), 1, out);
    }
  }
  fclose(out);


  /* Fill the next sounder data record */
  act_snd_data= sounder_data + *act_snd_rec;
  act_snd_data->stime= TimeYMDHMSToEpoch( prm->time.yr, prm->time.mo, prm->time.dy, prm->time.hr, prm->time.mt, prm->time.sc);
  memcpy( act_snd_data->program_name, progname, sizeof(act_snd_data->program_name));
  act_snd_data->site_id= prm->stid;
  act_snd_data->beam_num= prm->bmnum;
  act_snd_data->freq= prm->tfreq;
  act_snd_data->noise= prm->noise.mean;
  act_snd_data->frange= prm->frang;
  act_snd_data->rsep= prm->rsep;
  act_snd_data->nrang= prm->nrang;
  for( i=0; i< prm->nrang; i++ ) {
    act_snd_data->pwr[ i]= fit->rng[ i].p_l;
    act_snd_data->vel[ i]= fit->rng[ i].v;
    act_snd_data->width[ i]= fit->rng[ i].w_l;
    act_snd_data->AOA[ i]= fit->elv[ i].normal;
    act_snd_data->gsct[ i]= fit->rng[ i].gsct;
    act_snd_data->qflg[ i]= fit->rng[ i].qflg;
  }
  *act_snd_rec= *act_snd_rec + 1;
  if ( *act_snd_rec >= NUM_SND_DATA) *act_snd_rec= 0;
} /* write_sounding_record */

/****************** function compute_scatter_percentage *************************/
/* Mod: 20050315																*/
/* Use ths last ~15min sounder data to calculate the ionospheric scatter percentage */
/* for each sounder frequency and in each beam									*/
/* a lot more could be done, but for now we will keep it simple					*/

void compute_scatter_percentage( struct sounder_struct *sounder_data, int act_snd_rec, int sounder_freqs[], int sft, float iscat_percent[ MAX_SND_FREQS][ SND_NBM] )
{
  struct sounder_struct *act_snd_data;
  int isnd, jsnd, ifrq, i;
  int returns=0, gscat_returns=0;
  
  for ( jsnd=0; jsnd< NUM_SND_DATA/ 4; jsnd++) {
    isnd= act_snd_rec - jsnd;
    if (isnd < 0) isnd= isnd + NUM_SND_DATA;
    act_snd_data= sounder_data + isnd;
    /* make sure this record has data */
    if (act_snd_data->stime > 0 ) {
      returns=0;
      gscat_returns=0;
      for( i=0; i< act_snd_data->nrang; i++ ) {
	if( ( act_snd_data->qflg[i] == 1) && act_snd_data->pwr[i] >= 3.0 && act_snd_data->width[i] < 500.0 ) {
	  returns++;
	  if( act_snd_data->gsct[i] ) gscat_returns++;
	}
      }
     for ( ifrq=0; ifrq< sft; ifrq++) {
       if ( (act_snd_data->freq >= sounder_freqs[ ifrq]) && (act_snd_data->freq <= sounder_freqs[ ifrq] + frqrng) )
         break;
     }
     iscat_percent[ ifrq][act_snd_data->beam_num]= 100.0* (float)(returns - gscat_returns)/ act_snd_data->nrang;
    }
  }
}

/****************** function average_scatter_percentage *************************/
/* what is fw good for ?? */

void average_scatter_percentages( float iscat_percent[ MAX_SND_FREQS][ SND_NBM], int sft, float average_iscat_percent[ MAX_SND_FREQS], int fw )
{
int ifrq, ibm;

  /* average the scatter percentages over all beams */
  for( ifrq=0; ifrq < sft; ifrq++ ) {
    average_iscat_percent[ifrq]= 0.0;
    for( ibm=0; ibm < SND_NBM; ibm++ )
      average_iscat_percent[ ifrq]+= iscat_percent[ ifrq][ ibm];
    average_iscat_percent[ ifrq]/= 16.0;
    if( fw ) average_iscat_percent[ ifrq]*= ifrq;
  }
}

/******************* function find_optimal_freq_local() ************************/
/* our frequency optimization scheme is this:                          */
/*                                                                     */
/* at each frequency, find the most recent set of range data for each  */
/* beam. filter this data and then find the percentage of data points  */
/* that were ionospheric backscatter. to get the optimal frequency,    */
/* average over all beams at each frequency and look for the freq      */
/* with the highest percentage of ionsopheric backscatter.             */

int find_optimal_freq(int start_freq, int cutlass, int fw, int sounder_freqs[], int sft, struct sounder_struct *sounder_data, int act_snd_rec)
{
  int i,j;
  int def_freq, def_freq_bin;
  /* array of iono scatter percentages (freq and beam number) */
  float iscat_percent[ MAX_SND_FREQS][ SND_NBM];
  float average_iscat_percent[ MAX_SND_FREQS];
  float max_scatter=-10;
  int max_freq_bin=0;
  int dlf;
  FILE *out;
 

  /* set the optimal freq to the sounder frequency closest to start_freq in case we don't find a good one; 20060308 DAndre */
  if( !cutlass ) {
    dlf= 100000;
    for ( i= 0; i < sft; i++) { 
      if ( fabs( start_freq - sounder_freqs[ i]) < dlf) {
        dlf= fabs( start_freq - sounder_freqs[ i]);
        def_freq_bin= i;
      }
    }
  }
/*
  else
    def_freq_bin=lsfreq[def_freq]/1000;
*/

  /* initialize the arrays */
  for( i=0; i<MAX_SND_FREQS; i++ )
    for( j=0; j<SND_NBM; j++ ) {
      iscat_percent[i][j]=0;
    }

  compute_scatter_percentage( sounder_data, act_snd_rec, sounder_freqs, sft, iscat_percent);
  average_scatter_percentages( iscat_percent, sft, average_iscat_percent, fw );
  max_scatter=-10;
  for( i=0; i< sft; i++ )
    if( average_iscat_percent[i] >= max_scatter ) {
      max_scatter= average_iscat_percent[ i];
      max_freq_bin=i;
    }

  /* set a threshold on the improvement */
  if( max_scatter < 1.15* average_iscat_percent[def_freq_bin] || ( max_scatter - average_iscat_percent[def_freq_bin]) < 0.75 )
    max_freq_bin=def_freq_bin;

  def_freq= sounder_freqs[ max_freq_bin];

  out=fopen("/tmp/freq_search.out","w");
  if( out != NULL ) {
    for( i= 0; i < sft; i++ ) {
      if( !fw ) 
	fprintf(out,"\n%2d %d %10.7lf", i, sounder_freqs[ i], average_iscat_percent[i]); 
      else
	fprintf(out,"\n%2d %d %10.7lf", i, sounder_freqs[ i], average_iscat_percent[i]/ i); 
    }
    fprintf( out, "\nCutlass: %d",cutlass);
    fprintf( out, "\nFreq Weighting: %d",fw);
    fprintf( out, "\nReturned Frequency: %d kHz",def_freq);
    fprintf( out, "\n");
    fclose(out);
  }

  return(def_freq);
}






