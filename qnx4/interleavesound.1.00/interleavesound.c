/* interleavesound.c
   =================
   Author: E.G.Thomas
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/kernel.h>
#include <string.h>
#include <time.h>
#include "rtypes.h"
#include "dmap.h"
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

#include "sndwrite.h"

/*
 $Log: interleavesound.c,v $
 Revision 1.0  2019/06/14 egthomas
 Initial revision from interleavedscan and normalsound
 
*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: interleavesound.c,v 1.0 2019/06/14 egthomas Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;

#define MAX_SND_FREQS 12

int main(int argc,char *argv[]) {

  /* The pulse sequence table and lags for katscan */ 
  int ptab[8] = {0,14,22,24,27,31,42,43};

  int lags[LAG_SIZE][2] = {
    { 0, 0},		/*  0 */
    {42,43},		/*  1 */
    {22,24},		/*  2 */
    {24,27},		/*  3 */
    {27,31},		/*  4 */
    {22,27},		/*  5 */

    {24,31},		/*  7 */
    {14,22},		/*  8 */
    {22,31},		/*  9 */
    {14,24},		/* 10 */
    {31,42},		/* 11 */
    {31,43},		/* 12 */
    {14,27},		/* 13 */
    { 0,14},		/* 14 */
    {27,42},		/* 15 */
    {27,43},		/* 16 */
    {14,31},		/* 17 */
    {24,42},		/* 18 */
    {24,43},		/* 19 */
    {22,42},		/* 20 */
    {22,43},		/* 21 */
    { 0,22},		/* 22 */

    { 0,24},		/* 24 */

    {43,43}};		/* alternate lag-0  */


  char *sname=NULL;
  char *ename=NULL;
  char *sdname={SCHEDULER};
  char *edname={ERRLOG};
  char logtxt[1024];

  int n;
  pid_t sid;
  int exitpoll=0;

  int scnsc=60;
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char discretion=0;

  /* ---------- Beam sequence for interleavedscan ---------- */
  /*
    forward_beams[]/backward_beams[] are for scan in a clockwise/counterclockwise direction. 
    Please choose one of the num_scans-forward/backward_scans[] sets below 
    according to the beam number of the radar. 
  */

  /* For a 16-beam radar */
  int num_scans = 16;
  int forward_beams[16] = { 0,4,8,12, 2,6,10,14, 1,5,9,13, 3,7,11,15 };
  int backward_beams[16]= { 15,11,7,3, 13,9,5,1, 14,10,6,2, 12,8,4,0 };
  int snd_bms[8] = {0,2,4,6,8,10,12,14};

  /* For an eastward-looking radar with 20- or more beams (using only 20 beams to complete every 1 min) */ 
  /*
  int num_scans = 20;
  int forward_beams[20] = { 0,4,8,12,16, 2,6,10,14,18, 1,5,9,13,17, 3,7,11,15,19 };
  int backward_beams[20]= { 19,15,11,7,3, 17,13,9,5,1, 18,14,10,6,2, 16,12,8,4,0 };
  int snd_bms[10] = {0,2,4,6,8,10,12,14,16,18};
  */

  /* For an westward-looking radar with 20- or more beams (using only 20 beams to complete every 1 min) */ 

  /* max beam number: 23 (24-beam) */
  /*
  int num_scans = 20;
  int forward_beams[20] = { 4,8,12,16,20, 6,10,14,18,22, 5,9,13,17,21, 7,11,15,19,23 };
  int backward_beams[20]= { 23,19,15,11,7 ,21,17,13,9,5, 22,18,14,10,6, 20,16,12,8,4 };
  int snd_bms[10] = {22,20,18,16,14,12,10,8,6,4};
  */

  /* max beam number: 21 (22-beam) */ 
  /*
  int num_scans = 20;
  int forward_beams[20] = { 2,6,10,14,18, 4,8,12,16,20, 3,7,11,15,19, 5,9,13,17,21 };
  int backward_beams[20]= { 21,17,13,9,5, 19,15,11,7,3, 20,16,12,8,4, 18,14,10,6,2 };
  int snd_bms[10] = {20,18,16,14,12,10,8,6,4,2};
  */

  /* ------------------------------------------------------- */ 
  int bmseqnum = 0;


  /* ---------------- Variables for sounding --------------- */
  char snd_filename[100];
  FILE *snd_dat;
  /* If the file $SD_HDWPATH/sounder_[rad].dat exists, the next two parameters are read from it */
  /* the file contains one integer value per line */
  int snd_freqs_tot=8;
  int snd_freqs[MAX_SND_FREQS] = {11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0};
  int snd_freq_cnt=0, snd_bm_cnt=0;
  int odd_beams=0;
  int snd_freq;
  int snd_frqrng=100;
  float snd_time, snd_intt, time_needed=1.25;

  if (num_scans == 16) {
    int snd_bms_tot=8;
    int fast_intt_sc=3;
    int fast_intt_us=0;
    int snd_intt_sc=2;
    int snd_intt_us=0;
  else if (num_scans == 20) {
    int snd_bms_tot=10;
    int fast_intt_sc=2;
    int fast_intt_us=5;
    int snd_intt_sc=1;
    int snd_intt_us=5;
  }

  snd_intt = snd_intt_sc + snd_intt_us*1e-6;

  /* load the sounder frequencies from file if present */
  sprintf(snd_filename, "%s/sounder_%s.dat", getenv("SD_HDWPATH"), getenv("SD_RADARCODE"));
  fprintf(stderr, "Checking Sounder File: %s\n", snd_filename);
  snd_dat = fopen(snd_filename, "r");
  if (snd_dat != NULL) {
    fscanf(snd_dat, "%d", &snd_freqs_tot);
    if (snd_freqs_tot > 12) snd_freqs_tot = 12;
    for (snd_freq_cnt=0; snd_freq_cnt < snd_freqs_tot; snd_freq_cnt++)
      fscanf(snd_dat, "%d", &snd_freqs[snd_freq_cnt]);
    snd_freq_cnt = 0;
    fclose(snd_dat);
    fprintf(stderr,"Sounder File: %s read\n", snd_filename);
  } else {
    fprintf(stderr,"Sounder File: %s not found\n", snd_filename);
  }
  /* ------------------------------------------------------- */


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

  // For 1-min normal scan
  cp     = 197;
  intsc  = fast_intt_sc;
  intus  = fast_intt_us;
  mppul  = 8;
  mplgs  = 23;
  mpinc  = 1500;
  dmpinc = 1500;
  nrang  = 75;
  rsep   = 45;
  txpl   = 300; /* recalculated below with rsep */
  frang  = 180;

  SiteStart();

#if 1
  //maxatten=1;	//Chris
  //maxatten=2;	//Sessai
  //maxatten=1;	//Sessai
#endif

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "el", 't', &ename);
  OptionAdd(&opt, "sc", 't', &sname);

  OptionAdd(&opt, "nrang", 'i', &nrang);
  OptionAdd(&opt, "frang", 'i', &frang);
  OptionAdd(&opt, "rsep", 'i', &rsep);

  OptionAdd(&opt, "di", 'x', &discretion);
  OptionAdd(&opt, "dt", 'i', &day);
  OptionAdd(&opt, "nt", 'i', &night);
  OptionAdd(&opt, "df", 'i', &dfrq);
  OptionAdd(&opt, "nf", 'i', &nfrq);
  OptionAdd(&opt, "xcf", 'i', &xcnt);
  OptionAdd(&opt, "frqrng", 'i', &frqrng);
  OptionAdd(&opt, "sfrqrng", 'i',&snd_frqrng); /* sounding FCLR window [kHz] */

  arg=OptionProcess(1,argc,argv,&opt,NULL);

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  // set a negative CPID for discretionary time
  if (discretion) cp = -cp;

  // recalculate txpl
  txpl=(rsep*20)/3;

  // set the program name
  sprintf(progname,"interleavesound");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);
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

    scan = 1;

    ErrLog(errlog,progname,"Starting scan.");

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    skip=OpsFindSkip(scnsc,scnus);

    bmseqnum=skip;
    if (backward) {
      bmnum=backward_beams[bmseqnum];
      if (bmnum<ebm) bmnum=sbm;
    } else {
      bmnum=forward_beams[bmseqnum];
      if (bmnum>ebm) bmnum=sbm;
    }

    do {
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
        mpinc=dmpinc;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,
                      intsc,intus,hr,mt,sc,us);
      ErrLog(errlog,progname,logtxt);

      ErrLog(errlog,progname,"Setting beam.");

      SiteSetIntt(intsc,intus);
      SiteSetBeam(bmnum);

      ErrLog(errlog,progname,"Doing clear frequency search.");

      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog, progname, logtxt);

      if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

      SiteSetFreq(tfreq);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);

      ErrLog(errlog,progname,logtxt);

      tsgid=SiteTimeSeq(ptab);

      nave=SiteIntegrate(lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog,progname,logtxt);
        continue;
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
      if (bmseqnum==(num_scans-1)) break;

      /* Change the beamnum to the next beam */
      bmseqnum++;

      if (backward) {
        bmnum=backward_beams[bmseqnum];
        if (bmnum<ebm) bmnum=sbm;
      } else {
        bmnum=forward_beams[bmseqnum];
        if (bmnum>ebm) bmnum=sbm;
      }

    } while (1);
    ErrLog(errlog,progname,"Waiting for scan boundary.");

    if (exitpoll == 0) {
      /* In here comes the sounder code */
      /* set the "sounder mode" scan variable */
      scan = -2;

      /* set the xcf variable to do cross-correlations (AOA) */
      xcf = 1;

      /* we have time until the end of the minute to do sounding */
      /* minus a safety factor given in time_needed */
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      snd_time = 60.0 - (sc + us*1e-6);

      while (snd_time-snd_intt > time_needed) {
        intsc = snd_intt_sc;
        intus = snd_intt_us;

        /* set the beam */
        bmnum = snd_bms[snd_bm_cnt] + odd_beams;

        /* snd_freq will be an array of frequencies to step through */
        snd_freq = snd_freqs[snd_freq_cnt];

        /* the scanning code here */
        sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,intsc,intus,hr,mt,sc,us);
        ErrLog(errlog,progname,logtxt);
        ErrLog(errlog,progname,"Setting SND beam.");
        SiteSetIntt(intsc,intus);
        SiteSetBeam(bmnum);
        ErrLog(errlog, progname, "Doing SND clear frequency search.");
        if (SiteFCLR(snd_freq, snd_freq + snd_frqrng)==FREQ_LOCAL)
          ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
        SiteSetFreq(tfreq);
/*
        sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
        ErrLog(errlog, progname, logtxt);
*/
        tsgid = SiteTimeSeq(ptab);
        nave = SiteIntegrate(lags);
        if (nave < 0) {
          sprintf(logtxt, "SND integration error: %d", nave);
          ErrLog(errlog,progname, logtxt);
          continue;
        }
        sprintf(logtxt,"Number of SND sequences: %d",nave);
        ErrLog(errlog,progname,logtxt);

        OpsBuildPrm(&prm,ptab,lags);
        OpsBuildIQ(&iq);
        OpsBuildRaw(&raw);

        FitACF(&prm,&raw,&fblk,&fit);

        ErrLog(errlog, progname, "Sending SND messages.");
        msg.num = 0;
        msg.tsize = 0;
        RMsgSndAdd(&msg, sizeof(struct RadarParm), (unsigned char *) &prm, PRM_TYPE, 0);

        RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);

        RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);

        RMsgSndAdd(&msg, sizeof(struct RawData), (unsigned char *) &raw, RAW_TYPE, 0);
        RMsgSndAdd(&msg, sizeof(struct FitData), (unsigned char *) &fit, FIT_TYPE, 0);
        RMsgSndAdd(&msg, strlen(progname)+1, progname, NME_TYPE, 0);

        /* Only send these to echo_data; otherwise they get written to the data files */
        RMsgSndSend(tlist[0], &msg);

        sprintf(logtxt, "SBC: %d  SFC: %d\n", snd_bm_cnt, snd_freq_cnt);
        ErrLog(errlog, progname, logtxt);

        /* set the scan variable for the sounding mode data file only */
        if ((bmnum == snd_bms[0]) && (snd_freq == snd_freqs[0])) {
          prm.scan = 1;
        } else {
          prm.scan = 0;
        }

        /* save the sounding mode data */
        write_snd_record(progname, &prm, &fit);

        ErrLog(errlog, progname, "Polling SND for exit.");
        exitpoll=RadarShell(sid,&rstable);
        if (exitpoll !=0) break;

        /* check for the end of a beam loop */
        snd_freq_cnt++;
        if (snd_freq_cnt >= snd_freqs_tot) {
          /* reset the freq counter and increment the beam counter */
          snd_freq_cnt = 0;
          snd_bm_cnt++;
          if (snd_bm_cnt >= snd_bms_tot) {
            snd_bm_cnt = 0;
            odd_beams = !odd_beams;
          }
        }

        /* see if we have enough time for another go round */
        TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);
        snd_time = 60.0 - (sc + us*1e-6);
      }

      /* now wait for the next interleavescan */
      intsc = fast_intt_sc;
      intus = fast_intt_us;
      OpsWaitBoundary(scnsc,scnus);
    }

    bmseqnum=0;

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
} 


/********************** function write_snd_record() ************************/
/* changed the output to dmap format */

void write_snd_record(char *progname, struct RadarParm *prm, struct FitData *fit) {

  char data_path[100], data_filename[50], filename[80];

  char *snd_dir;
  FILE *out;

  char logtxt[1024];
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, getenv("SD_RADARCODE"));

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding file:%s",filename);
    ErrLog(errlog,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = SndFwrite(out, prm, fit);
  if (status == -1) {
    ErrLog(errlog,progname,"Error writing sounding record.");
  }

  fclose(out);
}
