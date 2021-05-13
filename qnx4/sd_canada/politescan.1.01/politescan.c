/* politescan.c
   ============
   Author: Kevin Krieger
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
#include "freq.h"

/*
 $Log: politescan.c,v $
 Revision 1.01 2015/01/26 17:00 KKrieger
 Modified to add fixed frequency option  

 Revision 1.00 2014/04/14 17:00 KKrieger
 Adapted from twofsound 1.02 
 
*/

/*===========================================================================*/
/* Program Description:									*/
/* This version of politescan has been adapted from twofsound			*/
/* It does everything the same, but uses SiteSetFreqQuiet instead.		*/
/* in order to prevent emissions. */
/* twofsound performs a scan through all 16 beams at 7.25s or 3.5s.		*/
/* integration time at two alternating fixed frequencies.				*/
/*===========================================================================*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: politescan.c,v 1.01 2015/01/26 17:00:00 KKrieger Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;


int main(int argc,char *argv[]) {

  int ptab_8[8] = {0,14,22,24,27,31,42,43};

  int lags_8[LAG_SIZE][2] = {
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

  int mppul_8= 8;
  int mplgs_8= 23;
  int mpinc_8= 1500;
  int dmpinc_8= 1500;
  int nmpinc_8= 1500;

  int ptab_7[7] = { 0, 9, 12, 20, 22, 26, 27};

  int lags_7[LAG_SIZE][2] = {
    { 0, 0},        /*  0 */
    {26,27},        /*  1 */
    {20,22},        /*  2 */
    { 9,12},        /*  3 */
    {22,26},        /*  4 */
    {22,27},        /*  5 */
    {20,26},        /*  6 */
    {20,27},        /*  7 */
    {12,20},        /*  8 */
    { 0, 9},        /*  9 */
    {12,22},        /* 10 */
    { 9,20},        /* 11 */
    { 0,12},        /* 12 */
    { 9,22},        /* 13 */
    {12,26},        /* 14 */
    {12,27},        /* 15 */

    { 9,26},        /* 17 */
    { 9,27},        /* 18 */

    {27,27}};       /* alternate lag-0 */

  int mppul_7= 7;
  int mplgs_7= 18;
  int mpinc_7= 2400;
  int dmpinc_7= 2400;
  int nmpinc_7= 2400;

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
  unsigned char p7=0;
  
  /* Fixed frequency to listen to */
  int fixfrq = 0;

  /* variables for twofsound */
  int st_id; /* Station id */
  int fsel= 0; /* frequency selection */
  /* Frequencies used for the radars */
  int sas_freq[ 2]= { 10500, 13000};
  int pgr_freq[ 2]= { 10500, 13000};
  int rkn_freq[ 2]= { 10200, 12200};
  int inv_freq[ 2]= { 10300, 12200};
  int cly_freq[ 2]= { 10500, 12500};

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

  st_id= atoi( getenv("SD_RADARID"));
  nrang=75;
  rsep=45;
  txpl=300;
  dfrang= 180;
  nfrang= 180;

  SiteStart();

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
  OptionAdd( &opt, "nrang", 'i', &nrang);
  OptionAdd( &opt, "rsep", 'i', &rsep);
  OptionAdd(&opt,"fast", 'x', &fast);
  OptionAdd(&opt,"p7", 'x', &p7);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);
  OptionAdd(&opt, "fixfreq", 'i', &fixfrq);

  arg=OptionProcess(1,argc,argv,&opt,NULL);  

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);  

  SiteSetupHardware();

  if (fast) {
     cp= 3380;
     scnsc=60;
     scnus=0;
     intsc= 3;
     intus= 500000;
  } else {
     cp= 3380;
     scnsc= 120;
     scnus=0;
     intsc= 7;
     intus= 250000;
  }
  if ( p7 == 0 ) {
    mppul= mppul_8;
    mplgs= mplgs_8;
    dmpinc= dmpinc_8;
    nmpinc= nmpinc_8;
  } else {
    mppul= mppul_7;
    mplgs= mplgs_7;
    dmpinc= dmpinc_7;
    nmpinc= nmpinc_7;
  }

  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  if (fast) sprintf(progname,"politescan (fast)");
  else sprintf(progname,"politescan");

  OpsFitACFStart();
  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
  }

  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  do {

    switch ( st_id) {
      case 5: stfrq= sas_freq[ fsel];
               break;
      case 6: stfrq= pgr_freq[ fsel];
               break;
      case 64: stfrq= inv_freq[ fsel];
               break;
      case 65: stfrq= rkn_freq[ fsel];
               break;
      case 66: stfrq= cly_freq[ fsel];
               break;
    }

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

	  if(fixfrq != 0) {
		if(FreqTest(ftable, fixfrq) != 0) {
			/* If our fixed frq is restriced print error and continue normally*/
			sprintf(logtxt,"Error: %d kHz is restricted, using default parameters.",fixfrq);
			ErrLog(errlog,progname,logtxt);
     	 	ErrLog(errlog,progname,"Doing clear frequency search."); 
      		if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        		ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
		} else {
			if(SiteFCLR(fixfrq, fixfrq+10) == FREQ_LOCAL)
				ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
			tfreq = fixfrq;
		}
	} else {
		ErrLog(errlog,progname,"No fixed frequency. Doing clear frequency search.");
		if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
			ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
	}
	SiteSetFreqQuiet(tfreq);

      sprintf(logtxt,"Listening on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog,progname,logtxt);
      if ( p7 == 0 ) {
        mppul= mppul_8;
        mplgs= mplgs_8;
        mpinc= mpinc_8;
        tsgid= SiteTimeSeq(ptab_8);
        nave= SiteIntegrate(lags_8);
        if (nave<0) {
          sprintf(logtxt,"Integration error:%d",nave);
          ErrLog(errlog,progname,logtxt);
          /* restart the radar */
          spawnl( P_WAIT, "/home/radar/script/restart.radar", NULL);
          /* continue; */
          exit( nave);
        }
        sprintf(logtxt,"Number of sequences [8]: %d", nave);
        ErrLog(errlog,progname,logtxt);
        OpsBuildPrm(&prm,ptab_8,lags_8);
      } else  {
        mppul= mppul_7;
        mplgs= mplgs_7;
        mpinc= mpinc_7;
        tsgid= SiteTimeSeq(ptab_7);
        nave= SiteIntegrate(lags_7);
        if (nave<0) {
          sprintf(logtxt,"Integration error:%d",nave);
          ErrLog(errlog,progname,logtxt);
          /* restart the radar */
          spawnl( P_WAIT, "/home/radar/script/restart.radar", NULL);
          /* continue; */
          exit( nave);
        }
        sprintf(logtxt,"Number of sequences [7]: %d",nave);
        ErrLog(errlog,progname,logtxt);
        OpsBuildPrm(&prm,ptab_7,lags_7);
      }

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
      /* now wait for the next normal_scan */
      fsel= (fsel + 1) % 2;
      OpsWaitBoundary(scnsc,scnus);
    }

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
}
