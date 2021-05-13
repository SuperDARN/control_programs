/* twotsg.c
   ============
   Author: Dieter Andre
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
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
 $Log: twotsg.c,v $
 Revision 1.01 2013/02/12 16:00:00 KKrieger
 Fix for 7 pulse lag table: lag 4
 Moved SiteSetIntt after SiteFCLR; DAndre
 
 $Log: twotsg.c,v $
 Revision 1.00  2013/01/16 20:00:00 DAndre
 Initial revision
 
*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: twotsg.c,v 1.01 2013/02/12 20:00:00 code Exp $"};
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


  int ptab_7[7] = { 0, 9, 12, 20, 22, 26, 27};

  int lags_7[LAG_SIZE][2] = {
    { 0, 0},		/*  0 */
    {26,27},		/*  1 */
    {20,22},		/*  2 */
    { 9,12},		/*  3 */
    {22,26},		/*  4 */
    {22,27},		/*  5 */
    {20,26},		/*  6 */
    {20,27},		/*  7 */
    {12,20},		/*  8 */
    { 0, 9},		/*  9 */
    {12,22},		/* 10 */
    { 9,20},		/* 11 */
    { 0,12},		/* 12 */
    { 9,22},		/* 13 */
    {12,26},		/* 14 */
    {12,27},		/* 15 */

    { 9,26},		/* 17 */
    { 9,27},		/* 18 */

    {27,27}};		/* alternate lag-0 */

  char *sname=NULL;
  char *ename=NULL;
  char *sdname={SCHEDULER};
  char *edname={ERRLOG};
  char logtxt[1024];

  int n;
  pid_t sid;
  int exitpoll=0;

  int cnt=0;
  int num_scans;
  int scnsc;
  int scnus;
  int skip;
  int fast;
  int mppul_8;
  int mplgs_8;
  int mpinc_8;
  int mppul_7;
  int mplgs_7;
  int mpinc_7;


  unsigned char discretion=0;

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

  mppul_8= 8;
  mplgs_8= 23;
  mpinc_8= 1500;

  mppul_7= 7;
  mplgs_7= 18;
  mpinc_7= 2400;

  nrang=75;
  rsep=45;
  txpl=300;

  SiteStart();

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */


  OptionAdd(&opt,"di",'x',&discretion);


  OptionAdd(&opt,"el",'t',&ename);
  OptionAdd(&opt,"sc",'t',&sname);
  OptionAdd(&opt,"rsep",'i',&rsep);
  OptionAdd( &opt, "dt", 'i', &day);
  OptionAdd( &opt, "nt", 'i', &night);
  OptionAdd( &opt, "df", 'i', &dfrq);
  OptionAdd( &opt, "nf", 'i', &nfrq);
  OptionAdd( &opt, "dr", 'i', &dfrang);
  OptionAdd( &opt, "nr", 'i', &nfrang);
  OptionAdd( &opt, "xcf", 'i', &xcnt);

  OptionAdd( &opt,"fast",'x',&fast);
  OptionAdd( &opt, "bm", 'i', &bmnum);

  arg=OptionProcess(1,argc,argv,&opt,NULL);

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  if (fast) {
    cp=3253;
    num_scans= 16;
    scnsc=60;
    scnus=0;
    intsc=3;
    intus=500000;
    sprintf(progname,"twotsg (fast)");
  }
  else {
    cp=3252;
    num_scans= 16;
    scnsc=120;
    scnus=0;
    intsc=7;
    intus=250000;
    sprintf(progname,"twotsg");
  };

  if ( discretion) cp= -cp;
  txpl=(rsep*20)/3;


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

    scan=1;

    ErrLog(errlog,progname,"Starting scan.");

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    /* skip=OpsFindSkip(scnsc,scnus); */
    /* I did not want to modify OpsFindSkip */
    /* but skip is now the index into the beam list */
    /* and has to run to num_scans - 1 */
    {
      int tv;
      int bv;
      int iv;
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      iv= intsc*1000000 + intus;
      bv= scnsc* 1000000 + scnus;
      tv=(mt* 60 + sc)* 1000000 + us + iv/2 - 100000;
      skip=(tv % bv)/iv;
      if (skip> num_scans-1) skip=0;
      if (skip<0) skip=0;
    }


    do {

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
 
      if (OpsDayNight()==1) {
        stfrq=dfrq;
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
        frang=nfrang;
      }        

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog,progname,logtxt);

      ErrLog(errlog,progname,"Setting beam.");
      SiteSetBeam(bmnum);

      /* only search for clear frequency once for the two pulse sequences */
      if ( (skip % 2) == 0 ) {
        ErrLog(errlog,progname,"Doing clear frequency search."); 
        sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
        ErrLog( errlog, progname, logtxt);
        if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
          ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
        SiteSetFreq(tfreq);
      }

      SiteSetIntt( intsc, intus);
      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog,progname,logtxt);

      if ( (skip % 2) == 0 ) {
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
      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prm,PRM_TYPE,0);
      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);
      RMsgSndAdd(&msg,strlen(sharedmemory)+1, sharedmemory, IQS_TYPE,0);
      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &raw,RAW_TYPE,0);
      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fit,FIT_TYPE,0);
      RMsgSndAdd(&msg,strlen(progname)+1,progname,NME_TYPE,0);
      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 

      ErrLog(errlog,progname,"Polling for exit."); 
      exitpoll=RadarShell(sid,&rstable);
      if (exitpoll !=0) break;
      scan=0;
      if (skip == (num_scans-1)) break;
      skip= skip + 1;

    } while (1);
    ErrLog(errlog,progname,"Waiting for scan boundary."); 
  
    if (exitpoll==0) OpsWaitBoundary(scnsc,scnus);

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
} 
 
