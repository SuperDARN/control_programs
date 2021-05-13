/* rbspscan.c
   ============
   Author: Dieter Andre
*/


#include <i86.h>
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
 $Log: rbspscan.c,v $
 Revision 1.10 2014/12/24 20:00:00 KKrieger
 Added total scan time checking to prevent
 delays at the end of a scan

 Revision 1.9  2012/11/29 20:00:00  DAndre
 Added options dfrang and nfrang
 Removed option frang

 Revision 1.8  2012/10/29 09:17:00  DAndre
 This is the canadian version of rbspscan
 with interleaved camping beams

*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: rbspscan.c,v 1.10 2014/12/24 20:00:00 KKrieger Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]= { TASK_NAMES, 0};

int arg=0;
struct OptionData opt;

int main(int argc,char *argv[]) {

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
 
  int scnsc=120;
  int scnus=0;
  double totalscantime = 0;
  time_t scanstarttime;
  time_t scanstoptime;
  int skip;
  int cnt=0;
  int fixfrq=0;

  int eastbm=2;	/* east beam */
  int meribm=3;	/* meridional beam */
  int westbm=5;	/* west beam */

  int num_scans= 31;
  /* beams for forward and backward scanning radars; -1 will be replaced by the selected camping beam */
  int forward_beams[ 31]=  {   0,  -1,   1,  -1,   2,  -1,   3,  -1,   4,  -1,   5,  -1,   6,  -1,   7,  -1,   8,  -1,
                               9,  -1,  10,  -1,  11,  -1,  12,  -1,  13,  -1,  14,  -1,  15 };
  int backward_beams[ 31]= {  15,  -1,  14,  -1,  13,  -1,  12,  -1,  11,  -1,  10,  -1,   9,  -1,   8,  -1,   7,  -1,
                               6,  -1,   5,  -1,   4,  -1,   3,  -1,   2,  -1,   1,  -1,   0 };

  unsigned char discretion=0;

  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 

  strncpy(combf,progid,80);

  OpsSetupRadar();
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
                  &sbm, &ebm, &dfrq, &nfrq, &dfrang, &nfrang, &dmpinc, &nmpinc, &frqrng, &xcnt);

  cp= 200;
  SiteStart();


  intsc= 3;
  intus= 750000;
  mppul=8;
  mplgs=23;
  mpinc=1500;
  nmpinc=1500;
  dmpinc=1500;
  nrang=75;
  rsep=45;
  txpl=300;
  sbm=15;
  ebm=1;


  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */


  OptionAdd(&opt,"di",'x',&discretion);


  OptionAdd(&opt,"el",'t',&ename);
  OptionAdd(&opt,"sc",'t',&sname);

  OptionAdd(&opt,"frang",'i',&frang);
  OptionAdd(&opt,"rsep",'i',&rsep);

  OptionAdd( &opt, "dt", 'i', &day);
  OptionAdd( &opt, "nt", 'i', &night);
  OptionAdd( &opt, "df", 'i', &dfrq);
  OptionAdd( &opt, "nf", 'i', &nfrq);
  OptionAdd( &opt, "dr", 'i', &dfrang);
  OptionAdd( &opt, "nr", 'i', &nfrang);
  OptionAdd( &opt, "xcf", 'i', &xcnt);
  /* Transmit at this frequency */
  OptionAdd( &opt, "fixfrq", 'i', &fixfrq);
  /* Camping Beams */
  OptionAdd( &opt, "meribm",'i', &meribm);		/* Meridional beam */
  OptionAdd( &opt, "westbm",'i', &westbm);		/* West beam */
  OptionAdd( &opt, "eastbm",'i', &eastbm);		/* East beam */
  OptionAdd( &opt, "nrang", 'i', &nrang);

  arg= OptionProcess( 1, argc, argv, &opt, NULL);

  /* fill in the camping beams */
  forward_beams[  1]= westbm;
  forward_beams[  3]= meribm;
  forward_beams[  5]= eastbm;
  forward_beams[  7]= westbm;
  forward_beams[  9]= meribm;
  forward_beams[ 11]= eastbm;
  forward_beams[ 13]= westbm;
  forward_beams[ 15]= meribm;
  forward_beams[ 17]= eastbm;
  forward_beams[ 19]= westbm;
  forward_beams[ 21]= meribm;
  forward_beams[ 23]= eastbm;
  forward_beams[ 25]= westbm;
  forward_beams[ 27]= meribm;
  forward_beams[ 29]= eastbm;

  backward_beams[  1]= eastbm;
  backward_beams[  3]= meribm;
  backward_beams[  5]= westbm;
  backward_beams[  7]= eastbm;
  backward_beams[  9]= meribm;
  backward_beams[ 11]= westbm;
  backward_beams[ 13]= eastbm;
  backward_beams[ 15]= meribm;
  backward_beams[ 17]= westbm;
  backward_beams[ 19]= eastbm;
  backward_beams[ 21]= meribm;
  backward_beams[ 23]= westbm;
  backward_beams[ 25]= eastbm;
  backward_beams[ 27]= meribm;
  backward_beams[ 29]= westbm;
 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  sprintf(progname,"rbspscan");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);
  }

  do {
	scanstarttime = time(NULL);
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
   sprintf( logtxt, "Beam skip: %d", skip);
   ErrLog( errlog, progname, logtxt);

    if (backward) {
      bmnum= backward_beams[ skip];
    } else {
      bmnum=  forward_beams[ skip];
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

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d) %d",bmnum,intsc,intus,hr,mt,sc,us, skip );
      ErrLog(errlog,progname,logtxt);

      ErrLog(errlog,progname,"Setting beam.");

      SiteSetIntt(intsc,intus);
      SiteSetBeam(bmnum);

      ErrLog(errlog,progname,"Doing clear frequency search."); 
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog( errlog, progname, logtxt);

      if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

      if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq= fixfrq;
      SiteSetFreq(tfreq);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog,progname,logtxt);

      tsgid=SiteTimeSeq(ptab);

      nave=SiteIntegrate(lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog,progname,logtxt);
        /* restart the radar */
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
 
      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq,
                 IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,
                 IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &raw, RAW_TYPE,0);
      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fit, FIT_TYPE,0);
      RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,0);
      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 

      ErrLog(errlog,progname,"Polling for exit.");
      exitpoll=RadarShell(sid,&rstable);
      if (exitpoll !=0) break;
      scan=0;
      if (skip == (num_scans-1)) break;
      skip= skip + 1;
      if (backward) {
        bmnum= backward_beams[ skip];
      } else {
        bmnum=  forward_beams[ skip];
      }
    } while (1);
	scanstoptime = time(NULL);
	totalscantime = difftime(scanstoptime, scanstarttime);
	if(totalscantime >= scnsc) {
		sprintf(logtxt, "Scan time over limit! %f seconds. Continuing", totalscantime);
		ErrLog(errlog, progname, logtxt);
	} else {
		sprintf(logtxt, "Scan total time: %f seconds", totalscantime);
		ErrLog(errlog,progname,logtxt);
    	ErrLog(errlog,progname,"Waiting for scan boundary."); 
		if (exitpoll==0) OpsWaitBoundary(scnsc,scnus);
	}

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
}

