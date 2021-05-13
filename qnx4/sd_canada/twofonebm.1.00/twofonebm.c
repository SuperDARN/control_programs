/* twofonebm.c
   ============
   Author: Kevin Krieger
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
 $Log: twofonebm.c,v $
 Revision 1.0  2013/01/11 20:00:00 KKrieger
 twofonebm uses two alternating frequencies 
 on one beam with 3 second integration time.
 Frequency is switched every 3 seconds.
 It uses only the 7 pulse sequence.
*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: twofonebm.c,v 1.0 2013/01/11 18:00:00 KKrieger Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]= { TASK_NAMES, 0};

int arg=0;
struct OptionData opt;

int main(int argc,char *argv[]) {

/* Lag table */	
	int ptab[7] = {0, 9, 12, 20, 22, 26, 27};

	int lags[LAG_SIZE][2] = {
		{ 0, 0},		/* 0 */
		{26,27},		/* 1 */
		{20,22},		/* 2 */
		{ 9,12},		/* 3 */
		{22,26},		/* 4 */
		{22,27},		/* 5 */
		{20,26},		/* 6 */
		{20,27},		/* 7 */
		{12,20},		/* 8 */
		{ 0, 9},		/* 9 */
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
 
  /* How long does a round of scanning last?
   * scnsc is seconds
   * scnus is microseconds */
  int scnsc=120;
  int scnus=0;

  /* skip how many beams? */
  int skip;

  /* Counter for cross correlation btw main and int*/
  int cnt=0;

  /* How many scans in scan window? */
  int num_scans = 40;

  /* Station's ID */
  int st_id;

  /* Frequencies to use */
  int transmit_freqs[2] = {10500, 12500};

  /* Beam number to transmit on */
  int transmit_beam = 0;

  unsigned char discretion=0;

  /* Get all of the command line arguments into a buffer to parse */
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

  st_id= atoi( getenv("SD_RADARID"));

  /* CPID of 3550 for twofonebm */
  cp= 3550;
  SiteStart();

  /* Integration time */
  intsc= 2;
  intus= 900000;

  /* 7 pulse sequence, 18 lags, 2400 us pulse increment time */
  mppul= 7;
  mplgs= 18;
  mpinc= 2400;

  /* 75 range gates, 45 km range gate separation, 300 us pulse length */
  nrang=75;
  rsep= 45;
  txpl= 300;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */


  OptionAdd(&opt,"di",'x',&discretion);

  OptionAdd(&opt,"el",'t',&ename);
  OptionAdd(&opt,"sc",'t',&sname);

  OptionAdd( &opt,"frang",'i',&frang);
  OptionAdd( &opt,"rsep",'i',&rsep);
  OptionAdd( &opt,"nrang",'i',&nrang);
  OptionAdd( &opt,"xcf",'i',&xcnt);

  OptionAdd( &opt,"bm", 'i', &transmit_beam);
  OptionAdd( &opt,"freq0",'i', &transmit_freqs[0]);
  OptionAdd( &opt,"freq1",'i', &transmit_freqs[1]);

  arg= OptionProcess( 1, argc, argv, &opt, NULL);

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);
  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();
  if ( discretion) cp= -cp;
  txpl=(rsep*20)/3;
  sprintf(progname,"twofonebm");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);
  }

  /* Loop until this RCP is done */
  do {

    if (SiteStartScan()== 0) continue;

	/* Is it time for a new data file? */
    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(tlist[n]);
        RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
      }
    }

    scan=1;

    ErrLog(errlog,progname,"Starting scan.");

	/* Do we perform cross correlation? */
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;


	/* Find out how many beams we need to skip.
 	 * tv is the time past the hour in microseconds.
 	 * bv is the length of time we scan for in us
 	 * iv is the integration time in us 
 	 * skip is the number of scans already completed during this
 	 * scan window. */
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
	/* Set the beamnumber to the desired transmit beam */
	bmnum = transmit_beam;

	/* Loop until we have completed all transmissions in this scan window  */
    do {

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

	  /* Alternate the transmit frequency */
      stfrq= transmit_freqs[skip % 2];

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
      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);
      RMsgSndAdd(&msg, strlen(sharedmemory)+1, sharedmemory, IQS_TYPE, 0);

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

