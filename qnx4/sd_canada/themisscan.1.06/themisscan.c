/* themisscan.c
   ============
   Author: Dieter Andre
*/

/*
 Copyright 2004 The Johns Hopkins University/Applied Physics Laboratory.
 All rights reserved.
 
 This material may be used, modified, or reproduced by or for the U.S.
 Government pursuant to the license rights granted under the clauses at DFARS
 252.227-7013/7014.
 
 For any other permissions, please contact the Space Department
 Program Office at JHU/APL.
 
 This Distribution and Disclaimer Statement must be included in all copies of
 "Radar Operating System - Control Programs" (hereinafter "the Program").
 
 The Program was developed at The Johns Hopkins University/Applied Physics
 Laboratory (JHU/APL) which is the author thereof under the "work made for
 hire" provisions of the copyright law.  
 
 JHU/APL assumes no obligation to provide support of any kind with regard to
 the Program.  This includes no obligation to provide assistance in using the
 Program or to provide updated versions of the Program.
 
 THE PROGRAM AND ITS DOCUMENTATION ARE PROVIDED AS IS AND WITHOUT ANY EXPRESS
 OR IMPLIED WARRANTIES WHATSOEVER.  ALL WARRANTIES INCLUDING, BUT NOT LIMITED
 TO, PERFORMANCE, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE
 HEREBY DISCLAIMED.  YOU ASSUME THE ENTIRE RISK AND LIABILITY OF USING THE
 PROGRAM TO INCLUDE USE IN COMPLIANCE WITH ANY THIRD PARTY RIGHTS.  YOU ARE
 ADVISED TO TEST THE PROGRAM THOROUGHLY BEFORE RELYING ON IT.  IN NO EVENT
 SHALL JHU/APL BE LIABLE FOR ANY DAMAGES WHATSOEVER, INCLUDING, WITHOUT
 LIMITATION, ANY LOST PROFITS, LOST SAVINGS OR OTHER INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, ARISING OUT OF THE USE OR INABILITY TO USE THE
 PROGRAM."
 
 
 
 
 
 
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
 $Log: themisscan.c,v $
 Revision 1.6  2014/12/24 17:00:00 KKrieger
 Added total scan time checking to prevent
 delays at the end of a scan

 Revision 1.5  2012/11/27 20:30:00  DAndre
 Added options dfrang and nfrang
 Removed option frang

 Revision 1.4  2008/11/24 09:25:00  DAndre
 Removed changes to the scan sequence and timing.

 Revision 1.3  2008/03/15 11:22:28  code
 Added I&Q sample capture with iqwrite.

 Revision 1.2  2008/03/14 17:51:48  code
 Added support for I&Q sample capture.

 Revision 1.1  2008/03/12 16:48:35  code
 Initial revision


*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","raw_write","fit_write","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: themisscan.c,v 1.6 2014/12/24 17:00:00 KKrieger Exp $"};
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
  /* we need these to skip to the right beam on startup */
  int skipsc= 3;
  int skipus= 0;
  int cnt=0;
  int fixfrq=0;
  int camping_beam= 7; /* Default Camping Beam */

  int num_scans= 38;
  /* Second within the 2min interval at which this beam is supposed to start */
  int scan_times[ 38]=     {   0,   3,   6,   9,  12,  15,  18,  21,  24,  27,  30,  33,  36,  39,  42,  45,  48,  51,  54,  60,
                              63,  66,  69,  72,  75,  78,  81,  84,  87,  90,  93,  96,  99, 102, 105, 108, 111, 114 };
  /* beams for forward and backward scanning radars; -1 will be replaced by the selected camping beam */
  int forward_beams[ 38]=  {   0,  -1,   1,  -1,   2,  -1,   3,  -1,   4,  -1,   5,  -1,   6,  -1,   7,  -1,   8,  -1,   9,  -1,
                              10,  -1,  11,  -1,  12,  -1,  13,  -1,  14,  -1,  15,  -1,  -1,  -1,  -1,  -1,  -1,  -1 };
  int backward_beams[ 38]= {  15,  -1,  14,  -1,  13,  -1,  12,  -1,  11,  -1,  10,  -1,   9,  -1,   8,  -1,   7,  -1,   6,  -1,
                               5,  -1,   4,  -1,   3,  -1,   2,  -1,   1,  -1,   0,  -1,  -1,  -1,  -1,  -1,  -1,  -1 };

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

  cp=3300;
  SiteStart();


  /* Because of overrun the Integration time was reduced to 2.6s to make sure that each beam */
  /* starts at the desired 3s interval */
  intsc= 2;
  intus= 600000;
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
  /* Camping Beam */
  OptionAdd( &opt, "camp", 'i', &camping_beam);
  OptionAdd( &opt, "nrang", 'i', &nrang);

  arg= OptionProcess( 1, argc, argv, &opt, NULL);

  /* make sure this is in the allowed range, otherwise set to default */
  if ( (camping_beam < 0) && (camping_beam > 15) ) camping_beam= 7;
  /* replace the -1 with the camping beam value */
  for (n=1; n<num_scans; n++) {
    if ( forward_beams[ n] == -1)  forward_beams[ n]= camping_beam;
    if (backward_beams[ n] == -1) backward_beams[ n]= camping_beam;
  }
 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  sprintf(progname,"themisscan");

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
      iv= skipsc*1000000 + skipus;
      bv= scnsc* 1000000 + scnus;
      tv=(mt* 60 + sc)* 1000000 + us + iv/2 - 100000;
      skip=(tv % bv)/iv;
      if (skip> num_scans-1) skip=0;
      if (skip<0) skip=0;
    }
   sprintf(logtxt, "Beam skip: %d", skip);
   ErrLog(errlog, progname, logtxt);

    if (backward) {
      bmnum= backward_beams[ skip];
    } else {
      bmnum=  forward_beams[ skip];
    }

    do {
// TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);
// sprintf( logtxt, "Start of do loop: %d:%d:%d:%d", hr, mt, sc, us);
// ErrLog( errlog, progname, logtxt);

      /* Synchronize to the desired start time */
      /* This will only work, if the total time through the do loop is < 3s */
      /* If this is not the case, decrease the Integration time */
      {
        int t_now;
        int t_dly;
        TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);
        t_now= ( (mt* 60 + sc)* 1000 + us/ 1000 ) % (scnsc* 1000 + scnus/ 1000);
        t_dly= scan_times[ skip]* 1000 - t_now;
        if (t_dly > 0) delay( t_dly);
      }

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

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d) %d %d",bmnum,intsc,intus,hr,mt,sc,us, skip, scan_times[ skip]);
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
		ErrLog(errlog,progname, logtxt);		
	} else {
		sprintf(logtxt, "Scan total time: %f seconds", totalscantime);
		ErrLog(errlog,progname, logtxt);
		ErrLog(errlog,progname,"Waiting for scan boundary."); 
  		if(exitpoll==0) OpsWaitBoundary(scnsc,scnus);
	}
  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
}

