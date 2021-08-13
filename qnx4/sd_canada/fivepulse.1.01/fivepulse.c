/* fivepulse.c
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
 $Log: fivepulse.c,v $
 Revision 1.1  2013/04/29 20:00:00 DAndre
 Corrected error in the lag table [lag 2; { 5,8} -> { 6,8}

 Revision 1.0  2013/01/03 20:00:00 DAndre
 fivepulse uses a five pulse sequence designed
 by George Sofko and Ashton Reimer.
 The same beam is scanned at two frequencies,
 then the next beam is scanned.

*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: fivepulse.c,v 1.0 2013/01/03 18:00:00 DAndre Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]= { TASK_NAMES, 0};

int arg=0;
struct OptionData opt;

int main(int argc,char *argv[]) {

  int ptab[5] = { 0, 6, 8, 9, 13};
  int lags[LAG_SIZE][2] = {
    { 0, 0},		/*  0 */
    { 8, 9},		/*  1 */
    { 6, 8},		/*  2 */
    { 6, 9},		/*  3 */
    { 9,13},		/*  4 */
    { 8,13},		/*  5 */
    { 0, 6},		/*  6 */
    { 6,13},		/*  7 */
    { 0, 8},		/*  8 */
    { 0, 9},		/*  9 */

    {13,13}};		/* alternate lag-0  */


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
  int fixfrq=0;
  int st_id;

  /* Frequencies used for the radars */
  int sas_freq[ 2]= { 10500, 13000};
  int pgr_freq[ 2]= { 10500, 13000};
  int rkn_freq[ 2]= { 10200, 12200};
  int inv_freq[ 2]= { 10300, 12200};
  int cly_freq[ 2]= { 10500, 12500};
  int num_scans= 32;
  /* beams for forward and backward scanning radars */
  int forward_beams[ 32]=  {   0,   0,   1,   1,   2,   2,   3,   3,   4,   4,   5,   5,   6,   6,   7,   7,   8,   8,
                               9,   9,  10,  10,  11,  11,  12,  12,  13,  13,  14,  14,  15, 15 };
  int backward_beams[ 32]= {  15,  15,  14,  14,  13,  13,  12,  12,  11,  11,  10,  10,   9,   9,   8,   8,   7,   7,
                               6,   6,   5,   5,   4,   4,   3,   3,   2,   2,   1,   1,   0,   0 };
  int transmit_freqs[ 32]; 

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

  st_id= atoi( getenv("SD_RADARID"));
  cp= 3400;
  SiteStart();

  intsc= 3;
  intus= 600000;
  mppul= 5;
  mplgs= 10;
  mpinc= 2000;
  nmpinc= 2000;
  dmpinc= 2000;
  nrang=75;
  rsep= 30;
  txpl= 200;
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
  OptionAdd( &opt, "nrang", 'i', &nrang);

  arg= OptionProcess( 1, argc, argv, &opt, NULL);

  /* set the frequencies */
  for (n= 0; n < num_scans;n++) {
    switch ( st_id) {
      case 5: transmit_freqs[ n]= sas_freq[ n % 2];
               break;
      case 6: transmit_freqs[ n]= pgr_freq[ n % 2];
               break;
      case 64: transmit_freqs[ n]= inv_freq[ n % 2];
               break;
      case 65: transmit_freqs[ n]= rkn_freq[ n % 2];
               break;
      case 66: transmit_freqs[ n]= cly_freq[ n % 2];
               break;
    }
  }

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);

  SiteSetupHardware();

  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  sprintf(progname,"fivepulse");

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
// sprintf( logtxt, "Skip: %d for %d:%d:%d:%d", skip, hr, mt, sc, us);
// ErrLog( errlog, progname, logtxt);

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
      stfrq= transmit_freqs[ skip];

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
      if (backward) {
        bmnum= backward_beams[ skip];
      } else {
        bmnum=  forward_beams[ skip];
      }
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

