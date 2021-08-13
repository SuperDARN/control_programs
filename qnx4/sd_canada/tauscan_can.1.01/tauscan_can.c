/* tauscan_can.c
   ==========
   Author: Dieter ANDRE
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
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
#include "fitacfex.h"

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
 $Log: tauscan_can.c,v $
 Revision 1.01  2014/01/30 19:30:00  ASReimer
 Removed unecessary code, added lag 15

 Revision 1.00  2013/07/23 19:30:00  DAndre
 Adapted from the APL Revision 1.14 for use
 by the Canadian radars

*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: tauscan_can.c,v 1.01 2014/01/30 ASReimer Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;
      
int main(int argc,char *argv[]) {
  int i;

  /* we use Ashton Reimer's pulse and lag table */
  int ptab[13]={0,14,15,22,25,27,31,45,49,51,54,61,62};


  int lags[LAG_SIZE][2] = {
	{ 0, 0}, /* 0 */
	{14,15}, /* 1 */
	{61,62}, /* 1 */
	{25,27}, /* 2 */
	{49,51}, /* 2 */
	{22,25}, /* 3 */
	{51,54}, /* 3 */
	{27,31}, /* 4 */
	{45,49}, /* 4 */
	{22,27}, /* 5 */
	{49,54}, /* 5 */
	{25,31}, /* 6 */
	{45,51}, /* 6 */
	{15,22}, /* 7 */
	{54,61}, /* 7 */
	{14,22}, /* 8 */
	{54,62}, /* 8 */
	{22,31}, /* 9 */
	{45,54}, /* 9 */
	{15,25}, /* 10 */
	{51,61}, /* 10 */
	{14,25}, /* 11 */
	{51,62}, /* 11 */
	{15,27}, /* 12 */
	{49,61}, /* 12 */
	{14,27}, /* 13 */
	{49,62}, /* 13 */
	{ 0,14}, /* 14 */
	{31,45}, /* 14 */
    { 0,15}, /* 15 */
	{15,31}, /* 16 */
	{45,61}, /* 16 */
	{14,31}, /* 17 */
	{45,62}, /* 17 */
	{ 62,62} /* alternate lag-0 */
  };

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

  int nowait=0;

  unsigned char fast=0;
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
   
  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l nowait l",
                  &sbm,&ebm,
                  &dfrq,&nfrq,
                  &dfrang,&nfrang,
                  &dmpinc,&nmpinc,
                  &frqrng,&xcnt,&nowait);

  SiteStart();

  cp=3600;
  intsc=7;
  intus=0;
  mppul=13;
  mplgs=18;     /* Total # of lags including 0 and missing 15 */
  nmpinc=1800;
  dmpinc=1800;
  nrang=75;
  rsep=45;
  txpl=300;
/*
  dfrq=14500;
  nfrq=10200;
  sbm=23;
  ebm=4;
  frqrng=240;
*/

  lagnum=0;
  for (i=0;i<256;i++) {
    if ((lags[i][0]==ptab[mppul-1]) && (lags[i][1]==ptab[mppul-1])) break;   
    lagnum++;
  }
  fprintf(stderr,"%d\n",lagnum);


  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */


  OptionAdd(&opt,"di",'x',&discretion);


  OptionAdd(&opt,"el",'t',&ename);
  OptionAdd(&opt,"sc",'t',&sname);

  OptionAdd(&opt,"frqrng",'i',&frqrng);
 
  OptionAdd(&opt,"rsep",'i',&rsep);

  OptionAdd( &opt, "dt", 'i', &day);
  OptionAdd( &opt, "nt", 'i', &night);
  OptionAdd( &opt, "df", 'i', &dfrq);
  OptionAdd( &opt, "nf", 'i', &nfrq);
  OptionAdd( &opt, "dr", 'i', &dfrang);
  OptionAdd( &opt, "nr", 'i', &nfrang);
  OptionAdd( &opt, "xcf", 'i', &xcnt);
  OptionAdd(&opt,"fast",'x',&fast);
  OptionAdd(&opt,"fixfrq", 'i', &fixfrq);

  arg=OptionProcess(1,argc,argv,&opt,NULL);  
 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);  

  SiteSetupHardware();

  if (fast) {
     cp=3601;
     scnsc=60;
     scnus=0;
     intsc=3;
     intus=500000;
  }

  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  if (fast) sprintf(progname,"tauscan_can (fast)");
  else sprintf(progname,"tauscan_can");

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
        stfrq=dfrq;
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        stfrq=nfrq;
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

      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog( errlog, progname, logtxt);

      if (SiteFCLR(stfrq,stfrq+frqrng)==FREQ_LOCAL)
        ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

      if ( (fixfrq > 8000) && (fixfrq < 25000 ) ) tfreq= fixfrq;

      SiteSetFreq(tfreq); 

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);

      ErrLog(errlog,progname,logtxt);

      tsgid=SiteTimeSeq(ptab);

      nave=SiteIntegrateex(lags);
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
      /*
      sprintf(logtxt, "Thresh pwr0: %f %f", raw.thr, raw.pwr0[10]);
      ErrLog(errlog,progname,logtxt);
      sprintf(logtxt, "acfd xcfd: %f %f %f %f", raw.acfd[10][0][0], raw.acfd[10][0][1],
        raw.xcfd[10][0][0], raw.xcfd[10][0][1]);
      ErrLog(errlog,progname,logtxt);
      */

      FitACFex(&prm,&raw,&fblk,&fit);

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
    if ( ! nowait) {   
      if (exitpoll==0) OpsWaitBoundary(scnsc,scnus);
    }
  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;   
} 
 
