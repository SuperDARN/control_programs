/* beaconscan.c
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
 $Log: beaconscan.c,v $
 Revision 1.00 2014/11/07 20:00 KKrieger
 Adapted from politescan 1.00 
*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: beaconscan.c,v 1.00 2014/11/07 20:00:00 KKrieger Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;

int main(int argc,char *argv[]) {
/*  int ptab[8] = {0, 14, 22, 24, 27, 31, 42, 43};
  int lags[LAG_SIZE][2] = 
{{0,0},
{42,43},
{22,24},
{24,27},
{27,31},
{22,27},
{24,31},
{14,22},
{22,31},
{14,24},
{31,42},
{31,43},
{14,27},
{0,14},
{27,42},
{27,43},
{14,31},
{24,42},
{24,43},
{22,42},
{22,43},
{0,22},
{0,24},
{43,43}};
*/
  int *ptab;
int **lags;
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
  int skip=0;
  int cnt=0;

  unsigned char discretion=0;
  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 

  strncpy(combf,progid,80);

/* Get command line options */
  OpsSetupCommand(argc,argv);
/* Get radar information from environment (station ID, radar name, etc) */
  OpsSetupRadar();
/* Set up radar shell variables (mppul, intsc, bmnum, etc...) */
  OpsSetupShell();
  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
                  &sbm,&ebm,
                  &dfrq,&nfrq,
                  &dfrang,&nfrang,
                  &dmpinc,&nmpinc,
                  &frqrng,&xcnt);

/* Initialize DDS and site specific variables such as start and end beams */
  SiteStart();

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "el", 't', &ename);
  OptionAdd(&opt, "sc", 't', &sname);
  OptionAdd(&opt, "di", 'x', &discretion);
  OptionAdd(&opt, "dt", 'i', &day);
  OptionAdd(&opt, "nt", 'i', &night);
  OptionAdd(&opt, "df", 'i', &dfrq);
  OptionAdd(&opt, "nf", 'i', &nfrq);
  OptionAdd(&opt, "dr", 'i', &dfrang);
  OptionAdd(&opt, "nr", 'i', &nfrang);
  OptionAdd(&opt, "dm", 'i', &dmpinc);
  OptionAdd(&opt, "nm", 'i', &nmpinc);
  OptionAdd(&opt, "sb", 'i', &sbm);
  OptionAdd(&opt, "eb", 'i', &ebm);
  OptionAdd(&opt, "xcf", 'i', &xcnt);
  OptionAdd(&opt, "nrang", 'i', &nrang);
  OptionAdd(&opt, "rsep", 'i', &rsep);

  arg=OptionProcess(1,argc,argv,&opt,NULL);  

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);  

/* Setup site specific driver for receiver card,
 set buffer locations and amount of buffers */
  SiteSetupHardware();

  cp= 3390;
  scnsc= 120;
  scnus=0;
  intsc= 7;
  intus= 250000;
  mppul = 0;
  mplgs = 0;
 /* dmpinc = 0;
  nmpinc = 0;
  nrang = 0;*/
 /* rsep = 45;
  txpl = 300;
  dfrang = 180;
  nfrang = 180;*/
  

  if ( discretion) cp= -cp;
  sprintf(progname,"beaconscan");

/* Setup our list of tasks to talk with (echoraw, iqwrite... etc) */
 /* OpsFitACFStart();*/
  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
  }

  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  do {
/* Are we at 2 hour boundary? */
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

    do {
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      sprintf(logtxt,"Listening on beam:%d listentime:%ds.%dus (%d:%d:%d:%d)",bmnum,
                      intsc,intus,hr,mt,sc,us);
      ErrLog(errlog,progname,logtxt);
      ErrLog(errlog,progname,"Setting beam.");

/* SiteSetIntt sets the time when ddsintegrate should stop integrating,
which will just be intsc seconds and itus microseconds in the future */
      SiteSetIntt(intsc,intus);
/* This sets the azimuth to be used when calculating phases during frequency
programming */
      SiteSetBeam(bmnum);

      /* check frequency here */
	  if (FreqTest(ftable, tfreq) != 0) {
		/* If our frequency is restricted, set to default frequency 
		and print error */
		sprintf(logtxt, "ERROR: The frequency: %d kHz is restricted!", tfreq);
		ErrLog(errlog, progname, logtxt);
		tfreq = ftable->dfrq;
	    sprintf(logtxt, "Setting frequency to: %d kHz", tfreq);
        ErrLog(errlog,progname,logtxt);
	  }

      SiteSetFreqQuiet(tfreq);

      sprintf(logtxt,"Listening on: %d",tfreq);
      ErrLog(errlog,progname,logtxt);
    
/* Here we should use the receiver to listen for a certain amount of
time on the beam, then write the information to IQ file */
/* TODO */
/*SiteIntegrate(lags);*/
     sleep(intsc);
      sprintf(logtxt,"Listening on beam %d finished", bmnum);
      ErrLog(errlog,progname,logtxt);
      OpsBuildPrm(&prm,ptab,lags);
      ErrLog(errlog,progname,"Building IQ data structures");
      OpsBuildIQ(&iq);

      ErrLog(errlog,progname,"Sending messages."); 
      msg.num=0;
      msg.tsize=0;
      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);
      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);
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
      OpsWaitBoundary(scnsc,scnus);
    }

  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;
}
