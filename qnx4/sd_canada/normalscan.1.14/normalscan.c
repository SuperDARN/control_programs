/* normalscan.c
   ============
   Author: R.J.Barnes
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
 $Log: normalscan.c,v $
 Revision 1.14  2018/11/22 20:00:00 KKrieger
 Added intsc and intus (integration time) to
 command line options

 Revision 1.13  2014/12/01 20:00:00  KKrieger
 Added total scan time checking to prevent
 delays at the end of a scan

 Revision 1.12  2012/11/27 20:00:00  DAndre
 Added options dfrang and nfrang
 Removed option frang

 Revision 1.11  2008/11/20 15:18:00  DAndre
 Added option fixfrq and restart on integration error.

 Revision 1.10  2008/03/15 00:44:50  code
 Added iqwrite.

 Revision 1.9  2008/03/14 17:31:39  code
 Changes to accomodate IQ sample capture.

 Revision 1.8  2006/07/12 15:48:23  code
 Added call to set up command line and limited data written to rawacf and fitacf.

 Revision 1.7  2006/02/07 20:55:06  barnes
 Simon Shepherd's modification to the lag table.

 Revision 1.6  2006/02/07 17:50:17  barnes
 Fixed bug in calling Errlog.

 Revision 1.5  2006/02/07 17:43:41  barnes
 Dieter Andre's modification to make integration errors less mysterious.

 Revision 1.4  2005/08/01 18:18:47  barnes
 Fixed incorrect command line option.

 Revision 1.3  2005/07/19 15:26:59  barnes
 Added Dieter Andre's extra command line options.

 Revision 1.2  2004/07/23 15:48:27  barnes
 Fixed bug in beam swinging direction.

 Revision 1.1  2004/05/11 17:43:22  barnes
 Initial revision
 
*/

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

char cmdlne[1024];
char progid[80]={"$Id: normalscan.c,v 1.14 2018/11/22 20:00:00 KKrieger Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

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
   
  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",                        
                  &sbm,&ebm,                              
                  &dfrq,&nfrq,                  
                  &dfrang,&nfrang,                            
                  &dmpinc,&nmpinc,                            
                  &frqrng,&xcnt);        

  cp=150;
  intsc=0;
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
  OptionAdd( &opt, "nrang", 'i', &nrang);
  OptionAdd( &opt, "xcf", 'i', &xcnt);
 
  OptionAdd(&opt,"fast",'x',&fast);
  OptionAdd( &opt, "fixfrq", 'i', &fixfrq);
  OptionAdd( &opt, "intsc", 'i', &intsc);
  OptionAdd( &opt, "intus", 'i', &intus);


  
  arg=OptionProcess(1,argc,argv,&opt,NULL);  
 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);  

  SiteSetupHardware();

  if (fast) {
     cp=151;
     scnsc=60;
     scnus=0;
     // Only set to 3 seconds if it wasn't set on the command line
     if (intsc == 0 && intus == 0) {
       intsc = 3;
       intus = 0;
     }
  } else {
     // Only set to 7 seconds if it wasn't set on the command line
     if (intsc == 0 && intus == 0) {
	intsc = 7;
	intus = 0;	
     }
  }
  if ( discretion) cp= -cp;

  txpl=(rsep*20)/3;

  if (fast) sprintf(progname,"normalscan (fast)");
  else sprintf(progname,"normalscan");

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

    skip=OpsFindSkip(scnsc,scnus);
    sprintf(logtxt, "Beam skip: %d", skip);
	ErrLog(errlog,progname,logtxt);

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
        /* continue; */
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

      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prm,
		PRM_TYPE,0); 

      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq,
		 IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,
		 IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &raw,
		RAW_TYPE,0);    

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fit,
		FIT_TYPE,0);   
      RMsgSndAdd(&msg,strlen(progname)+1,progname,
		NME_TYPE,0);   
 

      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 
  
      ErrLog(errlog,progname,"Polling for exit."); 
    
      exitpoll=RadarShell(sid,&rstable);
      if (exitpoll !=0) break;
      scan=0;
      if (bmnum==ebm) break;
      if (backward) bmnum--;
      else bmnum++;

    } while (1);
    scanstoptime = time(NULL);
	totalscantime = difftime(scanstoptime,scanstarttime);
	if(totalscantime > scnsc) {
		sprintf(logtxt, "Scan time over limit! %f seconds. Continuing", totalscantime);
		ErrLog(errlog,progname,logtxt);	
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
 
