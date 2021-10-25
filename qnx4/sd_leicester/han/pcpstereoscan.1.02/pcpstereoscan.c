/* stereoscan.c
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
#include <sys/proxy.h>
#include <sys/kernel.h>
#include <string.h>
#include <time.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"

#include "taskid.h"
#include "errlog.h"
#include "freq.h"
#include "rmsg.h"
#include "radarshell.h"
#include "rmsgsnd.h"
#include "tsg.h"
#include "tsgtable.h"
#include "maketsg.h"
#include "global.h"
#include "globals.h"
#include "setup.h"
#include "reopen.h"
#include "tmseq.h"
#include "raws.h"
#include "sync.h"
#include "interface.h"
#include "hdw.h"

/*
 $Log: pcpstereoscan.c,v $
 Revision 1.2  2006/07/12 15:51:43  code
 Added call to set up command line and limited data written to rawacf and fitacf.

 Revision 1.1  2006/03/30 11:19:18  barnes
 Initial revision
 
*/

/*
	Modified 10/12/2001 PH

	Added bnA and bnB frequency band options to cycle through frequency
	bands during each scan. No check is made for invalid band numbers.
	A band number of -1 is ignored. Bands can be added/removed at any
	time using radar_shell, changes won't take effect until the end of
	the current scan. If no band numbers are specified, normal day/night
	frequencies are used.

	Frequency bands can also be specified for campA and campB modes.

	Extra options can be placed in a separate schedule file. The file
	name should be the last argument in the schedule argument list and
	should include the full path.

	Modified 16/01/02 PH

	Fixed synchronisation bug. The code wasn't subtracting the time taken
	for a clear frequency search from the beam integration time, resulting
	in an extended beam dwell time and a sweep lasting for more than 60 s
	for fast stereo and 120 s for slow stereo 7 s integration.

	Standard slow stereo (normal stereo) is now 7 seconds (by Mark Lester).

	Modified 30/01/02 PH

	added -cts? options that pre-set other options

			dwell	chan A	cpidA	chan B	cpidB	Ice/Fin	offset	range
			------	------	------	------	------	------- ------	-----
	cts2 -    7		normal	 152	 tbj	-26002	 5 / 9	 +400	 45
	cts4 -	  7		normal	 152	normal	-26004	 5 / 9	 +400	 45
	cts6 -	  7		normal	 152	1 camp	-26006	 5 / 9	 +400	 45
	cts8 -	  7		normal	 152	2 camp	-26008	 5 / 9	 +400	 45

	cts3 -	  3		normal	 153	 tbj	-26003	 5 / 9	 +400	 45
	cts5 -	  3		normal	 153	normal	-26005	 5 / 9	 +400	 45
	cts7 -	  3		normal	 153	1 camp	-26007	 5 / 9	 +400	 45
	cts9 -	  3		normal	 153	2 camp	-26009	 5 / 9	 +400	 45

	Single camp beams are hard coded to the standard Finland beam 9 and
	Iceland beam 5.

	Double camp beams are the same for both radars and defined as
	DOUBLE_CAMP_LOW and DOUBLE_CAMP_HIGH.

	range gates and first range are let to the default settings of 45km
	and 180km respectively.

	specifying any cts option will override low and high beam, offset, cpIDs,
	integration time and scan boundary. tbj mode overrides frequencies.

	Modified 05/04/02 PH

	Uses the new stereo aware version of fitacf (1.06).

	Modified 30/04/2002 PH

	Added cpidA and cpidB to set the cpid. This option overrides any
	previously defined cpid.

	Added bmnA and bmnB to cycle through beams numbers on both channels.
	To specify double camp beams use -bm0B 5 -bm1B 9
	To specify single camp beams use -bm0B 5
	The same beam number can be specified more than once.
	The number of beams in the channel A beam list determines the number of
	beams per scan. This is important if you are also using frequency lists.

	cts 2, 3, 4, 5 beams patterns can be changed with bm*A and bm*B options.

	Modified 25/09/02 PH

	Changed so that the B channel frequency band changes at the end of the
	channel B scan, so that it is independent of the number of beams on
	channel A.

  Modified 7th Dec 2001 to add camp beam flags
  Modified 23 Nov 2001 to add new -ns and -fs flags and changed some defaults
  Modified 10th Aug to account for backwards scanning radars
  Modified 8th Aug fitacf_s
  Julian modified version. March 2001

*/

/* set CPIDs for this experiment, 9212 is CPID provided by UAF for polar cap patch experiment
   -26999 is dummy CPID for the non-existent channelB */
#define CPID_A 9212
#define CPID_B -26999


#define INTT      3
#define RSEP_A 15
#define RSEP_B 15
#define FRANG_A 180
#define FRANG_B 180


#define LOW_BEAM_A   0
#define HIGH_BEAM_A  15
#define LOW_BEAM_B   0
#define HIGH_BEAM_B  15

#define SINGLE_CAMP_BEAM_PYK 5
#define SINGLE_CAMP_BEAM_HAN 9


#define DOUBLE_CAMP_LOW 5
#define DOUBLE_CAMP_HIGH 9

#define UCONT_NAME "ucont_moni"

#define CHN_A 0
#define CHN_B 1

#define NUMBANDS 10
#define NUMBEAMS 16


#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","rawacfwrite","fitacfwrite"

#define PCPBEAM 9
#define PCPFNUM 8
int pcpfreqs[PCPFNUM]; 
int pcpcnt;

void u_read_uconts(void);
void u_init_freq_bands(void);

char cmdlne[1024];
char progid[80]={"$Id: pcpstereoscan.c,v 1.2 2011/12/13 15:51:43 code Exp $"};
char progname[256];
struct TaskID *errlog;

int usfreq[42];
int ufreq_range[42];

pid_t uucont_proxy;

int low_beam_A=LOW_BEAM_A;
int high_beam_A=HIGH_BEAM_A;
int low_beam_B=LOW_BEAM_B;
int high_beam_B=HIGH_BEAM_B;




char *tasklist[]=
 { TASK_NAMES,
  0};

int arg=0;
struct OptionData opt;
      
int main(int argc,char *argv[]) {

  int ptab[7] = {0,9,12,20,22,26,27};

  int lags[LAG_SIZE][2] = {
    { 0, 0},		/*  0 */
    {26,27},		/*  1 */
    {20,22},		/*  2 */
    {9,12},		/*  3 */
    {22,26},		/*  4 */
    {22,27},		/*  5 */

    {20,26},		/*  7 */
    {20,27},		/*  8 */
    {0,9},		/*  9 */
    {12,22},		/* 10 */
    {9,20},		/* 11 */
    {0,12},	  	    /* 12 */
    {9,22},		/* 13 */
    {12,26},		/* 14 */
    {12,27},		/* 15 */
    {9,26},		/* 16 */
    {9,27},		/* 17 */
    {27,27}};		/* alternate lag-0  */


  char *sname=NULL;
  char *ename=NULL;
  char *sdname={SCHEDULER};
  char *edname={ERRLOG};
  char logtxt[1024];

  char a[10];

  int n,i;
  pid_t sid;
  int exitpoll=0;
 
  int scnsc=0;
  int scnus=0;
 
  unsigned char ns=0;
  unsigned char fs=0;

  int stereo_offset=400; /* channel A delayed by this 
                            number of microseconds */


  int ifreqsA[NUMBANDS],ifreqsB[NUMBANDS];

  int day_nightA=1;
  int day_nightB=1;

  int numfreqbandsA;
  int numfreqbandsB;

  int ifreqsA_index=0; 
  int ifreqsB_index=0;
  
  int ifreqsAband;
  int ifreqsBband;

  int sbm, ebm, bmnm[16], bmcnt;

  int numbeamsA,numbeamsB;

  unsigned char cts2=0,cts4=0,cts6=0,cts8=0;
  unsigned char cts3=0,cts5=0,cts7=0,cts9=0;
  
  int cpidA=0,cpidB=0;

  xcfA=xcfB=1;
 
  cpA=CPID_A;
  cpB=CPID_B;

  SiteStart();
 
  intsc=INTT;
  intus=0;
 
  maxattenA=maxattenB=7;

  if ((uucont_proxy=SiteInitProxy(UCONT_NAME))==-1) {
    perror("cannot attach proxy");
  }
 
  u_init_freq_bands();
  
  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 
printf("** 1\n");
  strncpy(combf,progid,80);   
  OpsSetupCommand(argc,argv);
  OpsSetupRadar();
  OpsSetupShell();

  RadarShellAdd(&rstable,"intt",var_LONG,&intsc);
  RadarShellAdd(&rstable,"inttsc",var_LONG,&intsc);
  RadarShellAdd(&rstable,"inttus",var_LONG,&intus);

  RadarShellAdd(&rstable,"txplA",var_LONG,&txplA);
  RadarShellAdd(&rstable,"mpincA",var_LONG,&mpincA);
  RadarShellAdd(&rstable,"mppulA",var_LONG,&mppulA);
  RadarShellAdd(&rstable,"mplgsA",var_LONG,&mplgsA);
  RadarShellAdd(&rstable,"nrangA",var_LONG,&nrangA);
  RadarShellAdd(&rstable,"frangA",var_LONG,&frangA);
  RadarShellAdd(&rstable,"rsepA",var_LONG,&rsepA);
  RadarShellAdd(&rstable,"bmnumA",var_LONG,&bmnumA);
  RadarShellAdd(&rstable,"xcfA",var_LONG,&xcfA);
  RadarShellAdd(&rstable,"tfreqA",var_LONG,&tfreqA);
  RadarShellAdd(&rstable,"scanA",var_LONG,&scanA);
  RadarShellAdd(&rstable,"mxpwrA",var_LONG,&mxpwrA);
  RadarShellAdd(&rstable,"lvmaxA",var_LONG,&lvmaxA);
  RadarShellAdd(&rstable,"rxriseA",var_LONG,&rxriseA);
  RadarShellAdd(&rstable,"combfA",var_STRING,combfA);
  RadarShellAdd(&rstable,"cpA",var_LONG,&cpA);

  RadarShellAdd(&rstable,"txplB",var_LONG,&txplB);
  RadarShellAdd(&rstable,"mpincB",var_LONG,&mpincB);
  RadarShellAdd(&rstable,"mppulB",var_LONG,&mppulB);
  RadarShellAdd(&rstable,"mplgsB",var_LONG,&mplgsB);
  RadarShellAdd(&rstable,"nrangB",var_LONG,&nrangB);
  RadarShellAdd(&rstable,"frangB",var_LONG,&frangB);
  RadarShellAdd(&rstable,"rsepB",var_LONG,&rsepB);
  RadarShellAdd(&rstable,"bmnumB",var_LONG,&bmnumB);
  RadarShellAdd(&rstable,"xcfB",var_LONG,&xcfB);
  RadarShellAdd(&rstable,"tfreqB",var_LONG,&tfreqB);
  RadarShellAdd(&rstable,"scanB",var_LONG,&scanB);
  RadarShellAdd(&rstable,"mxpwrB",var_LONG,&mxpwrB);
  RadarShellAdd(&rstable,"lvmaxB",var_LONG,&lvmaxB);
  RadarShellAdd(&rstable,"rxriseB",var_LONG,&rxriseB);
  RadarShellAdd(&rstable,"combfB",var_STRING,combfB);
  RadarShellAdd(&rstable,"cpB",var_LONG,&cpB);

  OptionAdd(&opt, "ns",	'x', &ns);
  OptionAdd(&opt, "fs",	'x', &fs);
  OptionAdd(&opt, "e",	't', &ename);  
  OptionAdd(&opt, "sc",	't', &sname);
  OptionAdd(&opt, "intt",	'i', &intsc);
  OptionAdd(&opt, "dfA",	'i', &dfrqA);
  OptionAdd(&opt, "dfB",	'i', &dfrqB);
  OptionAdd(&opt, "nfA",	'i', &nfrqA);
  OptionAdd(&opt, "nfB",	'i', &nfrqB);
  
  OptionAdd(&opt, "offset",	'i',&stereo_offset);

  OptionAdd(&opt, "frA",	'i', &frangA);
  OptionAdd(&opt, "frB",	'i', &frangB);
  OptionAdd(&opt, "rgA",	'i', &rsepA);
  OptionAdd(&opt, "rgB",	'i', &rsepB);
  OptionAdd(&opt, "lbA",	'i', &low_beam_A);
  OptionAdd(&opt, "hbA",	'i', &high_beam_A);
  OptionAdd(&opt, "lbB",	'i', &low_beam_B);
  OptionAdd(&opt, "hbB",	'i', &high_beam_B);
  OptionAdd(&opt, "sp",	'i', &scnsc);

  OptionAdd(&opt, "cpidA", 'i', &cpidA);
  OptionAdd(&opt, "cpidB", 'i', &cpidB);

  /* set up remaining shell variables */

  RadarShellAdd(&rstable, "dfA",var_LONG,&dfrqA);
  RadarShellAdd(&rstable, "dfB",var_LONG,&dfrqB);
  RadarShellAdd(&rstable, "nfA",var_LONG,&nfrqA);
  RadarShellAdd(&rstable, "nfB",var_LONG,&nfrqB);

  RadarShellAdd(&rstable, "offset",	var_LONG,&stereo_offset);
 
  RadarShellAdd(&rstable, "low_beamA",	var_LONG,&low_beam_A);
  RadarShellAdd(&rstable, "high_beamA",var_LONG,&high_beam_A);
  RadarShellAdd(&rstable, "low_beamB",	var_LONG,&low_beam_B);
  RadarShellAdd(&rstable, "high_beamB",var_LONG,&high_beam_B);
  RadarShellAdd(&rstable, "scan_period",var_LONG,&scnsc);
printf("** 2\n");

  /* add new options - individual frequency bands */


  /*
	the beam lists now contain:
	1) the number of beams in the beam lists
	2) if no beam list or camp beams were specified = 0 -> 15
	3) if high and low beams were specified			= low_beam -> high_beam
  */
   
  sprintf(progname,"pcpstereoscan");
printf("** 3\n");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
  }
  /* set array of beams to match forward or backward scanning radar */
  if(backward){
	for(bmcnt=15;bmcnt>=0;bmcnt--){
		bmnm[bmcnt]=15-bmcnt;
	}
  }
  else{
	for(bmcnt=0;bmcnt<=15;bmcnt++){
		bmnm[bmcnt]=bmcnt;
	}
  }
printf("** 4\n");

  /*  Set up the array of frequency indices to use for sounding */
  if(stid==0x09){
	pcpfreqs[0]=20;
	pcpfreqs[1]=22;
	pcpfreqs[2]=23;
	pcpfreqs[3]=26;
	pcpfreqs[4]=30;
	pcpfreqs[5]=32;
	pcpfreqs[6]=33;
	pcpfreqs[7]=35;
   }else if (stid==0x10){
	pcpfreqs[0]=0;
	pcpfreqs[1]=1;
	pcpfreqs[2]=2;
	pcpfreqs[3]=4;
	pcpfreqs[4]=6;
	pcpfreqs[5]=7;
	pcpfreqs[6]=8;
	pcpfreqs[7]=10;
   }
printf("** 5\n");

   /* set the number of range gates to 225 */
   nrangA=225;
   nrangB=225;
   /* set range seperation to 15km */
   rsepA=15;
   rsepB=15;

  /* scan loop */
  do {
	/* 3 second integrations during the scan */
  	intsc=3;
  	intus=0;
printf("** 6\n");
    	if (SiteStartScan()==0) continue;
    	if (OpsReOpen(2,0,0) !=0) {
      		ErrLog(errlog,progname,"Opening new files.");
printf("** 7\n");
      		for (n=0;n<tnum;n++) {
        		RMsgSndClose(tlist[n]);
        		RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
printf("** 8\n");
      		}
    	}
	/* set scan parameter to match forward or backward scanning radar */
    	scanA=scanB=1; 
printf("** 9\n");
    	ErrLog(errlog,progname,"Starting scan.");

	/* Scan over all 16 beams */
    	for (bmcnt=0;bmcnt<=15;bmcnt++) {
      		bmnumA=bmnm[bmcnt];
      		bmnumB=bmnm[bmcnt];

      		TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
printf("** 10\n");
       		if (OpsDayNight()==1) {
       			stfrqA=usfreq[dfrqA];
       			frqrngA=ufreq_range[dfrqA];
       		} else {
       			stfrqA=usfreq[nfrqA];
       			frqrngA=ufreq_range[nfrqA];
       		} 

      		if (OpsDayNight()==1) {
       			stfrqB=usfreq[dfrqB];
       			frqrngB=ufreq_range[dfrqB];
       		} else {
       			stfrqB=usfreq[nfrqB]; 	//jth change 
//       			tfrqB=usfreq[nfrqB]; 	//jth change 
       			frqrngB=ufreq_range[nfrqB];
       		} 
printf("** 11\n");
      		SiteSetChannel(CHN_B);
      		SiteSetBeam(bmnumB);
      		SiteSetChannel(CHN_A);
      		SiteSetBeam(bmnumA);
printf("** 11a\n"); 
      		SiteSetIntt(intsc,intus);
printf("** 11b\n");
printf("**stfrqA=%d, frqrngA=%d, stfrqB=%d, frqrngB=%d\n",stfrqA, frqrngA, stfrqB, frqrngB); 
      		if (SiteFCLRS(stfrqA,stfrqA+frqrngA,stfrqB,
                    stfrqB+frqrngB)==FREQ_LOCAL)
        		ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");
printf("** 12\n");
      		if (tfreqA==-1) tfreqA=ftable->dfrq;
      		if (tfreqB==-1) tfreqB=ftable->dfrq;
printf("** 13\n");
      		sprintf(logtxt,"Channel A Transmitting on: %d (Noise=%g)",tfreqA,noiseA);
      		ErrLog(errlog,progname,logtxt);

      		sprintf(logtxt,"Channel B Transmitting on: %d (Noise=%g)",tfreqB,noiseB);
      		ErrLog(errlog,progname,logtxt);

      		SiteSetChannel(CHN_A);
      		SiteSetFreq(tfreqA);
      		SiteSetChannel(CHN_B);
      		SiteSetFreq(tfreqB);

	  	sprintf(logtxt,"Integrating:%2d,%2d intt:%d (%02d:%02d:%02d) %5d  %5d - %s  %d\n",
		      	bmnumA, bmnumB, intsc, hr, mt, 
              		sc, tfreqA, tfreqB,  
              		OpsDayNight() ? "day" : "night", abs(tfreqA - tfreqB));
      		ErrLog(errlog,progname,logtxt);
      		u_read_uconts();

      		txplA = (rsepA * 20) / 3;
      		txplB = (rsepB * 20) / 3;
      		tsgidA=SiteTimeSeqS(0,ptab);
      		tsgidB=SiteTimeSeqS(1,ptab);

      		SiteIntegrateS(lags,lags);   

      		if (naveA<0) {
        		sprintf(logtxt,"Integration A failed:%d",naveA);
        		ErrLog(errlog,progname,logtxt); 
        		continue;
      		}
      		if (naveB<0) {
        		sprintf(logtxt,"Integration B failed:%d",naveB);
        		ErrLog(errlog,progname,logtxt); 
        		continue;
      		}

      		sprintf(logtxt,"Number of sequences: %d %d",naveA,naveB);
      		ErrLog(errlog,progname,logtxt);

      		OpsBuildRawS(0,&prmA,&rawA,ptab,lags);
      		OpsBuildRawS(1,&prmB,&rawB,ptab,lags);

      		FitACF(&prmA,&rawA,&fblk,&fitA);
      		FitACF(&prmB,&rawB,&fblk,&fitB);

      		ErrLog(errlog,progname,"Sending messages."); 
      		msg.num=0;
      		msg.tsize=0;
      		RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmA,
			PRM_TYPE,0); 
      		RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawA,
			RAW_TYPE,0);    
      		RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitA,
			FIT_TYPE,0);
      		RMsgSndAdd(&msg,strlen(progname)+1,progname,
			NME_TYPE,0);   
      		RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmB,
			PRM_TYPE,1); 
      		RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawB,
			RAW_TYPE,1);    
      		RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitB,
			FIT_TYPE,1);
      		RMsgSndAdd(&msg,strlen(progname)+1,progname,
			NME_TYPE,1);   
      		for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 
  
      		ErrLog(errlog,progname,"Polling for exit."); 
      		exitpoll=RadarShell(sid,&rstable);

	  	/* Calculate delay offsets if stereo offset has been changed
         		If stereo_offset is +ve A is later than B
       		*/

	  	if (stereo_offset > 0) {
	    		delays[1] = 0;
			delays[0] = stereo_offset / 10; /* stereo_offset is in usec */
 	  	} else {
			delays[0] = 0;
			delays[1] = (stereo_offset / 10) * (-1); /* stereo_offset is in usec */
	  	}

	  	if (exitpoll !=0) break;

	  	scanA = 0;
	  	scanB = 0;

   	} 
    
	/* increment individual frequency band index. Doesn't matter whether
	   individual frequency bands are being used for this channel or not.
     	*/
	/************  Sounding at end of scan ******************************/
	
	scanA=-2;
	scanB=-2;
        /* set sounding integration time to 1 second */
  	intsc=1;
  	intus=0;
	/* step through array of 8 frequencies */
	for(pcpcnt=0;pcpcnt<PCPFNUM;pcpcnt++){
		/* use fixed beam for this sounding */
      		bmnumA=PCPBEAM;
      		bmnumB=PCPBEAM;
      		TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

       		stfrqA=usfreq[pcpfreqs[pcpcnt]];
       		frqrngA=ufreq_range[pcpfreqs[pcpcnt]];

      		SiteSetChannel(CHN_B);
      		SiteSetBeam(bmnumB);
      		SiteSetChannel(CHN_A);
      		SiteSetBeam(bmnumA);
 
      		SiteSetIntt(intsc,intus);
      		if (SiteFCLRS(stfrqA,stfrqA+frqrngA,stfrqB,
                    stfrqB+frqrngB)==FREQ_LOCAL)
        		ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

      		if (tfreqA==-1) tfreqA=ftable->dfrq;
      		if (tfreqB==-1) tfreqB=ftable->dfrq;

      		sprintf(logtxt,"Channel A Transmitting on: %d (Noise=%g)",tfreqA,noiseA);
      		ErrLog(errlog,progname,logtxt);

      		sprintf(logtxt,"Channel B Transmitting on: %d (Noise=%g)",tfreqB,noiseB);
      		ErrLog(errlog,progname,logtxt);

      		SiteSetChannel(CHN_A);
      		SiteSetFreq(tfreqA);
      		SiteSetChannel(CHN_B);
      		SiteSetFreq(tfreqB);
	  	sprintf(logtxt,"Integrating:%2d,%2d intt:%d (%02d:%02d:%02d) %5d  %5d - %s  %d\n",
		      	bmnumA, bmnumB, intsc, hr, mt, 
              		sc, tfreqA, tfreqB,  
              		OpsDayNight() ? "day" : "night", abs(tfreqA - tfreqB));
      		ErrLog(errlog,progname,logtxt);
      		u_read_uconts();

      		txplA = (rsepA * 20) / 3;
      		txplB = (rsepB * 20) / 3;
      		tsgidA=SiteTimeSeqS(0,ptab);
      		tsgidB=SiteTimeSeqS(1,ptab);

      		SiteIntegrateS(lags,lags);   

      		if (naveA<0) {
        		sprintf(logtxt,"Integration A failed:%d",naveA);
        		ErrLog(errlog,progname,logtxt); 
        		continue;
      		}
      		if (naveB<0) {
        		sprintf(logtxt,"Integration B failed:%d",naveB);
        		ErrLog(errlog,progname,logtxt); 
        		continue;
      		}
      		sprintf(logtxt,"Number of sequences: %d %d",naveA,naveB);
      		ErrLog(errlog,progname,logtxt);

      		OpsBuildRawS(0,&prmA,&rawA,ptab,lags);
      		OpsBuildRawS(1,&prmB,&rawB,ptab,lags);

      		FitACF(&prmA,&rawA,&fblk,&fitA);
      		FitACF(&prmB,&rawB,&fblk,&fitB);

      		ErrLog(errlog,progname,"Sending messages."); 
      		msg.num=0;
      		msg.tsize=0;
      		RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmA,
			PRM_TYPE,0); 
      		RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawA,
			RAW_TYPE,0);    
      		RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitA,
			FIT_TYPE,0);
      		RMsgSndAdd(&msg,strlen(progname)+1,progname,
			NME_TYPE,0);   
      		RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmB,
			PRM_TYPE,1); 
      		RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawB,
			RAW_TYPE,1);    
      		RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitB,
			FIT_TYPE,1);
      		RMsgSndAdd(&msg,strlen(progname)+1,progname,
			NME_TYPE,1);   
      		for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 
      		ErrLog(errlog,progname,"Polling for exit."); 
      		exitpoll=RadarShell(sid,&rstable);

	  	/* Calculate delay offsets if stereo offset has been changed
         		If stereo_offset is +ve A is later than B
       		*/

	  	if (stereo_offset > 0) {
	    		delays[1] = 0;
			delays[0] = stereo_offset / 10; /* stereo_offset is in usec */
 	  	} else {
			delays[0] = 0;
			delays[1] = (stereo_offset / 10) * (-1); /* stereo_offset is in usec */
	  	}

	  	if (exitpoll !=0) break;



	}
	scanA = 0;
	scanB = 0;

	/************  Sounding at end of scan ******************************/

    	ErrLog(errlog,progname,"Waiting for scan boundary."); 
    	if ((scnsc !=0) || (scnus !=0)) {
      		if (exitpoll==0) OpsWaitBoundary(scnsc,scnus);
    	}
  } while (exitpoll==0);
  SiteEnd();
  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  RShellTerminate(sid);
  return 0;   
} 
 

/***************************************************************************/

/* Sets up the licenced frequency bands for Iceland & finland */

void u_init_freq_bands() {

/* start freq of each band */

   /* Finland */

   usfreq[0] = 8305;
   usfreq[1] = 8965;
   usfreq[2] = 9900;
   usfreq[3] = 11075;
   usfreq[4] = 11550;
   usfreq[5] = 12370;
   usfreq[6] = 13200;
   usfreq[7] = 15010;
   usfreq[8] = 16210;
   usfreq[9] = 16555;
   usfreq[10] = 17970;
   usfreq[11] = 18850;
   usfreq[12] = 19415;
   usfreq[13] = 19705;
   usfreq[14] = 19800;

   /* Iceland */

   usfreq[20] = 8000;
   usfreq[21] = 8430;
   usfreq[22] = 8985;
   usfreq[23] = 10155;
   usfreq[24] = 10655;
   usfreq[25] = 11290;
   usfreq[26] = 11475;
   usfreq[27] = 12105;
   usfreq[28] = 12305;
   usfreq[29] = 12590;
   usfreq[30] = 13360;
   usfreq[31] = 13875;
   usfreq[32] = 14400;
   usfreq[33] = 15805;
   usfreq[34] = 16500;
   usfreq[35] = 16820;
   usfreq[36] = 18175;
   usfreq[37] = 18835;
   usfreq[38] = 19910;
   usfreq[39] = 10155;

   /* width of each band */

   /* Finland */

   ufreq_range[0] = 30;
   ufreq_range[1] = 75;
   ufreq_range[2] = 85;
   ufreq_range[3] = 200;
   ufreq_range[4] = 50;
   ufreq_range[5] = 45;
   ufreq_range[6] = 60;
   ufreq_range[7] = 70;
   ufreq_range[8] = 150;
   ufreq_range[9] = 60;
   ufreq_range[10] = 80;
   ufreq_range[11] = 15;
   ufreq_range[12] = 265;
   ufreq_range[13] = 50;
   ufreq_range[14] = 190; 

   /* Iceland */

   ufreq_range[20] = 195;
   ufreq_range[21] = 420;
   ufreq_range[22] = 410;
   ufreq_range[23] = 500;
   ufreq_range[24] = 520;
   ufreq_range[25] = 160;
   ufreq_range[26] = 120;
   ufreq_range[27] = 130;
   ufreq_range[28] = 205;
   ufreq_range[29] = 690;
   ufreq_range[30] = 205;
   ufreq_range[31] = 120;
   ufreq_range[32] = 615;
   ufreq_range[33] = 560;
   ufreq_range[34] = 185;
   ufreq_range[35] = 655;
   ufreq_range[36] = 595;
   ufreq_range[37] = 50;
   ufreq_range[38] = 90;
   ufreq_range[39] = 1020;
}

/* Sends a proxy message to the microcontroller monitoring 
   task every beam.
*/

void u_read_uconts() {
  if (uucont_proxy != 0) Trigger(uucont_proxy);
}

