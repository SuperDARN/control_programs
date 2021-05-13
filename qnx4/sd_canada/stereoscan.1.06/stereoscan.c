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
#include "iqdata.h"
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
#include "builds.h"
#include "sync.h"
#include "interface.h"
#include "hdw.h"

/*
 $Log: stereoscan.c,v $
 Revision 1.6  2008/03/18 14:35:45  code
 Added support for I&Q sample capture.

 Revision 1.5  2007/10/30 14:28:47  code
 Modifications to get the code working with digital receivers.

 Revision 1.4  2007/08/09 14:09:51  code
 Allowed command lines to be passed as a file.

 Revision 1.3  2007/08/08 16:55:44  code
 Fixed bug in using hex notation for stid.

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



#define CPID_A -6401
#define CPID_B -26401
#define CPID_NS 152
#define CPID_FS 153


#define INTT      7
#define RSEP_A 45
#define RSEP_B 45
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
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite","raw_write","fit_write"

/*
  "fitacfwrite"
*/


void u_read_uconts(void);
void u_init_freq_bands(void);

void RemInvalidEntries(int ifreqsA[], int ifreqsB[]);
void FindNumBands(int *numfreqbandsA, int *numfreqbandsB, int *day_nightA, int *day_nightB, int ifreqsA[], int ifreqsB[], int print);
void RemInvalidBeams(int ibeamsA[], int ibeamsB[]);
void FindNumBeams(int *numbeamsA, int *numbeamsB, int ibeamsA[], int ibeamsB[], int print);

char cmdlne[1024];
char progid[80]={"$Id: stereoscan.c,v 1.6 2008/03/18 14:35:45 code Exp $"};
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

  int ibeamsA[NUMBEAMS],ibeamsB[NUMBEAMS];

  int day_nightA=1;
  int day_nightB=1;

  int numfreqbandsA;
  int numfreqbandsB;
  
  int ifreqsA_index=0; 
  int ifreqsB_index=0;
  
  int ifreqsAband;
  int ifreqsBband;

  int numbeamsA,numbeamsB;

  int ibeamsA_index=0;
  int ibeamsB_index=0;

  unsigned char cts2=0,cts4=0,cts6=0,cts8=0;
  unsigned char cts3=0,cts5=0,cts7=0,cts9=0;
  
  int cpidA=0,cpidB=0;

  for (i=0;i<NUMBANDS;i++) ifreqsA[i]=ifreqsB[i]=-1;
  for (i=0;i<NUMBEAMS;i++) ibeamsA[i]=ibeamsB[i]=-1;
  
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

  /* add new options - individual frequency bands */

  strcpy(a,"b0A");
  for (i = 0; i < NUMBANDS; i++) {
    a[1] = '0' + i; /* change 2nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ifreqsA[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ifreqsA[i]);
  }

  strcpy(a,"b0B");
  for (i = 0; i < NUMBANDS; i++) {
    a[1] = '0' + i; /* change 2nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ifreqsB[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ifreqsB[i]);
  }

  strcpy(a,"bm0A");
  
  for (i = 0; i < 10; i++) {
    a[2] = '0' + i;  /* change 3nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ibeamsA[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsA[i]);
  }

  strcpy(a,"bm10A");
  for (i = 0; i < 6; i++) {
    a[3] = '0' + i;  /* change 4nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ibeamsA[i + 10]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsA[i + 10]);
  }

  strcpy(a,"bm0B");
  for (i = 0; i < 10; i++) {
    a[2] = '0' + i;  /* change 3nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ibeamsB[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsB[i]);
  }

  strcpy(a,"bm10B");
  for (i = 0; i < 6; i++) {
    a[3] = '0' + i; /* change 4nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ibeamsB[i + 10]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsB[i + 10]);
  }

  /* add CTs options */

  OptionAdd(&opt, "cts2", 'x', &cts2);
  OptionAdd(&opt, "cts3", 'x', &cts3);
  OptionAdd(&opt, "cts4", 'x', &cts4);
  OptionAdd(&opt, "cts5", 'x', &cts5);
  OptionAdd(&opt, "cts6", 'x', &cts6);
  OptionAdd(&opt, "cts7", 'x', &cts7);
  OptionAdd(&opt, "cts8", 'x', &cts8);
  OptionAdd(&opt, "cts9", 'x', &cts9);
 
  arg=OptionProcess(1,argc,argv,&opt,NULL);  
 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);  

  /* handle CTs */

  if ((cts2) || (cts4) || (cts6) || (cts8)) {
    cpA = 152;
	intsc = 7;
    intus = 0;
    stereo_offset = 400;
    scnsc = 120;
    scnus = 0;

	/* tbj mode */

    if (cts2) {
	  cpB = -26002;
      if (stid==0x09) {
	    ifreqsB[0] = 23;
		ifreqsB[1] = 26;
		ifreqsB[2] = 28;
		ifreqsB[3] = 30;
		ifreqsB[4] = 32;
		ifreqsB[5] = 34;
		ifreqsB[6] = 35;
      } else if (stid==0x10) {
		ifreqsB[0] = 2;
		ifreqsB[1] = 4;
		ifreqsB[2] = 5;
		ifreqsB[3] = 6;
		ifreqsB[4] = 7;
		ifreqsB[5] = 8;
		ifreqsB[6] = 10;
      }
     }

	 /* normal scan */

	 if (cts4) cpB = -26004;
	
	 /* single camp beam */

	if (cts6) {
	  cpB = -26006;
      if (stid==0x09) ibeamsB[0] = SINGLE_CAMP_BEAM_PYK;
      else ibeamsB[0] = SINGLE_CAMP_BEAM_HAN;

	}

	/* double camp beam */

	if (cts8) {
	  cpB = -26008;

	  ibeamsB[0] = DOUBLE_CAMP_LOW;
	  ibeamsB[1] = DOUBLE_CAMP_HIGH;
	}
  } else if ((cts3) || (cts5) || (cts7) || (cts9)) {
    cpA = 153;
	intsc = 3;
	stereo_offset = 400;
	scnsc = 60.0;
    scnus = 0.0;

	/* tbj */

	if (cts3) {
	  cpB = -26003;
       
	  if (stid==0x09) {
	    ifreqsB[0] = 23;
		ifreqsB[1] = 26;
		ifreqsB[2] = 28;
		ifreqsB[3] = 30;
		ifreqsB[4] = 32;
		ifreqsB[5] = 34;
		ifreqsB[6] = 35;
      } else if (stid==33) {
		ifreqsB[0] = 2;
		ifreqsB[1] = 4;
		ifreqsB[2] = 5;
		ifreqsB[3] = 6;
		ifreqsB[4] = 7;
		ifreqsB[5] = 8;
		ifreqsB[6] = 10;
      }
	}

	/* normal scan */

	if (cts5) cpB = -26005;
	

	/* single camp beam */

	if (cts7) {
	  cpB = -26007;
      if (stid==0x09) ibeamsB[0] = SINGLE_CAMP_BEAM_PYK;
      else ibeamsB[0] = SINGLE_CAMP_BEAM_HAN;

	}

	/* double camp beam */

	if (cts9) {
	  cpB = -26009;

	  ibeamsB[0] = DOUBLE_CAMP_LOW;
	  ibeamsB[1] = DOUBLE_CAMP_HIGH;
	}
  } else
    /* check for normal scan emulation */
  if (ns) {
    frangA		= FRANG_A;
	rsepA		= RSEP_A;
	low_beam_A	= LOW_BEAM_A;
	high_beam_A	= HIGH_BEAM_A;

	intsc = 7;
    intus = 0;
	scnsc = 120.0;
    scnus = 0.0;
	cpA = CPID_NS;  
  } else
    /* check for fast scan emulation */
  if (fs) {
    frangA		= FRANG_A;
	rsepA		= RSEP_A;
	low_beam_A	= LOW_BEAM_A;
	high_beam_A	= HIGH_BEAM_A;

	intsc = 3;
    intus = 0;
	scnsc = 60.0;
    scnus = 0.0;
	cpA = CPID_FS;
  }

  /* set cpid for channel A */

  if (cpidA) {
     cpA = cpidA;
	 cpB = -1 * abs(cpidA) - 20000;
  }

  /* set cpid for channel B */

  if (cpidB) cpB = cpidB;
	
  /* Calculate delay offsets
     If stereo_offset is +ve A is later than B
  */

  if (stereo_offset > 0) {
    delays[1] = 0;
	delays[0] = stereo_offset / 10; /* remember stereo_offset is in usec */
  } else {
    delays[0] = 0;
	delays[1] = (stereo_offset / 10) * (-1); /* stereo_offset is in usec */
  }

  mppulA = mppulB = 7;
  mplgsA = mplgsB = 18;

  txplA = (rsepA * 20) / 3;
  txplB = (rsepB * 20) / 3;

  /* just a dummy frequency */

  tfreqA = 13500;
  tfreqB = 14500; 

  SiteSetupHardware();

  /* remove invalid entries in the frequency band lists */

  RemInvalidEntries(ifreqsA, ifreqsB);

  /* determine the number of frequency bands */

  FindNumBands(&numfreqbandsA, &numfreqbandsB, 
               &day_nightA, &day_nightB, ifreqsA, ifreqsB, 1);

  /* check beams */

  if (low_beam_A  <  LOW_BEAM_A) low_beam_A  = LOW_BEAM_A;
  if (high_beam_A > HIGH_BEAM_A) high_beam_A = HIGH_BEAM_A;

  if (low_beam_B  <  LOW_BEAM_B) low_beam_B  = LOW_BEAM_B;
  if (high_beam_B > HIGH_BEAM_B) high_beam_B = HIGH_BEAM_B;

  /* end beams in the wrong order */

  if (low_beam_A > high_beam_A) {
    int beam = high_beam_A;
    high_beam_A = low_beam_A;
	low_beam_A  = beam;
  }

  if (low_beam_B > high_beam_B) {
    int beam = high_beam_B;
    high_beam_B = low_beam_B;
	low_beam_B  = beam;
  }

  /* sort beam lists */

  RemInvalidBeams(ibeamsA, ibeamsB);			
  FindNumBeams(&numbeamsA, &numbeamsB, ibeamsA, ibeamsB, 1);

  /*
	the beam lists now contain:
	1) the number of beams in the beam lists
	2) if no beam list or camp beams were specified = 0 -> 15
	3) if high and low beams were specified			= low_beam -> high_beam
  */
   
  sprintf(progname,"stereoscan");

  OpsFitACFStart();

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
  }

  /* scan loop */

  do {

    if (SiteStartScan()==0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(tlist[n]);
        RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);     
      }
    }

    if (backward) scanA=scanB=-1;
    else scanA=scanB=1; 

    ErrLog(errlog,progname,"Starting scan.");


    for (ibeamsA_index=0;ibeamsA_index<numbeamsA;ibeamsA_index++) {

      bmnumA=ibeamsA[ibeamsA_index];
      bmnumB=ibeamsB[ibeamsB_index];

	  ifreqsAband = ifreqsA[ifreqsA_index];
	  ifreqsBband = ifreqsB[ifreqsB_index];

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (day_nightA) { 
        if (OpsDayNight()==1) {
           stfrqA=usfreq[dfrqA];
           frqrngA=ufreq_range[dfrqA];
        } else {
           stfrqA=usfreq[nfrqA];
           frqrngA=ufreq_range[nfrqA];
        } 
      } else {
        stfrqA=usfreq[ifreqsAband];
        frqrngA=ufreq_range[ifreqsAband];
      }

      if (day_nightB) { 
        if (OpsDayNight()==1) {
           stfrqB=usfreq[dfrqB];
           frqrngB=ufreq_range[dfrqB];
        } else {
           stfrqB=usfreq[nfrqB];
           frqrngB=ufreq_range[nfrqB];
        } 
      } else {
        stfrqB=usfreq[ifreqsBband];
        frqrngB=ufreq_range[ifreqsBband];
      }

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

      /*
      SiteSetChannel(CHN_A);
      SiteSetFreq(tfreqA);
      SiteSetChannel(CHN_B);
      SiteSetFreq(tfreqB);
      */

      SiteSetFreqS(tfreqA,tfreqB);

	  sprintf(logtxt,"Integrating:%2d,%2d intt:%d (%02d:%02d:%02d) %5d (%02d)  %5d (%02d) - %s  %d\n",
		      bmnumA, bmnumB, intsc, hr, mt, 
              sc, tfreqA, ifreqsAband, tfreqB, ifreqsBband, 
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


      OpsBuildPrmS(0,&prmA,ptab,lags);
      OpsBuildIQS(0,&iqA);
      OpsBuildRawS(0,&rawA);

      OpsBuildPrmS(1,&prmB,ptab,lags);
      OpsBuildIQS(1,&iqB);
      OpsBuildRawS(1,&rawB);

      FitACF(&prmA,&rawA,&fblk,&fitA);
      FitACF(&prmB,&rawB,&fblk,&fitB);

      ErrLog(errlog,progname,"Sending messages."); 
  

      msg.num=0;
      msg.tsize=0;


      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmA,
		PRM_TYPE,0); 

     RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iqA,IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(int),(unsigned char *) &IQoffsetA,IQO_TYPE,0);   

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawA,
		RAW_TYPE,0);    

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitA,
		FIT_TYPE,0);

      RMsgSndAdd(&msg,strlen(progname)+1,progname,
		NME_TYPE,0);   

      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmB,
		PRM_TYPE,1); 


      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iqB,IQ_TYPE,1);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,IQS_TYPE,1);


      RMsgSndAdd(&msg,sizeof(int),(unsigned char *) &IQoffsetB,IQO_TYPE,1);   

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawB,
		RAW_TYPE,1);    

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitB,
		FIT_TYPE,1);

   
      RMsgSndAdd(&msg,strlen(progname)+1,progname,
		NME_TYPE,1);   


      for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 
    
 
      ErrLog(errlog,progname,"Polling for exit."); 
 
      exitpoll=RadarShell(sid,&rstable);

	  /* check the bands and beams here, after radar shell */

	  RemInvalidBeams(ibeamsA, ibeamsB);			
	  FindNumBeams(&numbeamsA, &numbeamsB, ibeamsA, ibeamsB, 0);

	  RemInvalidEntries(ifreqsA, ifreqsB);
	  FindNumBands(&numfreqbandsA, &numfreqbandsB, &day_nightA, &day_nightB, ifreqsA, ifreqsB, 0);

	  if (ifreqsA_index >= numfreqbandsA) ifreqsA_index = 0;
	  if (ifreqsB_index >= numfreqbandsB) ifreqsB_index = 0;
  
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

	  /* change beam number index */

	  ibeamsB_index++;

	  if (ibeamsB_index >= numbeamsB) ibeamsB_index = 0;

	  /* change band at the end of channel B scan */

	  if (ibeamsB_index == 0) {
	    if (numfreqbandsB != 0) {
		  ifreqsB_index++;
          if (ifreqsB_index >= numfreqbandsB) ifreqsB_index = 0;
		}
	  }
    } 
    
	/* increment individual frequency band index. Doesn't matter whether
	   individual frequency bands are being used for this channel or not.
     */

    if (numfreqbandsA != 0) {
      ifreqsA_index++;
      if (ifreqsA_index >= numfreqbandsA) ifreqsA_index = 0;
    }

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



/* remove invalid entries in the frequency band lists */

void RemInvalidEntries(int ifreqsA[NUMBANDS], int ifreqsB[NUMBANDS]) {
  int i, j;

  for (i = NUMBANDS - 1; i > 0; i--) {

	/* if the preceding element is -1 then move all the following
	   bands down
     */

	if ((ifreqsA[i] != -1) && (ifreqsA[i - 1] == -1)) {
	  for (j = i; j < NUMBANDS; j++) {
	    ifreqsA[j - 1] = ifreqsA[j];
		ifreqsA[j] = -1;
	  }
	}

	if ((ifreqsB[i] != -1) && (ifreqsB[i - 1] == -1)) {
	  for (j = i; j < NUMBANDS; j++) {
	    ifreqsB[j - 1] = ifreqsB[j];
		ifreqsB[j] = -1;
	  }
	}
  }
}

/* determine the number of frequency bands */

void FindNumBands(int *numfreqbandsA, int *numfreqbandsB, 
                  int *day_nightA, int *day_nightB, 
                  int ifreqsA[NUMBANDS], int ifreqsB[NUMBANDS], 
                  int print) {
  int i;
  char logtxt[256];

  *numfreqbandsA = 0;
  *numfreqbandsB = 0;

  for (i = 0; i < NUMBANDS; i++) {
    if (ifreqsA[i] != -1) (*numfreqbandsA)++;
	if (ifreqsB[i] != -1) (*numfreqbandsB)++;
  }

  /* check to see if day/night frequencies are to be used */

  if (*numfreqbandsA != 0) *day_nightA = 0;
  else *day_nightA = 1;

  if (*numfreqbandsB != 0) *day_nightB = 0;
  else *day_nightB = 1;

  if (print) {
    sprintf(logtxt, "Channel A: %d  Channel B: %d", *numfreqbandsA, 
            *numfreqbandsB);
	ErrLog(errlog, progname, logtxt);
  }

  /* print frequency bands for each channel */

  if ((*numfreqbandsA > 0) && print) {
    ErrLog(errlog,progname,"Frequency bands for channel A: ");

    for (i = 0; i < *numfreqbandsA; i++) {
	  sprintf(logtxt, "%1d ", ifreqsA[i]);
	  ErrLog(errlog,progname,logtxt);
	}
  }

  if ((*numfreqbandsB > 0) && print) {
    ErrLog(errlog,progname,"Frequency bands for channel B: ");

	for (i = 0; i < *numfreqbandsB; i++) {
	  sprintf(logtxt, "%1d ", ifreqsB[i]);
	  ErrLog(errlog,progname,logtxt);
	}
  }
}

/* remove invalid entries in the beam lists */

void RemInvalidBeams(int ibeamsA[NUMBEAMS], int ibeamsB[NUMBEAMS]) {
  int i, j;

  for (i = NUMBEAMS - 1; i > 0; i--) {
    if (ibeamsA[i] <  0) ibeamsA[i] = -1;
	if (ibeamsA[i] > 15) ibeamsA[i] = -1;

	if (ibeamsB[i] <  0) ibeamsB[i] = -1;
	if (ibeamsB[i] > 15) ibeamsB[i] = -1;
  }

  for (i = NUMBEAMS - 1; i > 0; i--) {

	/* if the preceding element is -1 then move all the 
       following beams down
     */

	if ((ibeamsA[i - 1] == -1) && (ibeamsA[i] != -1)) {
	  for (j = i; j < NUMBEAMS; j++) {
	    ibeamsA[j - 1] = ibeamsA[j];
		ibeamsA[j] = -1;
	  }
	}

	if ((ibeamsB[i - 1] == -1) && (ibeamsB[i] != -1)) {
	  for (j = i; j < NUMBEAMS; j++) {
	    ibeamsB[j - 1] = ibeamsB[j];
		ibeamsB[j] = -1;
	  }
	}
  }
}

/* determine the number of beams */

void FindNumBeams(int *numbeamsA, int *numbeamsB, 
                  int ibeamsA[NUMBEAMS], int ibeamsB[NUMBEAMS], 
                  int print) {
  int i;
  char logtxt[256];

  *numbeamsA = 0;
  *numbeamsB = 0;

  /* count channel A beams */

  for (i = 0; i < NUMBEAMS; i++) {
    if (ibeamsA[i] != -1) (*numbeamsA)++;
	else break;
  }

  /* count channel B beams */

  for (i = 0; i < NUMBEAMS; i++) {
    if (ibeamsB[i] != -1) (*numbeamsB)++;
	else break;
  }

  if (print) {
    sprintf(logtxt, "Beams A: %d  Beams B: %d", *numbeamsA, *numbeamsB);
	ErrLog(errlog, progname, logtxt);

	/* print frequency bands for each channel */

	if (*numbeamsA > 0) {
	  ErrLog(errlog,progname,"Beams for channel A: ");

	  for (i = 0; i < *numbeamsA; i++) {
	    sprintf(logtxt, "%1d ", ibeamsA[i]);
	    ErrLog(errlog,progname,logtxt);
	  }
	}

	if (*numbeamsB > 0) {
	  ErrLog(errlog,progname,"Beams for channel B: ");
       for (i = 0; i < *numbeamsB; i++) {
	     sprintf(logtxt, "%1d ", ibeamsB[i]);
		 ErrLog(errlog,progname,logtxt);
	   }
	}
  }

  /* if no beams have been specified fill the arrays with normal sequence
     between low_beam and high beam.  
   */
  
  if (*numbeamsA == 0) {
    if (backward) {
      for (i = 0; i < (high_beam_A - low_beam_A + 1); i++) {
        ibeamsA[i] = high_beam_A - i;
        (*numbeamsA)++;
 	  }
    } else {
 	  for (i = 0; i < (high_beam_A - low_beam_A + 1); i++) {
	    ibeamsA[i] = low_beam_A + i;
        (*numbeamsA)++;
	  }
    }
  }

  if (*numbeamsB == 0) {

	if (backward) {
      for (i = 0; i < (high_beam_B - low_beam_B + 1); i++) {
  	    ibeamsB[i] = high_beam_B - i;
        (*numbeamsB)++;
	  }
    } else {
  	  for (i = 0; i < (high_beam_B - low_beam_B + 1); i++) {
        ibeamsB[i] = low_beam_B + i;
        (*numbeamsB)++;
	  }
    }
  }
}



