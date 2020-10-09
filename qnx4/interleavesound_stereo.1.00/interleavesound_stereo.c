/* interleavesound_stereo.c
   ========================
   Author: E.G.Thomas
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
 $Log: interleavesound_stereo.c,v $
 Revision 1.0  2020/10/09 egthomas
 Initial revision from stereoscan and stereo_interleave
 
*/


#define CPID_A 191
#define CPID_B 26197


#define INTT    3
#define RSEP_A 45
#define RSEP_B 45
#define FRANG_A 180
#define FRANG_B 180


#define LOW_BEAM_A   0
#define HIGH_BEAM_A  15
#define LOW_BEAM_B   0
#define HIGH_BEAM_B  15

#define DEFAULT_DAY_BAND_A_HKW 2
#define DEFAULT_NIGHT_BAND_A_HKW 2
#define DEFAULT_DAY_BAND_B_HKW 1
#define DEFAULT_NIGHT_BAND_B_HKW 1

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
char progid[80]={"$Id: interleavesound_stereo.c,v 1.0 2020/10/09 egthomas Exp $"};
char progname[256];
struct TaskID *errlog;

int usfreq[121];
int ufreq_range[121];

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

  int ptabA[7] = {0,9,12,20,22,26,27};

  int lagsA[LAG_SIZE][2] = {
    { 0, 0},        /*  0 */
    {26,27},        /*  1 */
    {20,22},        /*  2 */
    { 9,12},        /*  3 */
    {22,26},        /*  4 */
    {22,27},        /*  5 */
    {20,26},        /*  6 */
    {20,27},        /*  7 */
    {12,20},        /*  8 */
    { 0, 9},        /*  9 */
    {12,22},        /* 10 */
    { 9,20},        /* 11 */
    { 0,12},        /* 12 */
    { 9,22},        /* 13 */
    {12,26},        /* 14 */
    {12,27},        /* 15 */
    { 9,26},        /* 16 */
    { 9,27},        /* 17 */
    {27,27}};		/* alternate lag-0  */


  int ptabB[8] = {0,14,22,24,27,31,42,43};

  int lagsB[LAG_SIZE][2] = {
    { 0, 0},        /*  0 */
    {42,43},        /*  1 */
    {22,24},        /*  2 */
    {24,27},        /*  3 */
    {27,31},        /*  4 */
    {22,27},        /*  5 */

    {24,31},        /*  7 */
    {14,22},        /*  8 */
    {22,31},        /*  9 */
    {14,24},        /* 10 */
    {31,42},        /* 11 */
    {31,43},        /* 12 */
    {14,27},        /* 13 */
    { 0,14},        /* 14 */
    {27,42},        /* 15 */
    {27,43},        /* 16 */
    {14,31},        /* 17 */
    {24,42},        /* 18 */
    {24,43},        /* 19 */
    {22,42},        /* 20 */
    {22,43},        /* 21 */
    { 0,22},        /* 22 */

    { 0,24},        /* 24 */
    {43,43}};       /* alternate lag-0 */

  mppulA=7;
  mplgsA=18;
  mppulB=8;
  mplgsB=19;

  unsigned char katscan=0;

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
 
  int stereo_offset=400; /* channel A delayed by this 
                            number of microseconds */

  int ifreqsA[NUMBANDS],ifreqsB[NUMBANDS];

  int ibeamsA[NUMBEAMS],ibeamsB[NUMBEAMS];

  int day_nightA=1;
  int day_nightB=0;

  int numfreqbandsA;
  int numfreqbandsB;

  int ifreqsA_index=0;
  int ifreqsB_index=0;

  int ifreqsAband;
  int ifreqsBband;

  int numbeamsA,numbeamsB;

  int ibeamsA_index=0;
  int ibeamsB_index=0;

  for (i=0;i<NUMBANDS;i++) ifreqsA[i]=ifreqsB[i]=-1;
  for (i=0;i<NUMBEAMS;i++) ibeamsA[i]=ibeamsB[i]=-1;

  xcfA=xcfB=1;

  cpA=CPID_A;
  cpB=CPID_B;

  SiteStart();

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
  strncpy(combfA,progid,80);
  strncpy(combfB,progid,80);

  OpsSetupCommand(argc,argv);
  OpsSetupRadar();
  OpsSetupShell();

  if (stid==41) {
	dfrqA=DEFAULT_DAY_BAND_A_HKW;
	nfrqA=DEFAULT_NIGHT_BAND_A_HKW;
	dfrqB=DEFAULT_DAY_BAND_B_HKW;
	nfrqB=DEFAULT_NIGHT_BAND_B_HKW;
  }

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

  OptionAdd(&opt, "e",	't', &ename);
  OptionAdd(&opt, "sc",	't', &sname);
  OptionAdd(&opt, "dfA",	'i', &dfrqA);
  OptionAdd(&opt, "nfA",	'i', &nfrqA);

  OptionAdd(&opt, "offset",	'i',&stereo_offset);

  OptionAdd(&opt, "frA",	'i', &frangA);
  OptionAdd(&opt, "frB",	'i', &frangB);
  OptionAdd(&opt, "rgA",	'i', &rsepA);
  OptionAdd(&opt, "rgB",	'i', &rsepB);

  OptionAdd(&opt, "katscan", 'x', &katscan);

  /* set up remaining shell variables */

  RadarShellAdd(&rstable, "dfA",var_LONG,&dfrqA);
  RadarShellAdd(&rstable, "nfA",var_LONG,&nfrqA);

  RadarShellAdd(&rstable, "offset",	var_LONG,&stereo_offset);

  /* add new options - individual frequency bands (for Channel B only!) */

  strcpy(a,"b0B");
  for (i = 0; i < NUMBANDS; i++) {
    a[1] = '0' + i; /* change 2nd char to ACSII number */
    OptionAdd(&opt, a,	'i', &ifreqsB[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ifreqsB[i]);
  }

  arg=OptionProcess(1,argc,argv,&opt,NULL);

  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);
  OpsLogStart(errlog,progname,argc,argv);

  if (stid == 9) {
    ifreqsB[0] = 23;
    ifreqsB[1] = 26;
    ifreqsB[2] = 28;
    ifreqsB[3] = 30;
    ifreqsB[4] = 32;
    ifreqsB[5] = 34;
    ifreqsB[6] = 35;
  } else if (stid == 10) {
    ifreqsB[0] = 2;
    ifreqsB[1] = 4;
    ifreqsB[2] = 5;
    ifreqsB[3] = 6;
    ifreqsB[4] = 7;
    ifreqsB[5] = 8;
    ifreqsB[6] = 10;
  } else if (stid == 41) {
    ifreqsb[0] = 1;
    ifreqsb[1] = 2;
    ifreqsb[2] = 3;
    ifreqsb[3] = 4;
    ifreqsb[4] = -1;
    ifreqsb[5] = -1;
    ifreqsb[6] = -1;
  } else {
    printf("Need to add Channel B frequency bands! (stid: %d)\n",stid);
    exit(-1);
  }

  frangA = FRANG_A;
  rsepA  = RSEP_A;
  frangB = FRANG_B;
  rsepB  = RSEP_B;

  intsc = 3;
  intus = 0;
  scnsc = 60.0;
  scnus = 0.0;

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

  /* Set pulse sequence */
  if (katscan) {
    ptab = ptab2;
    lags = lags2;
    mppulA = mppulB = mppul2;
    mplgsA = mplgsB = mplgs2;
  } else {
    ptab = ptab1;
    lags = lags1;
    mppulA = mppulB = mppul1;
    mplgsA = mppulB = mplgs1;
  }

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

  /* sort beam lists */

  RemInvalidBeams(ibeamsA, ibeamsB);			
  FindNumBeams(&numbeamsA, &numbeamsB, ibeamsA, ibeamsB, 1);

  /*
	the beam lists now contain:
	1) the number of beams in the beam lists
	2) an interleaved scanning sequence for Channel A
	3) a normal azimuthal scanning sequence for Channel B
  */
   
  sprintf(progname,"interleavesound_stereo");

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

      /* Channel A uses day/night frequencies */
      if (OpsDayNight()==1) {
         stfrqA=usfreq[dfrqA];
         frqrngA=ufreq_range[dfrqA];
      } else {
         stfrqA=usfreq[nfrqA];
         frqrngA=ufreq_range[nfrqA];
      } 

      /* Channel B uses sounding frequencies */
      stfrqB=usfreq[ifreqsBband];
      frqrngB=ufreq_range[ifreqsBband];

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


      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmA, PRM_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iqA,IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(int),(unsigned char *) &IQoffsetA,IQO_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawA, RAW_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitA, FIT_TYPE,0);

      RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,0);


      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmB, PRM_TYPE,1);

      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iqB,IQ_TYPE,1);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,IQS_TYPE,1);

      RMsgSndAdd(&msg,sizeof(int),(unsigned char *) &IQoffsetB,IQO_TYPE,1);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawB, RAW_TYPE,1);

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitB, FIT_TYPE,1);

      RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,1);


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

/* Sets up the licenced frequency bands for [your radar] */

void u_init_freq_bands() {

/* start freq of each band */

   /* HKW */

   usfreq[0] = 9150;	//keep bands 0 and 1 the same to avoid use of 0
   usfreq[1] = 9150;
   usfreq[2] = 10790;
/* remove 11.07 MHz band because it is for HOK East
   usfreq[3] = 11050;
   usfreq[4] = 15830;
*/
   usfreq[3] = 15830;
   usfreq[4] = 18180;

   usfreq[5] = 18180;
   usfreq[6] = 18180;
   usfreq[7] = 18180;
   usfreq[8] = 18180;
   usfreq[9] = 18180;
   usfreq[10] = 18180;
   usfreq[11] = 18180;
   usfreq[12] = 18180;
   usfreq[13] = 18180;
   usfreq[14] = 18180;
   usfreq[15] = 18180;
   usfreq[16] = 18180;
   usfreq[17] = 18180;
   usfreq[18] = 18180;
   usfreq[19] = 18180;
   usfreq[20] = 18180;

   /* width of each band */

   /* HKW */

   ufreq_range[0] = 40;
   ufreq_range[1] = 40;
   ufreq_range[2] = 40;
   ufreq_range[3] = 40;
   ufreq_range[4] = 40;
   ufreq_range[5] = 40;
   ufreq_range[6] = 40;
   ufreq_range[7] = 40;
   ufreq_range[8] = 40;
   ufreq_range[9] = 40;
   ufreq_range[10] = 40;
   ufreq_range[11] = 40;
   ufreq_range[12] = 40;
   ufreq_range[13] = 40;
   ufreq_range[14] = 40;
   ufreq_range[15] = 40;
   ufreq_range[16] = 40;
   ufreq_range[17] = 40;
   ufreq_range[18] = 40;
   ufreq_range[19] = 40;
   ufreq_range[20] = 40;
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

  int forward_beams[16] = { 0, 4, 8,12, 2, 6,10,14, 1, 5, 9,13, 3, 7,11,15 };
  int backward_beams[16]= {15,11, 7, 3,13, 9, 5, 1,14,10, 6, 2,12, 8, 4, 0 };

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
        ibeamsA[i] = backward_beams[i];
        (*numbeamsA)++;
 	  }
    } else {
 	  for (i = 0; i < (high_beam_A - low_beam_A + 1); i++) {
	    ibeamsA[i] = forward_beams[i];
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



