/* stereo_sound.c
   ==============
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
 $Log: stereo_sound.c,v $
 Revision 1.0  2023/04/13 egthomas
 Initial revision from stereoscan and normalsound
 
*/


#define CPID_A  152
#define CPID_B  20155
#define CPID_NS 152
#define CPID_FS 153


#define INTT      7
#define RSEP_A   45
#define RSEP_B   45
#define FRANG_A 180
#define FRANG_B 180


#define LOW_BEAM_A   0
#define HIGH_BEAM_A  15
#define LOW_BEAM_B   0
#define HIGH_BEAM_B  15

#define DEFAULT_BAND_A_HKW 2

#define DEFAULT_BAND_A_LYR 25

#define UCONT_NAME "ucont_moni"

#define CHN_A 0
#define CHN_B 1

#define NUMBANDS 17
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
char progid[80]={"stereo_sound.c 1.0 2023/04/13 egthomas"};
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

  int ptab[7] = {0,9,12,20,22,26,27};

  int lags[LAG_SIZE][2] = {
    { 0, 0},    /*  0 */
    {26,27},    /*  1 */
    {20,22},    /*  2 */
    { 9,12},    /*  3 */
    {22,26},    /*  4 */
    {22,27},    /*  5 */
    {20,26},    /*  6 */
    {20,27},    /*  7 */
    {12,20},    /*  8 */
    { 0, 9},    /*  9 */
    {12,22},    /* 10 */
    { 9,20},    /* 11 */
    { 0,12},    /* 12 */
    { 9,22},    /* 13 */
    {12,26},    /* 14 */
    {12,27},    /* 15 */

    { 9,26},    /* 17 */
    { 9,27},    /* 18 */
    {27,27}};   /* alternate lag-0 */


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

  int day_nightA=0;
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
  strncpy(combfA,progid,80);
  strncpy(combfB,progid,80);

  OpsSetupCommand(argc,argv);
  OpsSetupRadar();
  OpsSetupShell();

  //set up default frequencies
  if (stid == 41) {
    dfrqA = DEFAULT_BAND_A_HKW;
    nfrqA = DEFAULT_BAND_A_HKW;
  } else if (stid == 90) {
    dfrqA = DEFAULT_BAND_A_LYR;
    nfrqA = DEFAULT_BAND_A_LYR;
  } else {
    printf("stereo_sound not ready for this radar: %d\n",stid);
    exit(-1);
  }

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

  OptionAdd(&opt, "ns",   'x', &ns);
  OptionAdd(&opt, "fs",   'x', &fs);
  OptionAdd(&opt, "e",    't', &ename);
  OptionAdd(&opt, "sc",   't', &sname);
  OptionAdd(&opt, "intt", 'i', &intsc);

  OptionAdd(&opt, "offset", 'i', &stereo_offset);

  OptionAdd(&opt, "frA", 'i', &frangA);
  OptionAdd(&opt, "frB", 'i', &frangB);
  OptionAdd(&opt, "rgA", 'i', &rsepA);
  OptionAdd(&opt, "rgB", 'i', &rsepB);
  OptionAdd(&opt, "lbA", 'i', &low_beam_A);
  OptionAdd(&opt, "hbA", 'i', &high_beam_A);
  OptionAdd(&opt, "lbB", 'i', &low_beam_B);
  OptionAdd(&opt, "hbB", 'i', &high_beam_B);
  OptionAdd(&opt, "sp",  'i', &scnsc);

  OptionAdd(&opt, "nrangA", 'i', &nrangA);
  OptionAdd(&opt, "nrangB", 'i', &nrangB);

  /* set up remaining shell variables */

  RadarShellAdd(&rstable, "offset", var_LONG,&stereo_offset);

  RadarShellAdd(&rstable, "low_beamA",  var_LONG,&low_beam_A);
  RadarShellAdd(&rstable, "high_beamA", var_LONG,&high_beam_A);
  RadarShellAdd(&rstable, "low_beamB",  var_LONG,&low_beam_B);
  RadarShellAdd(&rstable, "high_beamB", var_LONG,&high_beam_B);
  RadarShellAdd(&rstable, "scan_period",var_LONG,&scnsc);

  /* add new options - custom beams */

  strcpy(a,"bm0A");

  for (i = 0; i < 10; i++) {
    a[2] = '0' + i;  /* change 3nd char to ACSII number */
    OptionAdd(&opt, a, 'i', &ibeamsA[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsA[i]);
  }

  strcpy(a,"bm10A");
  for (i = 0; i < 6; i++) {
    a[3] = '0' + i;  /* change 4nd char to ACSII number */
    OptionAdd(&opt, a, 'i', &ibeamsA[i + 10]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsA[i + 10]);
  }

  strcpy(a,"bm0B");
  for (i = 0; i < 10; i++) {
    a[2] = '0' + i;  /* change 3nd char to ACSII number */
    OptionAdd(&opt, a, 'i', &ibeamsB[i]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsB[i]);
  }

  strcpy(a,"bm10B");
  for (i = 0; i < 6; i++) {
    a[3] = '0' + i; /* change 4nd char to ACSII number */
    OptionAdd(&opt, a, 'i', &ibeamsB[i + 10]);
    RadarShellAdd(&rstable, a, var_LONG, &ibeamsB[i + 10]);
  }

  arg=OptionProcess(1,argc,argv,&opt,NULL);
 
  if (sname==NULL) sname=sdname;
  if (ename==NULL) ename=edname;

  sid=RShellRegister(sname,CONTROL_NAME);

  errlog=TaskIDMake(ename);
  OpsLogStart(errlog,progname,argc,argv);

  /* Channel B sounding frequencies */
  if (stid == 41) {
    ifreqsB[0] = 1;
    ifreqsB[1] = 3;
    ifreqsB[2] = 4;
  } else if (stid == 90) {
    ifreqsB[0] = 21;
    ifreqsB[1] = 23;
    ifreqsB[2] = 27;
    ifreqsB[3] = 29;
    ifreqsB[4] = 32;
    ifreqsB[5] = 35;
  } else {
    printf("stereo_sound not ready for this radar: %d\n",stid);
    exit(-1);
  }

  /* check for 2-min normal scan emulation */
  if (ns) {
    frangA      = FRANG_A;
    rsepA       = RSEP_A;
    low_beam_A  = LOW_BEAM_A;
    high_beam_A = HIGH_BEAM_A;

    intsc = 7;
    intus = 0;
    scnsc = 120.0;
    scnus = 0.0;
    cpA = CPID_NS;
    cpB = 20155;
  } else {
    /* 1-min fast scan emulation */
    frangA      = FRANG_A;
    rsepA       = RSEP_A;
    low_beam_A  = LOW_BEAM_A;
    high_beam_A = HIGH_BEAM_A;

    intsc = 3;
    intus = 0;
    scnsc = 60.0;
    scnus = 0.0;
    cpA = CPID_FS;
    cpB = 20157;
  }

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
    3) if high and low beams were specified         = low_beam -> high_beam
  */

  sprintf(progname,"stereo_sound");

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
printf("***ibeamsA_index=%d\n",ibeamsA_index);
printf("***ibeamsB_index=%d\n",ibeamsB_index);
printf("***ifreqsA_index=%d\n",ifreqsA_index);
printf("***ifreqsB_index=%d\n",ifreqsB_index);
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      /* Channel A uses fixed frequencies */
      stfrqA=usfreq[dfrqA];
      frqrngA=ufreq_range[dfrqA];

      /* Channel B uses sounding frequencies */
      stfrqB=usfreq[ifreqsBband];
      frqrngB=ufreq_range[ifreqsBband];

      SiteSetChannel(CHN_B);
      SiteSetBeam(bmnumB);
      SiteSetChannel(CHN_A);
      SiteSetBeam(bmnumA);

      SiteSetIntt(intsc,intus);
printf("***SiteSet done\n");
printf("stfrqA=%d, frqrngA=%d, stfrqB=%d, frqrngB=%d\n",stfrqA,frqrngA,stfrqB,frqrngB);
      if (SiteFCLRS(stfrqA,stfrqA+frqrngA,stfrqB,
                    stfrqB+frqrngB)==FREQ_LOCAL)
      ErrLog(errlog,progname,"Frequency Synthesizer in local mode.");

printf("***SiteFCLRS done\n");
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


      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmA,PRM_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iqA,IQ_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,IQS_TYPE,0);

      RMsgSndAdd(&msg,sizeof(int),(unsigned char *) &IQoffsetA,IQO_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawA,RAW_TYPE,0);

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitA,FIT_TYPE,0);

      RMsgSndAdd(&msg,strlen(progname)+1,progname,NME_TYPE,0);


      RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prmB,PRM_TYPE,1);

      RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iqB,IQ_TYPE,1);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory,IQS_TYPE,1);

      RMsgSndAdd(&msg,sizeof(int),(unsigned char *) &IQoffsetB,IQO_TYPE,1);

      RMsgSndAdd(&msg,sizeof(struct RawData),(unsigned char *) &rawB,RAW_TYPE,1);

      RMsgSndAdd(&msg,sizeof(struct FitData),(unsigned char *) &fitB,FIT_TYPE,1);

      RMsgSndAdd(&msg,strlen(progname)+1,progname,NME_TYPE,1);


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

/* Sets up the licenced frequency bands for radars */

void u_init_freq_bands() {

/* start freq of each band */

   /* HKW */

   usfreq[0] = 9150;    //keep bands 0 and 1 the same to avoid use of 0
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

   /* LYR */

   usfreq[20] = 8050;	//keep bands 0 and 1 the same to avoid use of 0
   usfreq[21] = 8050;
   usfreq[22] = 9400;
   usfreq[23] = 9600;
   usfreq[24] = 9800;
   usfreq[25] = 11600;
   usfreq[26] = 11800;
   usfreq[27] = 12000;
   usfreq[28] = 13570;
   usfreq[29] = 13770;
   usfreq[30] = 15100;
   usfreq[31] = 15350;
   usfreq[32] = 15700;
   usfreq[33] = 17480;
   usfreq[34] = 17600;
   usfreq[35] = 17800;
   usfreq[36] = 18900;


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

   /* LYR */

   ufreq_range[20] = 100;
   ufreq_range[21] = 100;
   ufreq_range[22] = 100;
   ufreq_range[23] = 100;
   ufreq_range[24] = 100;
   ufreq_range[25] = 100;
   ufreq_range[26] = 100;
   ufreq_range[27] = 100;
   ufreq_range[28] = 100;
   ufreq_range[29] = 100;
   ufreq_range[30] = 100;
   ufreq_range[31] = 100;
   ufreq_range[32] = 100;
   ufreq_range[33] = 100;
   ufreq_range[34] = 100; 
   ufreq_range[35] = 100; 
   ufreq_range[36] = 100; 
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



