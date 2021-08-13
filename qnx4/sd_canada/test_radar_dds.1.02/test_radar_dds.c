/* test_radar_dds.c
   ============
   Author: Dieter Andre
*/


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/name.h>
#include <string.h>
#include <time.h>
#include <process.h>
#include <fcntl.h>

#include "rtypes.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iqdata.h"

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

#include "build.h"
#include "sync.h"
#include "hdw.h"


#include "rawfeed.h"
#include "ddslib.h"
#include "gc214.h"
#include "iqcopy.h"
#include "shmem.h"

/*
 $Log: test_radar_dds.c,v $
 Revision 1.1  2011/08/13 17:00:00  DAndre
 Initial revision
 
*/

#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite"

#define DRNAME "gc214"
#define RAWFEEDNAME "echoraw"
#define MAXTSG 32
#define REAL_BUF_OFFSET 1
#define IMAG_BUF_OFFSET 0

char cmdlne[1024];
char progid[80]={"$Id: test_radar_dds.c,v 1.01 2008/11/20 15:15:00 DAndre Exp $"};
char progname[256];
struct TaskID *errlog;

char *tasklist[]=
 { TASK_NAMES,
  0};

int TestTimeSeq(int *patn) {
  int status;
  struct TSGprm prm;
  struct TSGbuf *buf=NULL;
  
  int i,id;
  int flag;
  int ss_to_x;

  memset(&prm,0,sizeof(struct TSGprm));

  prm.nrang=nrang;
  prm.frang=frang;
  prm.rtoxmin=0;
  prm.stdelay=0;
  prm.rsep=rsep;
  prm.smsep=smsep;
  prm.txpl=txpl; 
  prm.mpinc=mpinc;
  prm.mppul=mppul; 
  prm.mlag=0;
  prm.nbaud=1;
  prm.code=NULL;
  prm.gort=0; /* gated for GC214; 20070914 DAndre */
  prm.pat=malloc(sizeof(int)*prm.mppul);
  for (i=0;i<prm.mppul;i++) prm.pat[i]=patn[i];
  id=TSGCheck(&prm,tsgtable);

/*  if ((id !=-1) && ((status=DIOVerifyID(dioid,id)) !=0)) {  */
/* DIOVerify is not part of dio.c !! */
  if ((id !=-1)) {
    free(prm.pat);
    lagfr=tsgtable->buf[id].lagfr;
    smsep=tsgtable->buf[id].smsep;
    txpl=tsgtable->buf[id].txpl;
    return id;
  }

  /* new timing sequence so download it to radops_dio */
  printf( "Before TSGMake.\n");
  printf( "nrang: %d frang: %d rsep: %d\n", prm.nrang, prm.frang, prm.rsep);
  printf( "smsep: %d lagfr: %d txpl: %d\n", prm.smsep, prm.lagfr, prm.txpl);
  printf( "mppul: %d mpinc: %d mlag: %d\n", prm.mppul, prm.mpinc, prm.mlag);
  printf( "nbaud: %d samples: %d smdelay: %d\n", prm.nbaud, prm.samples, prm.smdelay);
  printf( "stdelay: %d gort: %d rtoxmin: %d\n\n", prm.stdelay, prm.gort, prm.rtoxmin);

  buf=TSGMake(&prm,&flag);

  printf( "After TSGMake.\n");
  printf( "nrang: %d frang: %d rsep: %d\n", prm.nrang, prm.frang, prm.rsep);
  printf( "smsep: %d lagfr: %d txpl: %d\n", prm.smsep, prm.lagfr, prm.txpl);
  printf( "mppul: %d mpinc: %d mlag: %d\n", prm.mppul, prm.mpinc, prm.mlag);
  printf( "nbaud: %d samples: %d smdelay: %d\n", prm.nbaud, prm.samples, prm.smdelay);
  printf( "stdelay: %d gort: %d rtoxmin: %d\n\n", prm.stdelay, prm.gort, prm.rtoxmin);
  
  if (buf==NULL) {
     free(prm.pat);
     return -1;
  }

  /* HACK */
  smsep= 20;
  prm.smsep= 20;
  rsep= 3;
  prm.rsep= 3;

  if (id==-1) id=TSGAdd(&prm,tsgtable); 
  if (id !=-1) {
/* status=DIOSetTSG(dioid,id,buf->len,buf->code,buf->rep,NULL); */
    printf( "Enter delay between ss high and tx start [> 75 usec]:\n");
    scanf( "%d", &ss_to_x);
    status= DDSVSetTSG( id, prm, ss_to_x);
    if (status==0) { 
      lagfr=prm.lagfr;
      smsep=prm.smsep;
      txpl=prm.txpl;
    } else {
     TSGRemove(tsgtable,id);
     id=status;
    }
  }
   free(prm.pat);
   TSGFree(buf);
   return id; 
}



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

  char *ename=NULL;
  char *edname={ERRLOG};

  int n, ij;
  float thetai;

/*===================================================================*/
  struct TSGprm *tsg_prm;
  int tsgid;
  int badrng= MAX_RANGE;
  ipcid_t drid;
  ipcid_t rawid;
  int roff= REAL_BUF_OFFSET;
  int ioff= IMAG_BUF_OFFSET;
  int bufnum;
  int bufsze;
  void *bufadr[ 16];
  float noise;
 
  struct timespec tick;
  int abuf,buf;
  int iqsze,iqoff; 
  int nchannel;
  int nsample;
 
  int status=0;
  float srate=3333;
/*===================================================================*/


  strcpy(cmdlne,argv[0]);
  for (n=1;n<argc;n++) {
    strcat(cmdlne," ");
    strcat(cmdlne,argv[n]);
  } 

  strncpy(combf,progid,80);
  OpsSetupRadar();

  cp= 3333;
  intsc=7;
  intus=0;
  mppul=8;
  mplgs=23;
  mpinc=1500;
  dmpinc=1500;
  nrang=75;
  rsep=45;
  txpl=300;

  /* from SiteStart */
  backward=0;
  sbm=0;
  ebm=15;
  xcnt=0;
  day=12;
  night=22;
  dfrq=14400;
  nfrq=10200;
  frqrng=300;
  status= DDSInit();
  if (status != DDS_CMD_OK) {
    fprintf(stderr, "Failed to locate DDS socket.");
    exit( -1);
  }


  if (ename==NULL) ename=edname;
  errlog=TaskIDMake(ename);  
  OpsLogStart(errlog,progname,argc,argv);

  /* from SiteSetupHardware */
#ifdef _QNX4
  drid=qnx_name_locate(0,DRNAME,0,NULL);
  rawid=qnx_name_locate(0,RAWFEEDNAME,0,NULL);

  if (drid==-1) {
    fprintf(stderr,"Failed to locate hardware.");
    exit(-1);
  }
#endif
  bufnum=GC214GetBufNum(drid);
  bufsze=GC214GetBufSize(drid);

  for (n=0;n<bufnum;n++) {
    bufadr[n]=GC214GetBufAdr(drid,n);
  }
  tsgtable=TSGMakeTable(MAXTSG);

  samples=(int16 *)
          ShMemAlloc(sharedmemory,IQBUFSIZE,O_RDWR |O_CREAT,1,&shmemfd);

  xcf= 1;
  txpl=(rsep*20)/3;

  sprintf(progname,"test_radar_dds");

  OpsSetupTask(tasklist);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(tlist[n]);
    RMsgSndOpen(tlist[n],strlen(cmdlne),cmdlne);
  }

  printf( "Enter beam number:\n");
  scanf( "%d", &bmnum);
  printf( "Enter transmit frequency [kHz]:\n");
  scanf( "%d", &tfreq);

  for (ij= 0; ij<5; ij++) {
  thetai= site->bmsep* (bmnum - (site->maxbeam/2.0 - 0.5));
  status= DDSSetAzimuth( thetai);
  /* identical tx and rx frequencies */
  status= DDSSetFreq( tfreq);
  status= GC214SetTxFreq(drid,tfreq);
  tsgid= TestTimeSeq(ptab);
  tsg_prm= &tsgtable->buf[tsgid];

  /* From: DDSIntegrateGC214 */

  nchannel = 2;
  buf=0;

  nsample= tsg_prm->samples;
  nsample+= tsg_prm->smdelay-1;

  smpnum= nsample;
  skpnum= tsg_prm->smdelay;
  rxchn= nchannel;
  iqsze= 0;
  iqoff= 0;
  RawFeedSet(rawid, tsg_prm,mplgs,lagnum,lags,0,0, noise,bmnum,tfreq,&tick);

  srate=300000.0/(2.0* tsg_prm->rsep); /* 1/smsep = sample frequency */
  status= GC214SetSRate(drid,srate);

/*  status= DDSIntegrateGC214Pulse(buf,nsample, nchannel,tsgid, drid); */
  status= GC214DoScan(drid,buf, nsample, nchannel);
  status= DDSVSendTSG( tsgid);

  abuf = buf;
  buf = (buf + 1) % 2 ;

  status=GC214GetScanStatus(drid)==SCAN_OK;
  if (status==1) RawFeedScan(rawid,abuf,nsample,nchannel,0,&tick);

  if ((samples !=NULL) && (status==1)) {
    seqtval[nave].tv_sec=tick.tv_sec;
    seqtval[nave].tv_nsec=tick.tv_nsec;
    seqatten[nave]=0;
    seqnoise[nave]= noise;
    seqoff[nave]=iqsze/2; /* in words */
    seqsze[nave]=(nsample*nchannel)*2; /* in words */
  }

  if (status==1) {
    /* copy into our I&Q save buffer */
    if ((samples !=NULL) && ((iqsze+(nsample*nchannel)*4)<IQBUFSIZE)) {
      IQCopy( (int16 *) bufadr[abuf],samples+iqoff,nsample,nchannel,roff,ioff,2,2*nsample);
      iqsze+=(nsample*nchannel)*4;
      iqoff=iqsze/2;
    }
  }
for (n=0; n < 200; n++) {
  if ( (n % 10) == 0 ) printf( "\n");
  printf( "%d ", samples[ n]);
}
  RawFeedTrigger(rawid);
  nave= nave + 1;
  OpsBuildPrm(&prm,ptab,lags);
  OpsBuildIQ(&iq);
  ErrLog(errlog,progname,"Sending messages."); 

  msg.num=0;
  msg.tsize=0;

  RMsgSndAdd(&msg,sizeof(struct RadarParm),(unsigned char *) &prm, PRM_TYPE,0); 
  RMsgSndAdd(&msg,sizeof(struct IQData),(unsigned char *) &iq, IQ_TYPE,0);
  RMsgSndAdd(&msg,strlen(sharedmemory)+1,sharedmemory, IQS_TYPE,0);
  RMsgSndAdd(&msg,strlen(progname)+1,progname, NME_TYPE,0);
  for (n=0;n<tnum;n++) RMsgSndSend(tlist[n],&msg); 

  }

  /* from SiteEnd */
  if (samples !=NULL)
  ShMemFree((unsigned char *) samples,sharedmemory,IQBUFSIZE,1,shmemfd);

  status= DDSTerminate();
  if (status != DDS_CMD_OK) {
    fprintf(stderr, "Failed to close DDS Socket.");
    exit( -1);
  }

  for (n=0;n<tnum;n++) RMsgSndClose(tlist[n]);
  ErrLog(errlog,progname,"Ending program.");
  return 0;
}

