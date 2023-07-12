/* uafsound.c
   ============
   Author: R.J.Barnes & J.Spaleta & J.Klein & E.G.Thomas
*/

/*
 LICENSE AND DISCLAIMER
 
 Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
 This file is part of the Radar Software Toolkit (RST).
 
 RST is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.

 RST is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with RST.  If not, see <http://www.gnu.org/licenses/>.
 
 
  
*/
/* Includes provided by the OS environment */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <argtable2.h>
#include <zlib.h>
#include <math.h>

/* Includes provided by the RST */ 
#include "rtypes.h"
#include "rtime.h"
#include "dmap.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iq.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"
#include "errlog.h"
#include "tcpipmsg.h"
#include "rmsg.h"
#include "rmsgsnd.h"
#include "build.h"
#include "global.h"
#include "reopen.h"
#include "setup.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"

/* sorry, included for checking sanity checking pcode sequences with --test (JTK)*/
#include "tsg.h" 
#include "maketsg.h"

#include "sndwrite.h"

/* Argtable define for argument error parsing */
#define ARG_MAXERRORS 30

#define MAX_INTEGRATIONS_PER_SCAN 100

#define MAX_SND_FREQS 12
#define RT_TASK 3

void write_snd_record(char *progname, struct RadarParm *prm,
                      struct FitData *fit);

int main(int argc,char *argv[]) {
  char progid[80]={"uafsound 2023/07/12"};
  char progname[256]="uafsound";
  char modestr[32];

  char *roshost=NULL;
  char *droshost={"127.0.0.1"};

  char *ststr=NULL;

  char *libstr=NULL;
  char *verstr=NULL;

  int status=0,n,i;
  int nerrors=0;

  /* Variables need for interprocess communications */
  char logtxt[1024];
  void *tmpbuf;
  size_t tmpsze;
/*
  struct TCPIPMsgHost shell={"127.0.0.1",44101,-1};
*/
  int tnum=4;      
  struct TCPIPMsgHost task[4]={
    {"127.0.0.1",1,-1}, /* iqwrite */
    {"127.0.0.1",2,-1}, /* rawacfwrite */
    {"127.0.0.1",3,-1}, /* fitacfwrite */
    {"127.0.0.1",4,-1}  /* rtserver */
  };

/* Define the available barker codes for phasecoding*/
  int *bcode=NULL;
  int bcode1[1]={1};
  int bcode2[2]={1,-1};
  int bcode3[3]={1,1,-1};
  int bcode4[4]={1,1,-1,1};
  int bcode5[5]={1,1,1,-1,1};
  int bcode7[7]={1,1,1,-1,-1,1,-1};
  int bcode11[11]={1,1,1,-1,-1,-1,1,-1,-1,1,-1};
  int bcode13[13]={1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

  /* lists for parameters across a scan, need to send to usrp_server for swings to work.. */
  int32_t scan_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t scan_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t scan_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t nBeams_per_scan = 0;
  int current_beam, iBeam;

  /* lists for parameters across a sounding scan, need to send to usrp_server for swings to work.. */
  int32_t snd_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_bc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_fc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_nBeams_per_scan = 0;
  int snd_iBeam;

  /* time sync of integration periods/ beams */
  int sync_scan;
  int time_now,  time_to_wait; /* times in ms for period synchronization */
  int *scan_times;  /* scan times in ms */

/* Pulse sequence Table */
  int ptab[8] = {0,14,22,24,27,31,42,43};

/* Lag sequence Table */
  int lags[LAG_SIZE][2] = {
    { 0, 0},		/*  0 */
    {42,43},		/*  1 */
    {22,24},		/*  2 */
    {24,27},		/*  3 */
    {27,31},		/*  4 */
    {22,27},		/*  5 */
    /* Lag 6 gap */
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
    /* Lag 23 gap */
    { 0,24},		/* 24 */
    {43,43}};		/* alternate lag-0  */

/* Integration period variables */
  int scnsc=120;
  int scnus=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  /* Variables for possibly different normal vs snd nrang/rsep */
  int def_intt_sc=0;
  int def_intt_us=0;
  int def_nrang=0;
  int def_rsep=0;
  int def_txpl=0;

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int default_clrskip_secs=30;
  int startup=1;

  /* XCF processing variables */
  int cnt=0;

  /* ---------------- Variables for sounding --------------- */
  char snd_filename[100];
  FILE *snd_dat;
  /* If the file $SITE_CFG/site.[rad]/sounder_[rad].dat exists, the next two parameters are read from it */
  /* the file contains one integer value per line */
  int snd_freqs_tot=8;
  int snd_freqs[MAX_SND_FREQS]= {11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0 };
  int *snd_bms;
  int snd_bmse[]={0,2,4,6,8,10,12,14,16,18};   /* beam sequences for 24-beam MSI radars using only */
  int snd_bmsw[]={22,20,18,16,14,12,10,8,6,4}; /*  the 20 most meridional beams */
  int snd_freq_cnt=0, snd_bm_cnt=0;
  int snd_bms_tot=10, odd_beams=0;
  int snd_freq;
  int snd_frqrng=100;
  int snd_nrang=75;
  int snd_rsep=45;
  int snd_txpl=300;
  int snd_sc=-1;
  int snd_intt_sc=1;
  int snd_intt_us=500000;
  float snd_time, snd_intt, time_needed=1.25;
  /* ------------------------------------------------------- */

  /* create commandline argument structs */
  /* First lets define a help argument */
  struct arg_lit  *al_help       = arg_lit0(NULL, "help", "Prints help infomation and then exits");
  /* Now lets define the keyword arguments */
  struct arg_lit  *al_debug      = arg_lit0(NULL, "debug","Enable debugging messages");
  struct arg_lit  *al_test       = arg_lit0(NULL, "test","Test-only, report parameter settings and exit without connecting to ros server");
  struct arg_lit  *al_discretion = arg_lit0(NULL, "di","Flag this is discretionary time operation"); 
  struct arg_lit  *al_fast       = arg_lit0(NULL, "fast","Flag this as fast 1-minute scan duration"); 
  struct arg_lit  *al_clrscan    = arg_lit0(NULL, "clrscan","Force clear frequency search at start of scan"); 
  /* Now lets define the integer valued arguments */
  struct arg_int  *ai_tau        = arg_int0(NULL, "tau", NULL,"Lag spacing in usecs"); /*OptionAdd( &opt, "tau", 'i', &mpinc);*/
  struct arg_int  *ai_nrang      = arg_int0(NULL, "nrang", NULL,"Number of range cells"); /*OptionAdd(&opt,"nrang",'i',&nrang);*/
  struct arg_int  *ai_frang      = arg_int0(NULL, "frang", NULL,"Distance to first range cell in km"); /*OptionAdd(&opt,"frang",'i',&frang); */
  struct arg_int  *ai_rsep       = arg_int0(NULL, "rsep", NULL,"Range cell extent in km"); /*OptionAdd(&opt,"rsep",'i',&rsep); */
  struct arg_int  *ai_dt         = arg_int0(NULL, "dt", NULL,"UTC Hour indicating start of day time operation"); /*OptionAdd( &opt, "dt", 'i', &day); */
  struct arg_int  *ai_nt         = arg_int0(NULL, "nt", NULL,"UTC Hour indicating start of night time operation"); /*OptionAdd( &opt, "nt", 'i', &night); */
  struct arg_int  *ai_df         = arg_int0(NULL, "df", NULL,"Day time transmit frequency in kHz"); /*OptionAdd( &opt, "df", 'i', &dfrq); */
  struct arg_int  *ai_nf         = arg_int0(NULL, "nf", NULL,"Night time transmit frequency in KHz"); /*OptionAdd( &opt, "nf", 'i', &nfrq); */
  struct arg_int  *ai_fixfrq     = arg_int0(NULL, "fixfrq", NULL,"Fixes the transmit frequency of the radar to one frequency, in KHz"); /*OptionAdd( &opt, "fixfrq", 'i', &fixfrq); */
  struct arg_int  *ai_xcf        = arg_int0(NULL, "xcf", NULL,"Enable xcf, --xcf 1: for all sequences --xcf 2: for every other sequence, etc..."); /*OptionAdd( &opt, "xcf", 'i', &xcnt); */
  struct arg_int  *ai_ep         = arg_int0(NULL, "ep", NULL,"Local TCP port for errorlog process"); /*OptionAdd(&opt,"ep",'i',&errlog.port); */
  struct arg_int  *ai_sp         = arg_int0(NULL, "sp", NULL,"Local TCP port for radarshall process"); /*OptionAdd(&opt,"sp",'i',&shell.port); */
  struct arg_int  *ai_bp         = arg_int0(NULL, "bp", NULL,"Local TCP port for start of support task proccesses"); /*OptionAdd(&opt,"bp",'i',&baseport); */
  struct arg_int  *ai_sb         = arg_int0(NULL, "sb", NULL,"Limits the minimum beam to the given value."); /*OptionAdd(&opt,"sb",'i',&sbm); */
  struct arg_int  *ai_eb         = arg_int0(NULL, "eb", NULL,"Limits the maximum beam number to the given value."); /*OptionAdd(&opt,"eb",'i',&ebm); */
  struct arg_int  *ai_cnum       = arg_int0("c",  "cnum", NULL,"Radar Channel number, minimum value 1"); /*OptionAdd(&opt,"c",'i',&cnum); */
  struct arg_int  *ai_clrskip    = arg_int0(NULL, "clrskip",NULL,"Minimum number of seconds to skip between clear frequency search"); /*OptionAdd(&opt, "clrskip", 'i', &clrskip_secs); */
  struct arg_int  *ai_cpid       = arg_int0(NULL, "cpid",NULL,"Select control program ID number"); /*OptionAdd(&opt, "cpid", 'i', &cpid); */

  /* Now lets define the string valued arguments */
  struct arg_str  *as_ros        = arg_str0(NULL, "ros", NULL,        "IP address of ROS server process"); /* OptionAdd(&opt,"ros",'t',&roshost); */
  struct arg_str  *as_ststr      = arg_str0(NULL, "stid", NULL,       "The station ID string. For example, use aze for azores east."); /* OptionAdd(&opt,"stid",'t',&ststr); */
  struct arg_str  *as_libstr     = arg_str0(NULL, "lib", NULL,        "The site library string. For example, use ros for for common libsite.ros"); 
  struct arg_str  *as_verstr     = arg_str0(NULL, "version", NULL,    "The site library version string. Defaults to: \"1\" "); 

  /* required end argument */
  struct arg_end  *ae_argend     = arg_end(ARG_MAXERRORS);

  /* create list of all arguement structs */
  void* argtable[] = {al_help,al_debug,al_test,al_discretion, al_fast, \
                      ai_tau, ai_nrang, ai_frang, ai_rsep, ai_dt, ai_nt, ai_df, ai_nf, ai_fixfrq, ai_xcf, ai_ep, ai_sp, ai_bp, ai_sb, ai_eb, ai_cnum, \
                      as_ros, as_ststr, as_libstr,as_verstr, ai_clrskip,al_clrscan,ai_cpid, ae_argend};

/* END of variable defines */

/* Set default values of globally defined variables here*/
  cp     = 9000;    /*unused Alaska cpid, will be reset below  */
  intsc  = 7;
  intus  = 0;
  mppul  = 8;
  mplgs  = 23;
  mpinc  = 1500;
  dmpinc = 1500;
  nrang  = 75;
  rsep   = 45;
  txpl   = 300;
  nbaud  = 1;

/* Set default values for all the cmdline options */
  al_discretion->count = 0;
  al_fast->count = 0;
  al_clrscan->count = 0;
  al_debug->count = 0;
  ai_bp->ival[0] = 44100;
  ai_fixfrq->ival[0] = -1;
  ai_tau->ival[0] = mpinc;
  ai_nrang->ival[0] = nrang;
  ai_frang->ival[0] = frang;
  ai_rsep->ival[0] = rsep;
  ai_dt->ival[0] = day;
  ai_nt->ival[0] = night;
  ai_df->ival[0] = dfrq;
  ai_nf->ival[0] = nfrq;
  ai_xcf->ival[0] = xcnt;
  ai_ep->ival[0] = errlog.port;
  ai_sp->ival[0] = shell.port;
  ai_sb->ival[0] = sbm;
  ai_eb->ival[0] = ebm;
  ai_cnum->ival[0] = cnum;
  ai_clrskip->ival[0] = -1;
  ai_cpid->ival[0] = 0;

 /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */
  nerrors = arg_parse(argc,argv,argtable);

  if (nerrors > 0) {
    arg_print_errors(stdout,ae_argend,"uafsound");
    return -1;
  }
  
  if (argc == 1) {
    printf("No arguements found, try running %s with --help for more information.\n", progid);
    return -1;
  }

  /* print help */
  if(al_help->count > 0) {
    printf("UAFSOUND: An attempt to get frequency sounding modes working for UAF USRP-style radars \n\n");
    printf(" Usage: %s", progid);
    arg_print_syntax(stdout,argtable,"\n");
    arg_print_glossary(stdout,argtable,"  %-25s %s\n");
    arg_freetable(argtable, sizeof(argtable)/sizeof(argtable[0]));
    return 0;
  }

  /* Set debug flag from command line arguement */
  if (al_debug->count) {
    debug = al_debug->count;
  }

  /* Load roshost argument here */
  if(strlen(as_ros->sval[0])) {
    roshost = malloc((strlen(as_ros->sval[0]) + 1) * sizeof(char));
    strcpy(roshost, as_ros->sval[0]);
  } else {
    roshost = getenv("ROSHOST");
    if (roshost == NULL) roshost = droshost;
  }

  /* Load station string argument here */
  if(strlen(as_ststr->sval[0])) {
    ststr = malloc((strlen(as_ststr->sval[0]) + 1) * sizeof(char));
    strcpy(ststr, as_ststr->sval[0]);
  } else {
    ststr = getenv("STSTR");
  }

  /* Load site library argument here */
  if(strlen(as_libstr->sval[0])) {
    libstr = malloc((strlen(as_libstr->sval[0]) + 1) * sizeof(char));
    strcpy(libstr, as_libstr->sval[0]);
  } else {
    libstr = getenv("LIBSTR");
    if (libstr == NULL) libstr=ststr;
  }
  if(strlen(as_verstr->sval[0])) {
    verstr = malloc((strlen(as_verstr->sval[0]) + 1) * sizeof(char));
    strcpy(verstr, as_verstr->sval[0]);
  } else {
    verstr = NULL;
  }

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    snd_bms = snd_bmse;
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0) || (strcmp(ststr,"bks") == 0)) {
    snd_bms = snd_bmsw;
  } else if (strcmp(ststr,"fhw") == 0) {
    snd_bms = snd_bmsw;
    for (i=0; i<snd_bms_tot; i++)
      snd_bms[i] -= 2;
  } else {
    snd_bms = snd_bmse;
    snd_bms_tot = 8;
    snd_intt_sc = 2;
    snd_intt_us = 0;
  }

  snd_intt = snd_intt_sc + snd_intt_us*1e-6;

  /* load the sounder frequencies from file in site directory if present */
  path = getenv("SITE_CFG");
  if (path == NULL) {
    fprintf(stderr,"Environment variable 'SITE_CFG' not defined.\n");
  }

  sprintf(snd_filename,"%s/site.%s/sounder_%s.dat", path, ststr, ststr);
  fprintf(stderr,"Checking Sounder File: %s\n",snd_filename);
  snd_dat = fopen(snd_filename, "r");
  if (snd_dat != NULL) {
    fscanf(snd_dat, "%d", &snd_freqs_tot);
    if (snd_freqs_tot > 12) snd_freqs_tot = 12;
    for (snd_freq_cnt=0; snd_freq_cnt < snd_freqs_tot; snd_freq_cnt++)
      fscanf(snd_dat, "%d", &snd_freqs[snd_freq_cnt]);
    snd_freq_cnt = 0;
    fclose(snd_dat);
    fprintf(stderr,"Sounder File: %s read\n",snd_filename);
  } else {
    fprintf(stderr,"Sounder File: %s not found\n",snd_filename);
  }


  printf("Requested :: ststr: %s libstr: %s verstr: %s\n",ststr,libstr,verstr);
/* This loads Radar Site information from hdw.dat files */
  OpsStart(ststr);

/* This loads Site library via dlopen and maps: site library specific functions into Site name space */
  status=SiteBuild(libstr,verstr); /* second argument is version string */
  if (status==-1) {
    fprintf(stderr,"Could not load requested site library\n");
    exit(1);
  }

  /* cnum is needed to know the config file name */
  if (ai_cnum->count) cnum = ai_cnum->ival[0];


/* Run SiteStart library function to load Site specific default values for global variables
 * This should be run before all options are parsed and before any task sockets are opened
 * arguments: host ip address, 3-letter station string
*/
  
  status=SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"SiteStart failure\n");
    exit(1);
  }

/* load any provided argument values overriding default values provided by SiteStart */ 
  if (ai_xcf->count)  xcnt = ai_xcf->ival[0];
  if (ai_tau->count) mpinc = ai_tau->ival[0];
  if (ai_nrang->count) nrang = ai_nrang->ival[0];
  if (ai_frang->count) frang = ai_frang->ival[0];
  if (ai_rsep->count) rsep = ai_rsep->ival[0];
  if (ai_dt->count) day = ai_dt->ival[0];
  if (ai_nt->count) night = ai_nt->ival[0];
  if (ai_df->count) dfrq = ai_df->ival[0];
  if (ai_nf->count) nfrq = ai_nf->ival[0];
  if (ai_ep->count) errlog.port = ai_ep->ival[0];
  if (ai_sp->count) shell.port = ai_sp->ival[0];
  if (ai_sb->count) sbm = ai_sb->ival[0];
  if (ai_eb->count) ebm = ai_eb->ival[0];
  if (ai_bp->count) baseport=ai_bp->ival[0];

 /* ========= SET PARAMETER TO EMULATE OTHER CONTROL PROGRAMS ============= */

  /* NORMAL beam order */
  fprintf(stderr, "Initializing normal beam pattern...\n");
  nBeams_per_scan = abs(ebm-sbm)+1;
  current_beam = sbm;

  /* defaults for normalsound, adjusted   */  
  cp=155;
  scnsc = 120;
  scnus = 0;
  snd_sc = 20;

  sync_scan = 0; 
 
  /* FAST option */  
  if (al_fast->count) {     /* If fast option selected use 1 minute scan boundaries */
    cp    = 157;
    scnsc = 60;
    scnus = 0;
    snd_sc = 12;
    sprintf(modestr," (fast)");
    strncat(progname,modestr,strlen(modestr)+1);
  } 

  for (iBeam =0; iBeam < nBeams_per_scan; iBeam++){
     scan_beam_number_list[iBeam] = current_beam;
     current_beam += backward ? -1:1;
  }

  snd_nBeams_per_scan = (snd_sc-time_needed)/(snd_intt_sc + snd_intt_us*1e-6);


  /* Print out details of beams */ 
  fprintf(stderr, "Sequence details: \n");
  for (iBeam =0; iBeam < nBeams_per_scan; iBeam++){
    fprintf(stderr, "  sequence %2d: beam: %2d, \n",iBeam, scan_beam_number_list[iBeam] );
  }




/* Open Connection to errorlog */  
  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {    
    fprintf(stderr,"Error connecting to error log.\n Host: %s  Port: %d\n",errlog.host,errlog.port);
  }
/* Open Connection to radar shell */  
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {    
    fprintf(stderr,"Error connecting to shell.\n");
  }

/* Open Connection to helper utilities like fitacfwrite*/  
  for (n=0;n<tnum;n++) task[n].port+=baseport;

/* Prep command string for tasks */ 
  strncpy(combf,progid,80);   
  OpsSetupCommand(argc,argv);
  OpsLogStart(errlog.sock,progname,argc,argv);  
  OpsSetupTask(tnum,task,errlog.sock,progname);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);     
  }


  /* Initialize timing variables */
  elapsed_secs=0;
  gettimeofday(&t1,NULL);
  gettimeofday(&t0,NULL);
  

  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-snd_sc)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  def_intt_sc = total_integration_usecs/1E6;
  def_intt_us = total_integration_usecs - (def_intt_sc*1E6);

  def_nrang = nrang;

  intsc = def_intt_sc;
  intus = def_intt_us;

  /* Configure phasecoded operation if nbaud > 1 */ 
  switch(nbaud) {
    case 1:
      bcode=bcode1;
    case 2:
      bcode=bcode2;
      break;
    case 3:
      bcode=bcode3;
      break;
    case 4:
      bcode=bcode4;
      break;
    case 5:
      bcode=bcode5;
      break;
    case 7:
      bcode=bcode7;
      break;
    case 11:
      bcode=bcode11;
      break;
    case 13:
      bcode=bcode13;
      break;
    default:
      ErrLog(errlog.sock,progname,"Error: Unsupported nbaud requested, exiting");
      SiteExit(1);
  }
  pcode=(int *)malloc((size_t)sizeof(int)*mppul*nbaud);
  for(i=0;i<mppul;i++){
    for(n=0;n<nbaud;n++){
      pcode[i*nbaud+n]=bcode[n];
    }
  }

  /* Set special cpid if provided on commandline */
  if(ai_cpid->count > 0) 
     cp=ai_cpid->ival[0];

  /* Set cp to negative value indication discretionary period */
  if (al_discretion->count) 
     cp= -cp;


  /* Calculate tx pulse length setting from range separation */
  txpl = (nbaud*rsep*20)/3;

  def_rsep = rsep;
  def_txpl = txpl;

  /* Attempt to adjust mpinc to be a multiple of 10 and a muliple of txpl */
  if ((mpinc % txpl) || (mpinc % 10))  {
    ErrLog(errlog.sock,progname,"Error: mpinc not multiple of txpl... checking to see if it can be adjusted");
    sprintf(logtxt,"Initial: mpinc: %d txpl: %d  nbaud: %d  rsep: %d", mpinc , txpl, nbaud, rsep);
    ErrLog(errlog.sock,progname,logtxt);
    if((txpl % 10)==0) {

      ErrLog(errlog.sock,progname, "Attempting to adjust mpinc to correct");
      if (mpinc < txpl) mpinc=txpl;
      int minus_remain=mpinc % txpl;
      int plus_remain=txpl -(mpinc % txpl);
      if (plus_remain > minus_remain)
        mpinc = mpinc - minus_remain;
      else
         mpinc = mpinc + plus_remain;
      if (mpinc==0) mpinc = mpinc + plus_remain;

    }
  }
  /* Check mpinc and if still invalid, exit with error */
  if ((mpinc % txpl) || (mpinc % 10) || (mpinc==0))  {
     sprintf(logtxt,"Error: mpinc: %d txpl: %d  nbaud: %d  rsep: %d", mpinc , txpl, nbaud, rsep);
     ErrLog(errlog.sock,progname,logtxt);
     SiteExit(0);
  }

  if(al_test->count > 0) {
        
    fprintf(stdout,"Control Program Argument Parameters::\n");
    fprintf(stdout,"  xcf arg:: count: %d value: %d xcnt: %d\n",ai_xcf->count,ai_xcf->ival[0],xcnt);
    fprintf(stdout,"  clrskip arg:: count: %d value: %d\n",ai_clrskip->count,ai_clrskip->ival[0]);
    fprintf(stdout,"  cpid: %d progname: \'%s\'\n",cp,progname);
    fprintf(stdout,"Scan Sequence Parameters::\n");
    fprintf(stdout,"  txpl: %d mpinc: %d nbaud: %d rsep: %d\n",txpl,mpinc,nbaud,rsep);
    fprintf(stdout,"  intsc: %d intus: %d scnsc: %d scnus: %d\n",intsc,intus,scnsc,scnus);
    fprintf(stdout,"  sbm: %d ebm: %d  nBeams_per_scan: %d\n",sbm,ebm,nBeams_per_scan);
    
    /* TODO: ADD PARAMETER CHECKING, SEE IF PCODE IS SANE AND WHATNOT */
   if(nbaud >= 1) {
        /* create tsgprm struct and pass to TSGMake, check if TSGMake makes something valid */
        /* checking with SiteTimeSeq(ptab); would be easier, but that talks to hardware..*/
        /* the job of aggregating a tsgprm from global variables should probably be a function in maketsg.c */
        int flag = 0;

        if (tsgprm.pat !=NULL) free(tsgprm.pat);
        if (tsgbuf !=NULL) TSGFree(tsgbuf);

        memset(&tsgprm,0,sizeof(struct TSGprm));   
        tsgprm.nrang   = nrang;
        tsgprm.frang   = frang;
        tsgprm.rsep    = rsep; 
        tsgprm.smsep   = smsep;
        tsgprm.txpl    = txpl;
        tsgprm.mppul   = mppul;
        tsgprm.mpinc   = mpinc;
        tsgprm.mlag    = 0;
        tsgprm.nbaud   = nbaud;
        tsgprm.stdelay = 18 + 2;
        tsgprm.gort    = 1;
        tsgprm.rtoxmin = 0;

        tsgprm.pat  = malloc(sizeof(int)*mppul);
        tsgprm.code = ptab;

        for (i=0;i<tsgprm.mppul;i++) 
           tsgprm.pat[i]=ptab[i];

        tsgbuf=TSGMake(&tsgprm,&flag);
        fprintf(stdout,"Sequence Parameters::\n");
        fprintf(stdout,"  lagfr: %d smsep: %d  txpl: %d\n",tsgprm.lagfr,tsgprm.smsep,tsgprm.txpl);
    
        if(tsgprm.smsep == 0 || tsgprm.lagfr == 0) {
            fprintf(stdout,"Sequence Parameters::\n");
            fprintf(stdout,"  lagfr: %d smsep: %d  txpl: %d\n",tsgprm.lagfr,tsgprm.smsep,tsgprm.txpl);
            fprintf(stdout,"WARNING: lagfr or smsep is zero, invalid timing sequence genrated from given baud/rsep/nrang/mpinc will confuse TSGMake and FitACF into segfaulting");
        }

        else {
            fprintf(stdout,"The phase coded timing sequence looks good\n");
        }
    } else {
        fprintf(stdout,"WARNING: nbaud needs to be  > 0\n");
    }

    if (nerrors > 0) {
        fprintf(stdout,"Errors found in commandline arguements: \n");
        arg_print_errors(stdout,ae_argend,"uafscan");
    }
    OpsFitACFStart();
 
    fprintf(stdout,"Test option enabled, exiting\n");
    return 0;
  }
  /* SiteSetupRadar, establish connection to usrp_server and do initial setup of memory buffers for raw samples */
  printf("Running SiteSetupRadar Station ID: %s  %d\n",ststr,stid);
  status=SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error connection to usrp_server.");
    exit (1);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();


  if ((def_nrang == snd_nrang) && (def_rsep == snd_rsep)) {
    printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
    tsgid=SiteTimeSeq(ptab);
  }

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {
    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
      printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
      tsgid=SiteTimeSeq(ptab);
    }

    /* reset clearfreq paramaters, in case daytime changed */
    for (iBeam =0; iBeam < nBeams_per_scan; iBeam++){
      scan_clrfreq_fstart_list[iBeam] = (int32_t) (OpsDayNight() == 1 ? dfrq : nfrq); 
      scan_clrfreq_bandwidth_list[iBeam] = frqrng;
      current_beam += backward ? -1:1;
    }

    /* Set iBeam for scan loop  */ 
    iBeam = OpsFindSkip(scnsc,scnus);

    /* send scan data to usrp_sever */
    if (SiteStartScan(nBeams_per_scan, scan_beam_number_list, scan_clrfreq_fstart_list, scan_clrfreq_bandwidth_list, ai_fixfrq->ival[0], sync_scan, scan_times, scnsc, scnus, intsc, intus, iBeam) !=0){
         ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");  
         sleep(1);
         continue;
    }
   /* OLD
     if (SiteStartScan(nBeams_per_scan, scan_beam_number_list, scan_clrfreq_fstart_list, scan_clrfreq_bandwidth_list, ai_fixfrq->ival[0], sync_scan, scan_times, scnsc, scnus, intsc, intus, iBeam) !=0) continue;
*/

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);     
      }
    }

    scan=1;
    ErrLog(errlog.sock,progname,"Starting scan.");
    if(al_clrscan->count) startup=1;
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;




    /* Scan loop for sequences/beams  */
    do {  
      bmnum = scan_beam_number_list[iBeam];

      TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);


      /* THIS IS NOW IS USRP_SERVER */
      /* SYNC periods/beams */ /*
      if (sync_scan) {
          time_now     = ( (mt*60 + sc)*1000 + us/1000 ) % (scnsc*1000 + scnus/1000);
          time_to_wait = scan_times[iBeam] - time_now;
          if (time_to_wait > 0){
             printf("Sync periods: Waiting for %d ms ...", time_to_wait);
             usleep(time_to_wait);
             printf("done.\n");
          } else {
             printf("Sync periods: Not waiting, sinc periods is %d ms too late.", time_to_wait);
          }
      }  */

      /* TODO: JDS: You can not make any day night changes that impact TR gate timing at dual site locations. Care must be taken with day night operation*/ 
      stfrq = scan_clrfreq_fstart_list[iBeam];
      if(ai_fixfrq->ival[0]>0) {
        stfrq=ai_fixfrq->ival[0];
        tfreq=ai_fixfrq->ival[0];
        noise=0; 
      }

      ErrLog(errlog.sock,progname,"Starting Integration.");
      sprintf(logtxt,"Int parameters:: rsep: %d mpinc: %d sbm: %d ebm: %d nrang: %d nbaud: %d clrskip_secs: %d clrscan: %d cpid: %d",
              rsep,mpinc,sbm,ebm,nrang,nbaud,ai_clrskip->ival[0],al_clrscan->count,cp);
      ErrLog(errlog.sock,progname,logtxt);

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum, intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
            
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);
      gettimeofday(&t1,NULL);
      elapsed_secs=t1.tv_sec-t0.tv_sec;
      if (elapsed_secs<0) elapsed_secs=0;
      if ((elapsed_secs >= ai_clrskip->ival[0]) || (startup==1)) {
          startup = 0;
          ErrLog(errlog.sock,progname,"Doing clear frequency search.");  
          sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
          ErrLog(errlog.sock,progname, logtxt);
  
          if (ai_fixfrq->ival[0]<=0) {
              tfreq=SiteFCLR(stfrq,stfrq+frqrng);
          }
          t0.tv_sec  = t1.tv_sec;
          t0.tv_usec = t1.tv_usec;
      }
      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);
    
      nave=SiteIntegrate(lags);   
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog.sock,progname,logtxt); 
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      /* Processing and sending data */ 
      OpsBuildPrm(prm,ptab,lags);    
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit);
      
      msg.num   = 0;
      msg.tsize = 0;

      tmpbuf = RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg, tmpsze, tmpbuf, PRM_TYPE, 0); 

      tmpbuf=IQFlatten(iq, prm->nave, &tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg, sizeof(unsigned int)*2*iq->tbadtr, (unsigned char *) badtr, BADTR_TYPE, 0);
      RMsgSndAdd(&msg, strlen(sharedmemory)+1, (unsigned char *) sharedmemory, IQS_TYPE, 0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0); 
 
      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0); 

      RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *) progname, NME_TYPE,0);   
     
     
      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg); 

      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE)  free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]); 
      }          

      scan = 0;

      iBeam++;
      if (iBeam >= nBeams_per_scan) break;

    } while (1);


    /* In here comes the sounder code */
    /* set the "sounder mode" scan variable */
    scan = -2;

    /* set the xcf variable to do cross-correlations (AOA) */
    if (xcnt > 1) xcf = 1;

    /* set the sounding mode integration time, number of ranges,
     * and range separation */
    intsc = snd_intt_sc;
    intus = snd_intt_us;
    nrang = snd_nrang;
    rsep = snd_rsep;
    txpl = snd_txpl;

    for (snd_iBeam=0; snd_iBeam < snd_nBeams_per_scan; snd_iBeam++) {
      snd_beam_number_list[snd_iBeam] = snd_bms[snd_bm_cnt] + odd_beams;
      snd_clrfreq_fstart_list[snd_iBeam] = snd_freqs[snd_freq_cnt];
      snd_clrfreq_bandwidth_list[snd_iBeam] = snd_frqrng;
      snd_bc[snd_iBeam] = snd_bm_cnt;
      snd_fc[snd_iBeam] = snd_freq_cnt;

      snd_freq_cnt++;
      if (snd_freq_cnt >= snd_freqs_tot) {
        /* reset the freq counter and increment the beam counter */
        snd_freq_cnt = 0;
        snd_bm_cnt++;
        if (snd_bm_cnt >= snd_bms_tot) {
          snd_bm_cnt = 0;
          odd_beams = !odd_beams;
        }
      }
    }

    /* Print out details of sounding beams */ 
    fprintf(stderr, "Sounding sequence details: \n");
    for (snd_iBeam =0; snd_iBeam < snd_nBeams_per_scan; snd_iBeam++){
      fprintf(stderr, "  sequence %2d: beam: %2d freq: %5d, \n",snd_iBeam,
              snd_beam_number_list[snd_iBeam], snd_clrfreq_fstart_list[snd_iBeam] );
    }

    snd_iBeam = 0;

    /* send sounding scan data to usrp_sever */
    if (SiteStartScan(snd_nBeams_per_scan, snd_beam_number_list, snd_clrfreq_fstart_list, snd_clrfreq_bandwidth_list, 0, sync_scan, scan_times, snd_scnsc, snd_scnus, snd_intt_sc, snd_intt_us, snd_iBeam) !=0){
         ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");  
         sleep(1);
         continue;
    }

    /* make a new timing sequence for the sounding */
    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
      printf("Preparing SND SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
      tsgid = SiteTimeSeq(ptab);
    }

    /* we have time until the end of the minute to do sounding */
    /* minus a safety factor given in time_needed */
    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    snd_time = 60.0 - (sc + us*1e-6);

    while (snd_time-snd_intt > time_needed) {

      /* set the beam */
      bmnum = snd_beam_number_list[snd_iBeam];

      /* snd_freq will be an array of frequencies to step through */
      snd_freq = snd_clrfreq_fstart_list[snd_iBeam];

      /* the scanning code is here */
      sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
      ErrLog(errlog.sock,progname,"Setting SND beam.");
      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock, progname, "Doing SND clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", snd_freq, snd_frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq = SiteFCLR(snd_freq, snd_freq + snd_frqrng);

      sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock, progname, logtxt);

      nave = SiteIntegrate(seq->lags);
      if (nave < 0) {
        sprintf(logtxt, "SND integration error: %d", nave);
        ErrLog(errlog.sock,progname, logtxt);
        continue;
      }
      sprintf(logtxt,"Number of SND sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,ptab,lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit);

      ErrLog(errlog.sock, progname, "Sending SND messages.");
      msg.num = 0;
      msg.tsize = 0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      RMsgSndSend(task[RT_TASK].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      sprintf(logtxt, "SBC: %d  SFC: %d", snd_bc[snd_iBeam], snd_fc[snd_iBeam]);
      ErrLog(errlog.sock, progname, logtxt);

      /* set the scan variable for the sounding mode data file only */
      if ((bmnum == snd_bms[0]) && (snd_freq == snd_freqs[0])) {
        prm->scan = 1;
      } else {
        prm->scan = 0;
      }

      OpsBuildSnd(prm,fit);

      /* save the sounding mode data */
      write_snd_record(progname, prm, fit);

      ErrLog(errlog.sock, progname, "Polling SND for exit.\n");

      snd_iBeam++;
      if (snd_iBeam >= snd_nBeams_per_scan) break;

      /* see if we have enough time for another go round */
      TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);
      snd_time = 60.0 - (sc + us*1e-6);
    }

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    /* make sure we didn't miss any beams/frequencies */
    if (snd_iBeam < snd_nBeams_per_scan-1) {
      snd_bm_cnt = snd_bc[snd_iBeam-1];
      snd_freq_cnt = snd_fc[snd_iBeam-1];
    }

    intsc = def_intt_sc;
    intus = def_intt_us;
    nrang = def_nrang;
    rsep = def_rsep;
    txpl = def_txpl;

    SiteEndScan(scnsc,scnus,5000);
  } while (1);
  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);
  
  /* free argtable and space allocated for arguements */
  arg_freetable(argtable, sizeof(argtable)/sizeof(argtable[0]));
  free(ststr);
  free(roshost);
  
  ErrLog(errlog.sock,progname,"Ending program.");


  SiteExit(0);

  return 0;   
} 


/********************** function write_snd_record() ************************/
/* changed the output to dmap format */

void write_snd_record(char *progname, struct RadarParm *prm, struct FitData *fit) {

  char data_path[100], data_filename[50], filename[80];

  char *snd_dir;
  FILE *out;

  char logtxt[1024]="";
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding file:%s",filename);
    ErrLog(errlog.sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = SndFwrite(out, prm, fit);
  if (status == -1) {
    ErrLog(errlog.sock,progname,"Error writing sounding record.");
  } else {
    ErrLog(errlog.sock,progname,"Sounding record successfully written.");
  }

  fclose(out);
}