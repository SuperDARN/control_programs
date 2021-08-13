/* interleavesound.c
   ================= */

/* 
 $Log: interleavesound.c,v $
 Revision 1.0  2020/09/09  egthomas
 Initial revision from kat_fsound

*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "cnv_time.h"
#include "types.h"
#include "option.h"
#include "limit.h"
#include "message.h"
#include "a_d_drive.h"
#include "dio.h"
#include "tsg.h"

#include "rawdata.h"
#include "fitdata.h"
#include "hardware.h"
#include "radar_name.h"
#include "fitacf.h"
#include "raw_read.h"

#include "task_msg.h"
#include "freq_lib.h"

#include "radar.h"

#include "fclr_set.h"
#include "fclr.h"

#include "read_clock.h"
#include "log_error.h"
#include "task_write.h"
#include "build_raw.h"
#include "get_status.h"
#include "read_data.h"
#include "test_reopen.h"
#include "tmseq.h"

#include "radar_shell.h"

#include "global.h"
#include "setup.h"

#include "shell.h"

#include "calc_skip.h"
#include "wait_boundary.h"
#include "site.h"

#include "drivers.h"
#include "tasks.h"

#define CP_ID 197

#define TEST_DATA "/data/test/test.dat"

/* name of the driver tasks */
char *a_d_name={A_D_NAME};
char *dio_name={DIO_NAME};

char *tasklist[]= { TASK_NAMES, 0};

char errlog_name[40]={ERRLOG_NAME};
char scheduler_name[40]={SCHEDULER_NAME};
char program_name[40]={"interleavesound"};
char program_id[80]={"$Id: interleavesound.c,v 1.0 2020/09/09 egthomas Exp $"};
char command[1024];

char logtxt[256];

struct rawdata raw;
struct fitdata fit;
struct task_block msg;
struct task_id **tsk=NULL;
struct task_id *etsk=NULL;

struct radar_hardware *hdw=NULL;
struct fit_block *fblk=NULL;

#define MAX_SND_FREQS 12
int recnum=0;

void main(int argc,char *argv[]) {

   /* define pulse sequence and lag table */

  int day_mpinc,night_mpinc,mpinc;
  int mppul,mplgs;
  int tsg_id;

  int ptab[8] = {0,14,22,24,27,31,42,43};
  int lags[2*24] ={ 
    0,42,22,24,27,22,24,14,  22,14,31,31,14, 0,27,27,14,  24,24,22,22, 0, 0, 
    0,43,24,27,31,27,31,22,  31,24,42,43,27,14,42,43,31,  42,43,42,43,22,24};

  /* define other variables */
  int i,c;
  int status;
  int debug=0;
  FILE *optfp;
  char *test_file=NULL;
  struct rawfp *rfp=NULL;
  int exit_poll=0;
  int argnum=0;
  int tnum;
  int sbm,ebm;
  int skip_beam;
  char *errlog_ptr=errlog_name; /* the default names */
  char *scheduler_ptr=scheduler_name;

  /* -------- Beam sequence for interleavedscan -------- */
  /* For a 16-beam radar */
  int num_scans = 16;
  int forward_beams[16] = { 0,4,8,12, 2,6,10,14,  1,5,9,13, 3,7,11,15};
  int backward_beams[16]= {15,11,7,3,  13,9,5,1, 14,10,6,2,  12,8,4,0};
  int bmseqnum = 0;
  /* --------------------------------------------------- */

  /* -------------- Variables for sounding ------------- */
  char snd_filename[100];
  FILE *snd_dat;
  int snd_freqs_tot=8;
  int snd_freqs[MAX_SND_FREQS] = {11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0};
  int snd_bms[]={0,2,4,6,8,10,12,14};
  int snd_freq_cnt=0, snd_bm_cnt=0;
  int snd_bms_tot=8, odd_beams=0;
  int snd_freq;
  int snd_frqrng=100;
  int fast_intt=3;
  int snd_intt=2;
  int scan_period=60;
  float snd_time, time_needed=1.25;

  /* load the sounder frequencies from file if present */
  sprintf(snd_filename, "%s/sounder_%s.dat", getenv("SD_HDWPATH"), getenv("SD_RADARCODE"));
  fprintf(stderr, "Checking Sounder File: %s\n", snd_filename);
  snd_dat = fopen(snd_filename, "r");
  if (snd_dat != NULL) {
    fscanf(snd_dat, "%d", &snd_freqs_tot);
    if (snd_freqs_tot > 12) snd_freqs_tot = 12;
    for (snd_freq_cnt=0; snd_freq_cnt < snd_freqs_tot; snd_freq_cnt++)
      fscanf(snd_dat, "%d", &snd_freqs[snd_freq_cnt]);
    snd_freq_cnt = 0;
    fclose(snd_dat);
    fprintf(stderr,"Sounder File: %s read\n", snd_filename);
  } else {
    fprintf(stderr,"Sounder File: %s not found\n", snd_filename);
  }
  /* --------------------------------------------------- */

  pid_t sid;

  /* initialize the start frequency */
  read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);
  if ( day_time()==1 )
    start_freq=day_start_freq;
  else
    start_freq=night_start_freq;

  /* get station identifier */
  st_id=get_st_id();

  /* declare operating parameters */
  cp=CP_ID;
  frang=day_frang;
  intt=fast_intt;

  mppul=8;
  mplgs=23;

  day_mpinc=1500;
  night_mpinc=1500;

  /* call site library */
  start_program();

  /* setup base shell variables */
  init_shell();

  /* set up the command line options */
  option_add(&cmdlne,"di",'x',&discretion);
  option_add(&cmdlne,"db",'x',&debug);
  option_add(&cmdlne,"e",'t',&errlog_ptr);
  option_add(&cmdlne,"sc",'t',&scheduler_ptr);
  option_add(&cmdlne,"tf",'t',&test_file);
  option_add(&cmdlne,"frang",'i',&frang);
  option_add(&cmdlne,"rsep",'i',&rsep);

  option_add(&cmdlne,"dt",'i',&day_start_hr);
  option_add(&cmdlne,"nt",'i',&night_start_hr);
  option_add(&cmdlne,"df",'i',&day_start_freq);
  option_add(&cmdlne,"nf",'i',&night_start_freq);
  option_add(&cmdlne,"dr",'i',&day_frang);
  option_add(&cmdlne,"nr",'i',&night_frang);
  option_add(&cmdlne,"dm",'i',&day_mpinc);
  option_add(&cmdlne,"nm",'i',&night_mpinc);
  option_add(&cmdlne,"xcf",'i',&xcount);

  option_add(&cmdlne,"freq_range",'i',&freq_range);
  option_add(&cmdlne,"snd_frqrng",'i',&snd_frqrng);

  /* setup the remaining shell variables */
  radar_shell_parse(&rstable,                                      
                    "sbm l ebm l \                           
                     day_start_freq l night_start_freq l \               
                     day_frang l night_frang l day_mpinc l \             
                     night_mpinc l freq_range l xcf l",                        
                     &sbm,&ebm,                              
                     &day_start_freq,&night_start_freq,                  
                     &day_frang,&night_frang,                            
                     &day_mpinc,&night_mpinc,                            
                     &freq_range,
                     &xcf);

  /* process the command line options */
  argnum=option_process(argc,argv,&cmdlne,cmdlne_err);
  if (argnum<argc) {
     optfp=fopen(argv[argnum],"r");
     if (optfp !=NULL) {
        option_file(optfp,&cmdlne,cmdlne_err);
        fclose(optfp);
     } else fprintf(stderr,"Failed to read option file.\n");
  }

  /* register with the error log and the scheduler */
  etsk=make_task_id(errlog_ptr);
  sid=register_program(scheduler_ptr,CONTROL_NAME);

  txpl=(rsep*20)/3;

 /* setup the comment block */
  strcpy(command,argv[0]);
  for (c=1;c<argc;c++) {
    strcat(command," ");
    strcat(command,argv[c]);
  }

  strncpy(combf,program_id,80);

  /* log startup */  
  log_start(etsk,program_name,argc,argv);

  /* configure the hardware */
  ad_id=locate_task_id(a_d_name);
  dio_id=locate_task_id(dio_name);


  if ((ad_id==-1) || (dio_id==-1)) {
    log_error(etsk,program_name,"Failed to locate hardware.");
    exit(-1);
  }

  reset_id=get_scan_reset(ad_id);

  bufnum=get_buf_num(ad_id);
  bufsze=get_buf_size(ad_id);

  for (i=0;i<bufnum;i++) {
    bufadr[i]=get_buf_adr(ad_id,i);
  }

  tsg_table=make_tsg_table(MAX_TSG);

  reset_dio(dio_id);
  fclr_set(dio_id,tsg_table,fclr_id);

  hdw=setup_hardware();
  tsk=setup_task(tasklist,&tnum);
  fblk=setup_fit(st_id,hdw);
  frq_table=get_freq_table();

  /* open dummy data file */
  if (test_file !=NULL) {
    log_error(etsk,program_name,"Opening test data file.");
    rfp=raw_open(test_file,NULL);
  }

  read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);

  log_error(etsk,program_name,"Resetting tasks.");

  /* open the first set of files */
  for (i=0;i<tnum;i++) {
    task_reset(tsk[i]);
    task_open(tsk[i],strlen(command),
              command,year,month,day,hour,minut,sec);
  }

  read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);

  if (backward) usr_resS1=-1;
  else usr_resS1=1;

  do {
    read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);

    if (start_scan()==0) continue;

    /* test and re-open files if necessary */
    if (test_reopen(2,0,0) !=0) {
      for (i=0;i<tnum;i++) {
        log_error(etsk,program_name,"Opening new files.");
        task_close(tsk[i],year,month,day,hour,minut,sec);
        task_open(tsk[i],strlen(command),command,
                  year,month,day,hour,minut,sec);
      }
    }

    if (xcount>0) {
      count++;
      if (count==xcount) {
        xcf=1;
        count=0;
      } else xcf=0;
    } else xcf=0;

    /* work out how many beams to skip */
    skip_beam=calc_skip(scan_period);

    scan=1;

    bmseqnum=skip_beam;
    if (backward) {
      bmnum=backward_beams[bmseqnum];
      if (bmnum<ebm) bmnum=sbm;
    } else {
      bmnum=forward_beams[bmseqnum];
      if (bmnum>ebm) bmnum=sbm;
    }

    log_error(etsk,program_name,"Starting scan.");

    usr_resS2=0;
    usr_resS3=0;

    do {

      if (day_time()==1) {
         start_freq=day_start_freq;
         mpinc=day_mpinc;
         frang=day_frang;
      } else {
         start_freq=night_start_freq;
         mpinc=night_mpinc;
         frang=night_frang;
      }

      read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);

      /*integrate(freq_dwell);*/

      /* set beam */
      set_beam(dio_id,bmnum);

      /* do fclr */
      tfreq=fclr(debug,start_freq,start_freq+freq_range,5);
      if (tfreq==-1) tfreq=frq_table->dfrq;

      /* set to the freq that was returned by fclr */
      if (set_freq(dio_id,tfreq)==FREQ_LOCAL) 
        log_error(etsk,program_name,
                  "Frequency Synthesizer in local mode.\n");

      /* do integration */

      tsg_id=time_seq(dio_id,tsg_table,ptab);

      nave=radar(debug,lags);
      if (nave==0) log_error(etsk,program_name,"Integration Failed.");

      get_status(0);
      if (nave>0) {
        if (rfp !=NULL) {
          if (read_data(rfp) !=0) {
            raw_close(rfp);
            rfp=raw_open(TEST_DATA,NULL);
          }
        }

        /* transmit data */
        build_raw(&raw,ptab,lags);
        fitacf(&raw,fblk,&fit);

        msg.num=0;
        msg.tsize=0;
        task_add(&msg,sizeof(struct rawdata),(unsigned char *) &raw,0,
                 year,month,day,hour,minut,sec);
        task_add(&msg,sizeof(struct fitdata),(unsigned char *) &fit,1,
                 year,month,day,hour,minut,sec);
        task_add(&msg,strlen(program_name)+1,program_name,2,
                 year,month,day,hour,minut,sec);

        for (i=0;i<tnum;i++) task_send(tsk[i],&msg);
      }
      exit_poll=radar_shell(sid,&rstable);

      if (exit_poll !=0) break;
      scan=0;
      if (bmseqnum==(num_scans-1)) break;

      /* Change the beamnum to the next beam */
      bmseqnum++;

      if (backward) {
        bmnum=backward_beams[bmseqnum];
        if (bmnum<ebm) bmnum=sbm;
      } else {
        bmnum=forward_beams[bmseqnum];
        if (bmnum>ebm) bmnum=sbm;
      }
    } while (1);

    if (exit_poll==0) {
      /* set the "sounder mode" scan variable */
      scan=-2;

      /* set the xcf variable to do cross-correlations (AOA) */
      xcf=1;

      /* setup the sounder mode integration time */
      /* determine the number of seconds we have for this mode  */
      /* subtract the current sec from 59 so we have at least   */
      /* one second to spare.                                   */
      read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);
      snd_time = 60.0 - (sec+msec/1000.);

      while(snd_time-(float)snd_intt > time_needed) {
        intt = snd_intt;

        /* set the beam */
        bmnum = snd_bms[snd_bm_cnt] + odd_beams;
        set_beam(dio_id,bmnum);

        /* snd_freq will be an array of frequencies to step through */
        snd_freq = snd_freqs[snd_freq_cnt];

        /* call the site library */
        /*integrate(freq_dwell);*/

        /* look for a clear freq */
        tfreq=fclr(debug,snd_freq,snd_freq+snd_frqrng,5);
        if( tfreq==-1 ) tfreq=frq_table->dfrq;

        /* and set the frequency */
        if (set_freq(dio_id,tfreq)==FREQ_LOCAL) 
          log_error(etsk,program_name,
                    "Frequency Synthesizer in local mode.\n");
        /*set_freq(dio_id,tfreq);*/

        /* do integration */
        tsg_id=time_seq(dio_id,tsg_table,ptab);
        nave=radar(debug,lags);
        if (nave==0) log_error(etsk,program_name,"Integration Failed.");

        get_status(0);

        /* build the raw data record */
        build_raw(&raw,ptab,lags);
        /* do the acf fit and make a fit record */
        fitacf(&raw,fblk,&fit);

        /* clear the message buffers */
        msg.num=0;
        msg.tsize=0;
        /* add the raw data to the message buffer */
        task_add(&msg,sizeof(struct rawdata),(unsigned char *) &raw,0,
                 year,month,day,hour,minut,sec);
        /* add the fit data to the message buffer */
        task_add(&msg,sizeof(struct fitdata),(unsigned char *) &fit,1,
                 year,month,day,hour,minut,sec);
        /* add the control program name to the message buffer */
        task_add(&msg,strlen(program_name)+1,program_name,2,
                 year,month,day,hour,minut,sec);

        /* send the message buffers to the tasks */
        //for (i=0;i<tnum;i++) task_send(tsk[i],&msg);
        task_send(tsk[0],&msg);

        /* set the scan variable for the sounding mode data file only */
        if ((bmnum == snd_bms[0]) && (snd_freq == snd_freqs[0])) {
          raw.prms.SCAN = 1;
        } else {
          raw.prms.SCAN = 0;
        }

        /* save the sounding mode data */
        write_snd_record(raw);

        /* is the scheduler trying to shut us down */
        exit_poll=radar_shell(sid,&rstable);

        /* yes, if not equal to 0 */
        if (exit_poll !=0) break;

        /* check for the end of a beam loop */
        snd_freq_cnt++;
        if (snd_freq_cnt >= snd_freqs_tot ) {
          /* reset the freq counter and increment the 
             beam counter */
          snd_freq_cnt = 0;
          snd_bm_cnt++;
          if (snd_bm_cnt >= snd_bms_tot) {
            snd_bm_cnt=0;
            odd_beams = !odd_beams;
          }
        }

        /* see if we have enough time for another go round */ 
        read_clock(&year,&month,&day,&hour,&minut,&sec,&msec,&usec);
        snd_time = 60.0 - (sec+msec/1000.);
      }

      /* now wait for the next interleavescan */
      intt = fast_intt;
      wait_boundary(scan_period);
    }

  bmseqnum = 0;

  } while (exit_poll==0);

  for (i=0;i<tnum;i++) {
    task_close(tsk[i],year,month,day,hour,minut,sec);
  }

  end_program();

  terminate(sid);

}


/********************** function write_snd_record() ************************/
void write_snd_record(struct rawdata raw) {

  char data_path[100], data_filename[50], filename[80];

  char *snd_dir;
  FILE *out;

  int thresh=0;

  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if(snd_dir == NULL)
    sprintf(data_path,"/data/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.dat.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", raw.prms.YEAR, raw.prms.MONTH, raw.prms.DAY, (raw.prms.HOUR/2)*2, getenv("SD_RADARCODE"));

  /* finally make the filename */
  sprintf(filename,"%s%s.dat.snd",data_path,data_filename);

  /* check to see if output file exists */
  out = fopen(filename,"r");

  /* if file doesn't exist, create a new output file */
  if (out==NULL) {
    out = fopen(filename,"w");
    if (out==NULL) {
      fprintf(stderr, "Error creating sounding file: %s\n", filename);
      return;
    }

    if (raw_header_fwrite("rawwrite",VSTRING,thresh,"interleavesound",out) !=0) {
      fprintf(stderr,"Error writing sounding data header\n");
      fclose(out);
      return;
    }

    /* reset the record number to zero for new file */
    recnum = 0;
  }
  fclose(out);

  /* open the output file */
  out = fopen(filename,"a");
  if (out==NULL) {
    /* crap. might as well go home */
    fprintf(stderr, "Error writing sounding data: %s\n", filename);
    return;
  }

  recnum++;
  raw_fwrite(out,"rawwrite",raw,thresh,recnum);

  fclose(out);
}

