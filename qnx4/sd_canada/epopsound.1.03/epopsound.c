/*
 * epopsound.c ============ Author: Dieter Andre 
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <process.h>
#include <unistd.h>
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
 * $Log: epopsound.c,v $ 
 * Revision 1.03 2018/07/26 22:00:00 KKrieger
 * Addition of option to change integration time
 *
 * Revision 1.02 2015/11/09 22:00:00 KKrieger
 * Addition of multiple frequencies
 *
 * Revision 1.01 2014/11/18 21:00:00 KKrieger
 * Optional marker pulse sequence added
 *
 * Revision 1.00 2013/10/29 21:00:00 KKrieger
 * Initial revision. Based off ddstest.1.03
 * 
 */

#define SCHEDULER "schedule"
#define ERRLOG "errlog"

#define CONTROL_NAME "control_program"
#define TASK_NAMES "echo_data","iqwrite","rawacfwrite","fitacfwrite"

#define MAX_FREQS 8

char cmdlne[1024];
char progid[80] = { "$Id: epopsound.c,v 1.03 2018/07/26 22:00:00 KKrieger Exp $" };
char progname[256];

/*
 * Keep track of all details of an epop pass frequency in kHz beam number 
 * (indexed from 0) start time, stop time, duration that epop is in beam, in
 * float seconds
 * 
 * define an Epop_Pass by: Epop_Pass some_epop_pass 
 */
typedef struct Epop_Pass {
	int freq_khz;
	int beam;
	struct tm start;
	struct tm stop;
	float duration_s;
} Epop_Pass;

/*
 * Get a file descriptor to open up the epop_passes file 
 */
FILE *fd;
struct TaskID *errlog;

char *tasklist[] = { TASK_NAMES,
	0
};

int arg = 0;
struct OptionData opt;

int main(int argc, char *argv[])
{
	/* Option to have a 'marker' pulse sequence every x beams.
	We will use the 7 pulse sequence for this and have
	command line flags to enable and configure the period of
	this marker pulse sequence */
	int ptab_7[7] = { 0, 9, 12, 20, 22, 26, 27};

	int lags_7[LAG_SIZE][2] = {
		{0,0},
		{26,27},
		{20,22},
		{9,12},
		{22,26},
		{22,27},
		{20,26},
		{20,27},
		{12,20},
		{0,9},
		{12,22},
		{9,20},
		{0,12},
		{9,22},
		{12,26},
		{12,27},
		
		{9,26},
		{9,27},

		{27,27}};

	int mppul_7 = 7;
	int mplgs_7 = 18;
	int mpinc_7 = 2400;
	int dmpinc_7 = 2400;
	int nmpinc_7 = 2400;
	int use_marker = 0; /* Default do not user marker */
	int marker_period = 15; /* Default every 15 beams marker period */
	int marker_counter = marker_period; /* Counter for the marker periods */

	int ptab_8[8] = { 0, 14, 22, 24, 27, 31, 42, 43 };

	int lags_8[LAG_SIZE][2] = {
		{0, 0},		/* 0 */
		{42, 43},	/* 1 */
		{22, 24},	/* 2 */
		{24, 27},	/* 3 */
		{27, 31},	/* 4 */
		{22, 27},	/* 5 */

		{24, 31},	/* 7 */
		{14, 22},	/* 8 */
		{22, 31},	/* 9 */
		{14, 24},	/* 10 */
		{31, 42},	/* 11 */
		{31, 43},	/* 12 */
		{14, 27},	/* 13 */
		{0, 14},	/* 14 */
		{27, 42},	/* 15 */
		{27, 43},	/* 16 */
		{14, 31},	/* 17 */
		{24, 42},	/* 18 */
		{24, 43},	/* 19 */
		{22, 42},	/* 20 */
		{22, 43},	/* 21 */
		{0, 22},	/* 22 */

		{0, 24},	/* 24 */

		{43, 43}
	};			/* alternate lag-0 */

	int mppul_8 = 8;
	int mplgs_8 = 23;
	int mpinc_8 = 1500;
	int dmpinc_8 = 1500;
	int nmpinc_8 = 1500;

	char *sname = NULL;
	char *ename = NULL;
	char *sdname = { SCHEDULER };
	char *edname = { ERRLOG };
	char logtxt[1024];
	char pass_buffer[1024];	/* Buffer to store passes information */
	FILE *fp = NULL;

	/*
	 * What is the file name for the epop passes file? 
	 */
	char *epop_passes_fname = NULL;
	char *default_epop_passes_fname = "/data/scd/epop.scd";

	/*
	 * Keep track of current pass and next pass 
	 */
	Epop_Pass current_pass;
	Epop_Pass next_pass;

	int n;
	pid_t sid;
	int exitpoll = 0;

/*	int scnsc = 1; These are not used
	int scnus = 0;
	int skip; */
	int cnt = 0;
	int fixfrq[MAX_FREQS] = {0};		/* Fixed frequencies to transmit on (kHz) */
	int startbeam = 0; /* Optional start and stop beams */ 
	int stopbeam = 15; /*  for default mode camping */
	int num_freqs = 1; /* Default 1 frequency to transmit on */
	int freq_counter = 0; /* Counter to decide when to transmit what frequency */

	/*
	 * Flags for default_mode and next_pass_found are set while scanning
	 * conjunction file, as well as current_conjunction
	 */
	int default_mode = 0;
	int next_pass_found = 0;
	int current_conjunction = 0;

	/* variable for temporary use */
	int temp = 0;

	/* Double to store time diffs */
	double time_diff = 0;
	/* Time structure to store time */
	time_t time_of_day;

	unsigned char discretion = 0;

	/*
	 * Get the command line arguments 
	 */
	strcpy(cmdlne, argv[0]);
	for (n = 1; n < argc; n++) {
		strcat(cmdlne, " ");
		strcat(cmdlne, argv[n]);
	}
	
	strncpy(combf, progid, 80);
	OpsSetupCommand(argc, argv);
	OpsSetupRadar();
	OpsSetupShell();

	RadarShellParse(&rstable,
			"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",
			&sbm, &ebm, &dfrq, &nfrq, &dfrang, &nfrang, &dmpinc,
			&nmpinc, &frqrng, &xcnt);

	cp = 3371;		/* Picked to be after ddstest */
	intsc = 1;
	intus = 0;		/* default value picked to be small enough to get 1
				 * second beam switching resolution */
	mppul = mppul_8;
	mplgs = mplgs_8;
	mpinc = mpinc_8;
	dmpinc = dmpinc_8;
	nmpinc = nmpinc_8;
	nrang = 75;
	rsep = 45;
	txpl = 300;
	stfrq = nfrq;
	frang = nfrang;
	SiteStart();

	/*
	 * ========= PROCESS COMMAND LINE ARGUMENTS ============= 
	 */

	OptionAdd(&opt, "di", 'x', &discretion);

	OptionAdd(&opt, "el", 't', &ename);
	OptionAdd(&opt, "sc", 't', &sname);
	OptionAdd(&opt, "rsep", 'i', &rsep);
	OptionAdd(&opt, "xcf", 'i', &xcnt);

	OptionAdd(&opt, "epopfile", 't', &epop_passes_fname);	/* Passes file for Cassiope */
    	OptionAdd(&opt, "startbeam", 'i', &startbeam);
	OptionAdd(&opt, "stopbeam", 'i', &stopbeam);
	OptionAdd(&opt, "fixfrq1", 'i', &fixfrq[0]);
	OptionAdd(&opt, "fixfreq1", 'i', &fixfrq[0]); /*add a common misspelling of fixfrq*/
	OptionAdd(&opt, "fixfrq2", 'i', &fixfrq[1]);
	OptionAdd(&opt, "fixfreq2", 'i', &fixfrq[1]);
	OptionAdd(&opt, "fixfrq3", 'i', &fixfrq[2]);
	OptionAdd(&opt, "fixfreq3", 'i', &fixfrq[2]);
	OptionAdd(&opt, "fixfrq4", 'i', &fixfrq[3]);
	OptionAdd(&opt, "fixfreq4", 'i', &fixfrq[3]);
	OptionAdd(&opt, "fixfrq5", 'i', &fixfrq[4]);
	OptionAdd(&opt, "fixfreq5", 'i', &fixfrq[4]);
	OptionAdd(&opt, "fixfrq6", 'i', &fixfrq[5]);
	OptionAdd(&opt, "fixfreq6", 'i', &fixfrq[5]);
	OptionAdd(&opt, "fixfrq7", 'i', &fixfrq[6]);
	OptionAdd(&opt, "fixfreq7", 'i', &fixfrq[6]);
	OptionAdd(&opt, "fixfrq8", 'i', &fixfrq[7]);
	OptionAdd(&opt, "fixfreq8", 'i', &fixfrq[7]);
	OptionAdd(&opt, "num_freqs", 'i', &num_freqs);
	OptionAdd(&opt, "use_marker", 'i',&use_marker);
	OptionAdd(&opt, "marker_period",'i',&marker_period);
	OptionAdd(&opt, "intsc",'i',&intsc);
	OptionAdd(&opt, "intus",'i',&intus);
	arg = OptionProcess(1, argc, argv, &opt, NULL);

	/* Error checking on num_freqs */
	if(num_freqs < 1 || num_freqs > MAX_FREQS) {
		sprintf(logtxt, "Error, %d different frequencies not allowed, min is 1, max is %d", num_freqs, MAX_FREQS);
		ErrLog(errlog,progname, logtxt);
	}

	/* Error checking on use_marker and marker_period */
	if(use_marker != 0) use_marker = 1;
	if(marker_period < 0) marker_period = 0;
	/* Set marker counter to the period so we send marker
       on the first beam */
	marker_counter = marker_period;
	
	/* Error checking on start and stop beams */
	if (startbeam < 0) startbeam = 0;
	if (stopbeam > 15) stopbeam = 15;
	if (startbeam > stopbeam) {
		temp = stopbeam;
		stopbeam = startbeam;
		startbeam = temp;
	}
	if(backward) bmnum = stopbeam;
	else bmnum = startbeam; 
	
	if (sname == NULL)
		sname = sdname;
	if (ename == NULL)
		ename = edname;

	sid = RShellRegister(sname, CONTROL_NAME);

	errlog = TaskIDMake(ename);
	OpsLogStart(errlog, progname, argc, argv);
	sprintf(logtxt, "startbeam: %d, stopbeam: %d", startbeam, stopbeam);
	ErrLog(errlog,progname,logtxt);	
	if(use_marker) {
		sprintf(logtxt, "Using marker sequence. Every %d beams",marker_period);
		ErrLog(errlog,progname,logtxt);
	}
	SiteSetupHardware();

	if (discretion)
		cp = -cp;
	txpl = (rsep * 20) / 3;

	sprintf(progname, "epopsound");

	/*
	 * Open the epop conjunction file 
	 */
	if (epop_passes_fname == NULL) {
		sprintf(logtxt,"No epop conjunction file name give, using default: %s",default_epop_passes_fname);
		ErrLog(errlog, progname, logtxt);
		epop_passes_fname = default_epop_passes_fname;
	}
	fp = fopen(epop_passes_fname, "r");
	if (fp == NULL) {
		sprintf(logtxt,"Error # %d opening epop conjuction file: %s",errno, epop_passes_fname);
		ErrLog(errlog, progname, logtxt);
		default_mode = 1;
	}
	OpsFitACFStart();

	OpsSetupTask(tasklist);
	for (n = 0; n < tnum; n++) {
		RMsgSndReset(tlist[n]);
		RMsgSndOpen(tlist[n], strlen(cmdlne), cmdlne);
	}

	do {/* while(exitpoll == 0) */
		/* Check state here */
		if(!default_mode) {
			if (current_conjunction) {
				if (difftime(time(NULL),mktime(&current_pass.stop)) > 0) {
					current_conjunction = 0;
				}
			}
			if (next_pass_found) {
				if (difftime(time(NULL),mktime(&next_pass.start)) > 0) {
					current_conjunction = 1;
					current_pass = next_pass;
					next_pass_found = 0;
				}
			}
		}
		if (current_conjunction) ErrLog(errlog, progname, "state: current_conjunction");
		if (default_mode) ErrLog(errlog, progname, "state: default_mode");
		if (next_pass_found) ErrLog(errlog, progname, "state: next_pass_found");
		
		/* Only if we have a file open and no current conjunction or next pass should we check the file*/
		if (fp != NULL &&  current_conjunction == 0 && next_pass_found == 0) {
			sprintf(logtxt,"Scanning epop conjunction file: %s",epop_passes_fname);
			ErrLog(errlog, progname, logtxt);

			/*
			 * Read lines until you find one that has a stop time in the
			 * future, in which case: if start time is in past, then set mode
			 * to that freq and beam else go to default mode until start time
			 * or until you reach EOF, in which case you just go to default
			 * mode. 
			 */
			do { /*while (!next_pass_found && !default_mode);*/

				/* Need to set dst bits to 0 or they will make our time values incorrect */
				current_pass.start.tm_isdst = 0;
				current_pass.stop.tm_isdst = 0;
				next_pass.start.tm_isdst = 0;
				next_pass.stop.tm_isdst = 0;
			
				if (fgets(pass_buffer, 1024, fp) == NULL) {
					if (!feof(fp)) {
						ErrLog(errlog, progname,"Unable to successfully read all of conjunction.");
						/*
						 * Set status to default, since we couldn't read conjunction  
						 */
						default_mode = 1;
						next_pass_found = 0;
						current_conjunction = 0;
					} else {
						/*
						 * Reached eof, so set status to default mode
						 */
						ErrLog(errlog, progname,"Reached end of file without finding a current or future pass.");
						ErrLog(errlog, progname,"Setting default mode.");
						default_mode = 1;
						next_pass_found = 0;
						current_conjunction = 0;
					}
				} else {
					ErrLog(errlog, progname,"Read pass successfully, scanning with sscanf");
					ErrLog(errlog, progname,pass_buffer);
					if (sscanf
					    (pass_buffer,
					     "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %f",
					     &next_pass.freq_khz,
					     &next_pass.beam,
					     &next_pass.start.tm_year,
					     &next_pass.start.tm_mon,
					     &next_pass.start.tm_mday,
					     &next_pass.start.tm_hour,
					     &next_pass.start.tm_min,
					     &next_pass.start.tm_sec,
					     &next_pass.stop.tm_year,
					     &next_pass.stop.tm_mon,
					     &next_pass.stop.tm_mday,
					     &next_pass.stop.tm_hour,
					     &next_pass.stop.tm_min,
					     &next_pass.stop.tm_sec,
					     &next_pass.duration_s) != 15) {
						/*
						 * error scanning input file. Print message to errlog
						 * and go to default mode. 
						 */
						sprintf(logtxt,
							"ERROR: Unsuccessfully scanned input pass! Going to default mode");
						ErrLog(errlog, progname,
						       logtxt);
						default_mode = 1;
						next_pass_found = 0;
						current_conjunction = 0;
					} else {
						/*
						 * The month is indexed from 0 in the tm struct, so
						 * remove 1 from the structs. The year is indexed from 1900
						 * so remove 1900 from year. 
						 */
						next_pass.start.tm_mon = next_pass.start.tm_mon - 1;
						next_pass.stop.tm_mon = next_pass.stop.tm_mon - 1;
						next_pass.start.tm_year = next_pass.start.tm_year - 1900;
						next_pass.stop.tm_year = next_pass.stop.tm_year - 1900;
						sprintf(logtxt,
							"Freq kHz: %d\nBeam: %d\nStart: %d %d %d %d %d %d\nStop: %d %d %d %d %d %d\nDuration: %f",
							next_pass.freq_khz,
							next_pass.beam,
							next_pass.start.tm_year+1900,
							next_pass.start.tm_mon+1,
							next_pass.start.tm_mday,
							next_pass.start.tm_hour,
							next_pass.start.tm_min,
							next_pass.start.tm_sec,
							next_pass.stop.tm_year+1900,
							next_pass.stop.tm_mon+1,
							next_pass.stop.tm_mday,
							next_pass.stop.tm_hour,
							next_pass.stop.tm_min,
							next_pass.stop.tm_sec,
							next_pass.duration_s);
						ErrLog(errlog, progname,logtxt);

						sprintf(logtxt,"Duration calculated: %d seconds", (int)difftime(mktime(&next_pass.stop),mktime(&next_pass.start)));
						ErrLog(errlog, progname,logtxt);
						
						/* Compare stop time with current time */
						time_of_day = time(NULL);
						sprintf(logtxt, "Current time: %s ", ctime(&time_of_day));
						ErrLog(errlog,progname,logtxt);
						time_of_day = mktime(&next_pass.stop);
						sprintf(logtxt, "Pass stop time: %s ", ctime(&time_of_day));
						ErrLog(errlog,progname,logtxt);
						time_diff = difftime(mktime(&next_pass.stop),time(NULL));
						sprintf(logtxt, "Difference in time from now to pass: %f seconds", time_diff);
						ErrLog(errlog,progname, logtxt);
						if(time_diff > 0.0) {
							/*
							 * Stop time is in the future, so we found our next pass
							 */
							next_pass_found = 1;
							default_mode = 0;
							ErrLog(errlog, progname,"Next pass found");
						} else {
							ErrLog(errlog,progname, "Stop time is in the past..keep looking.");
						}
					}
				}
				/*
				 * we don't have a current or future pass yet, or we haven't
				 * reached EOF yet 
				 */
			} while (!next_pass_found && !default_mode);
		}

		/*
		 * Here we should either be in default mode or have a next pass to
		 * wait for 
		 */
		if (!default_mode) {
			if (time_diff = difftime(time(NULL), mktime(&next_pass.start)) > 0.0) {
				/* IF the next pass' start time is in the past... then we should make it the current pass */
				current_pass = next_pass;
				current_conjunction = 1;
				next_pass_found = 0;
				sprintf(logtxt,"Current pass: %d seconds left, beam %d at freq %d kHz",
					(int)difftime(mktime(&current_pass.stop),time(NULL)), current_pass.beam,current_pass.freq_khz);
				ErrLog(errlog, progname, logtxt);
			} else {
				next_pass_found = 1;
				current_conjunction = 0;
				sprintf(logtxt,"Next pass is %d seconds in future",(int)difftime(time(NULL),mktime(&next_pass.start)));
				ErrLog(errlog, progname, logtxt);
			}
		}

		/*
		 * Currently SiteStartScan() just returns 1 
		 */
		if (SiteStartScan() == 0)
			continue;

		/*
		 * OpsReOpen(int hbnd, int mtbnd, int scbnd) will return 1 when
		 * it is first called, and at every hbdn*3600+60*mtbnd+scbnd
		 * seconds of the day. It opens new files (ex: rawacf) 
		 */
		if (OpsReOpen(2, 0, 0) != 0) {
			ErrLog(errlog, progname, "Opening new files.");
			for (n = 0; n < tnum; n++) {
				RMsgSndClose(tlist[n]);
				RMsgSndOpen(tlist[n], strlen(cmdlne), cmdlne);
			}
		}

		scan = 1;
		ErrLog(errlog, progname, "Starting scan.");

		/*
		 * count up to xcnt on every scan, then set xcf = 1 and reset
		 * counter 
		 */
		if (xcnt > 0) {
			cnt++;
			if (cnt == xcnt) {
				xcf = 1;
				cnt = 0;
			} else
				xcf = 0;
		} else {
 		   	xcf = 0;
		}
		/*
		 * Update the time stored in variables yr, mo, dy, hr, mt, sc, 
		 * us 
		 */
		TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);
		/*
		 * Here we should find out what beam we should be transmitting 
		 * on for epop, and also what fixed frequency. 
		 */
		if(!default_mode && (current_conjunction || next_pass_found)) {
			if(current_conjunction) {
				fixfrq[0] = current_pass.freq_khz;
				if(current_pass.beam >= 0 && current_pass.beam <=15) bmnum = current_pass.beam;
				else {
					sprintf(logtxt, "ERROR: beam # %d is invalid, going to default mode.", current_pass.beam);
					ErrLog(errlog, progname, logtxt);
					default_mode = 1;
					current_conjunction = 0;
					next_pass_found = 0;
				}
			} else {
				fixfrq[0] = next_pass.freq_khz;
				if(next_pass.beam >= 0 && next_pass.beam <= 15) bmnum = next_pass.beam;
				else {
					sprintf(logtxt, "ERROR: beam # %d is invalid for next pass, going to default mode.",next_pass.beam);
					ErrLog(errlog, progname, logtxt);
					default_mode = 1;
					current_conjunction = 0;
					next_pass_found = 0;
				}
			}
		}

		/* If we are going to default mode, then either some error occurred or 
 		 * we don't have any passes that end in the future.
 		 * Default mode is set here */
		if(default_mode == 1) {
			;
		}
		sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)", bmnum, intsc, intus, hr, mt, sc, us);
		ErrLog(errlog, progname, logtxt);

		ErrLog(errlog, progname, "Setting beam.");
		SiteSetIntt(intsc, intus);
		SiteSetBeam(bmnum);
		
		/*
		 * Now check and set the frequency 
		 */
		sprintf(logtxt,"freq_counter %d, fixfrq %d \n",freq_counter,fixfrq[freq_counter]);
        ErrLog(errlog,progname,logtxt);
		if (FreqTest(ftable, fixfrq[freq_counter]) != 0) {
			/*
			 * If our set frequency is restricted, just set to default 
			 * frequency and print error 
			 */
			sprintf(logtxt,"ERROR: Frequency of %d is restricted, using default freq.",tfreq);
			ErrLog(errlog,progname,logtxt);
			fixfrq[freq_counter] = ftable->dfrq;
		}
		
		/*
		 * SiteFCLR(int sfrq, int efrq) will call DDSFCLRGC214(...)
		 * which will determine a band using fft with lowest power to
		 * transmit on. It sets tfreq. Use frequency range of 10kHz
		 * so we can find approx noise
		 */
		/*if (SiteFCLR(fixfrq[freq_counter], fixfrq[freq_counter] + 10) == FREQ_LOCAL)
			ErrLog(errlog, progname,"Frequency Synthesizer in local mode.");
*/
		/* Logic for determining frequency */
		tfreq = fixfrq[freq_counter];
		SiteSetFreq(tfreq);

		sprintf(logtxt, "Transmitting on: %d",tfreq);
		ErrLog(errlog, progname, logtxt);

		if(use_marker) {
			ErrLog(errlog,progname, "Using marker pulse sequence");
			if(marker_counter == marker_period) {
				/* Use the 7 pulse sequence */
				mppul = mppul_7;
				mplgs = mplgs_7;
				mpinc = mpinc_7;
				tsgid = SiteTimeSeq(ptab_7);
				nave = SiteIntegrate(lags_7);
				if(nave < 0) {
					sprintf(logtxt, "Integration error:%d",nave);
					ErrLog(errlog,progname,logtxt);
					spawnl(P_WAIT,"/home/radar/script/restart.radar",NULL);
					exit(nave);
				}
				sprintf(logtxt,"Number of sequences [7]: %d",nave);
				ErrLog(errlog,progname,logtxt);
				OpsBuildPrm(&prm,ptab_7,lags_7);
				marker_counter = 0;
			} else {
				/* Use the 8 pulse sequence */
				mppul = mppul_8;
				mplgs = mplgs_8;
				mpinc = mpinc_8;
				tsgid = SiteTimeSeq(ptab_8);
				nave = SiteIntegrate(lags_8);
				if(nave < 0) {
					sprintf(logtxt, "Integration error:%d",nave);
					ErrLog(errlog,progname,logtxt);
					spawnl(P_WAIT,"/home/radar/script/restart.radar",NULL);
					exit(nave);
				}
				sprintf(logtxt,"Number of sequences [8]: %d",nave);
				ErrLog(errlog,progname,logtxt);
				OpsBuildPrm(&prm,ptab_8,lags_8);
				marker_counter += 1;
			}
		} else {
			ErrLog(errlog,progname, "Not using marker pulse sequence");
			tsgid = SiteTimeSeq(ptab_8);
			nave = SiteIntegrate(lags_8);
			if (nave < 0) {
				sprintf(logtxt, "Integration error:%d", nave);
				ErrLog(errlog, progname, logtxt);
				spawnl(P_WAIT,"/home/radar/script/restart.radar",NULL);
				exit(nave);
			}
		
			sprintf(logtxt, "Number of sequences: %d", nave);
			ErrLog(errlog, progname, logtxt);

			OpsBuildPrm(&prm, ptab_8, lags_8);
		}

		OpsBuildIQ(&iq);
		OpsBuildRaw(&raw);

		FitACF(&prm, &raw, &fblk, &fit);
		ErrLog(errlog, progname, "Sending messages.");

		msg.num = 0;
		msg.tsize = 0;
		RMsgSndAdd(&msg, sizeof(struct RadarParm),(unsigned char *)&prm, PRM_TYPE, 0);
		RMsgSndAdd(&msg, sizeof(struct IQData),(unsigned char *)&iq, IQ_TYPE, 0);
		RMsgSndAdd(&msg, strlen(sharedmemory) + 1, sharedmemory,IQS_TYPE, 0);
		RMsgSndAdd(&msg, sizeof(struct RawData),(unsigned char *)&raw, RAW_TYPE, 0);
		RMsgSndAdd(&msg, sizeof(struct FitData),(unsigned char *)&fit, FIT_TYPE, 0);
		RMsgSndAdd(&msg, strlen(progname) + 1, progname,NME_TYPE, 0);
		for (n = 0; n < tnum; n++) RMsgSndSend(tlist[n], &msg);

		ErrLog(errlog, progname, "Polling for exit.");
		exitpoll = RadarShell(sid, &rstable);
		if (exitpoll != 0) break;
		scan = 0;

		/* update frequency used */
		freq_counter++;
		if(freq_counter == num_freqs) freq_counter = 0;

		/* Set the beam for next scan (overridden if not in default mode)*/
		if (backward) {
			if (bmnum<=startbeam) {
				bmnum=stopbeam;
			} else {
				bmnum--;
			}
		} else {
			if(bmnum>=stopbeam) {
				bmnum=startbeam;
			} else {
				bmnum++;
			} 
		}

		/*ErrLog(errlog, progname, "Waiting for scan boundary.");
		if (exitpoll == 0) OpsWaitBoundary(scnsc, scnus);
*/
	} while (exitpoll == 0);
	
	SiteEnd();
	for (n = 0; n < tnum; n++) RMsgSndClose(tlist[n]);
	ErrLog(errlog, progname, "Ending program.");
	RShellTerminate(sid);
	if ((fp != NULL) && (fp != stderr))	fclose(fp);
	return 0;
}
