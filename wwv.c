/*
 * refclock_wwv - clock driver for NIST WWV/H time/frequency station
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include "ntp_fp.h"
#include "ntp_unixtime.h"
#define CLOCK_CODEC_OFFSET 0
#define MAXGAIN 16383

#ifndef TRUE
# define TRUE 1
#endif /* TRUE */
#ifndef FALSE
# define FALSE 0
#endif /* FALSE */

struct shmTime {
    /* 0 - if valid set -> use values, clear valid
	 * 1 - if valid set -> if count before and after read of values is equal -> use values, clear valid
	 */
    unsigned int mode;
    unsigned int count;
    time_t ct_Sec;
    int    ct_USec;
    time_t rt_Sec;
    int    rt_USec;
    /* precision: log(2) of source jitter */
    int    leap, precision, nsamples;
    volatile unsigned int valid;
    unsigned int dummy[10];
};

/*
 * Audio WWV/H demodulator/decoder
 *
 * This driver synchronizes the computer time using data encoded in
 * radio transmissions from NIST time/frequency stations WWV in Boulder,
 * CO, and WWVH in Kauai, HI. Transmissions are made continuously on
 * 2.5, 5, 10 and 15 MHz from WWV and WWVH, and 20 MHz from WWV. An
 * ordinary AM shortwave receiver can be tuned manually to one of these
 * frequencies or, in the case of ICOM receivers, the receiver can be
 * tuned automatically using this program as propagation conditions
 * change throughout the weasons, both day and night.
 *
 * The driver requires an audio codec or sound card with sampling rate 8
 * kHz and 16-bit sample depth. This is the same standard as used by the
 * telephone industry and is supported by most hardware and operating
 * systems, including Solaris, SunOS, FreeBSD, NetBSD and Linux. In this
 * implementation, only one audio driver and codec can be supported on a
 * single machine.
 *
 * The demodulation and decoding algorithms used in this driver are
 * based on those developed for the TAPR DSP93 development board and the
 * TI 320C25 digital signal processor described in: Mills, D.L. A
 * precision radio clock for WWV transmissions. Electrical Engineering
 * Report 97-8-1, University of Delaware, August 1997, 25 pp., available
 * from www.eecis.udel.edu/~mills/reports.html. The algorithms described
 * in this report have been modified somewhat to improve performance
 * under weak signal conditions and to provide an automatic station
 * identification feature.
 *
 * The ICOM code is normally compiled in the driver. It isn't used,
 * unless the mode keyword on the server configuration command specifies
 * a nonzero ICOM ID select code. The C-IV trace is turned on if the
 * debug level is greater than one.
 *
 * Fudge factors
 *
 * Fudge flag4 causes the dubugging output described above to be
 * recorded in the clockstats file. Fudge flag2 selects the audio input
 * port, where 0 is the mike port (default) and 1 is the line-in port.
 * It does not seem useful to select the compact disc player port. Fudge
 * flag3 enables audio monitoring of the input signal. For this purpose,
 * the monitor gain is set to a default value.
 *
 * CEVNT_BADTIME	invalid date or time
 * CEVNT_PROP		propagation failure - no stations heard
 * CEVNT_TIMEOUT	timeout (see newgame() below)
 */
/*
 * General definitions. These ordinarily do not need to be changed.
 */
#define	DEVICE_AUDIO	"/dev/audio" /* audio device name */
#define	AUDIO_BUFSIZ	320	/* audio buffer size (50 ms) */
#define	PRECISION	(-10)	/* precision assumed (about 1 ms) */
#define	DESCRIPTION	"WWV/H Audio Demodulator/Decoder" /* WRU */
#define SECOND		8000	/* second epoch (sample rate) (Hz) */
#define MINUTE		(SECOND * 60) /* minute epoch */
#define OFFSET		128	/* companded sample offset */
#define SIZE		256	/* decompanding table size */
#define	MAXAMP		6000.0f	/* max signal level reference */
#define	MAXCLP		100	/* max clips above reference per s */
#define MAXSNR		20.0f	/* max SNR reference */
#define MAXFREQ		1.5f	/* max frequency tolerance (187 PPM) */
#define DATCYC		170	/* data filter cycles */
#define DATSIZ		(DATCYC * MS) /* data filter size */
#define SYNCYC		800	/* minute filter cycles */
#define SYNSIZ		(SYNCYC * MS) /* minute filter size */
#define TCKCYC		5	/* tick filter cycles */
#define TCKSIZ		(TCKCYC * MS) /* tick filter size */
#define NCHAN		4	/* number of radio channels */
#define	AUDIO_PHI	5e-6f	/* dispersion growth factor */
#define	TBUF		128	/* max monitor line length */
#define BMAX        128 /* max timecode length */

/*
 * Tunable parameters. The DGAIN parameter can be changed to fit the
 * audio response of the radio at 100 Hz. The WWV/WWVH data subcarrier
 * is transmitted at about 20 percent percent modulation; the matched
 * filter boosts it by a factor of 17 and the receiver response does
 * what it does. The compromise value works for ICOM radios. If the
 * radio is not tunable, the DCHAN parameter can be changed to fit the
 * expected best propagation frequency: higher if further from the
 * transmitter, lower if nearer. The compromise value works for the US
 * right coast.
 */
#define DCHAN		3	/* default radio channel (15 Mhz) */
#define DGAIN		5.0f	/* subcarrier gain */

/*
 * General purpose status bits (status)
 *
 * SELV and/or SELH are set when WWV or WWVH have been heard and cleared
 * on signal loss. SSYNC is set when the second sync pulse has been
 * acquired and cleared by signal loss. MSYNC is set when the minute
 * sync pulse has been acquired. DSYNC is set when the units digit has
 * has reached the threshold and INSYNC is set when all nine digits have
 * reached the threshold. The MSYNC, DSYNC and INSYNC bits are cleared
 * only by timeout, upon which the driver starts over from scratch.
 *
 * DGATE is lit if the data bit amplitude or SNR is below thresholds and
 * BGATE is lit if the pulse width amplitude or SNR is below thresolds.
 * LEPSEC is set during the last minute of the leap day. At the end of
 * this minute the driver inserts second 60 in the seconds state machine
 * and the minute sync slips a second.
 */
#define MSYNC		0x0001	/* minute epoch sync */
#define SSYNC		0x0002	/* second epoch sync */
#define DSYNC		0x0004	/* minute units sync */
#define INSYNC		0x0008	/* clock synchronized */
#define FGATE		0x0010	/* frequency gate */
#define DGATE		0x0020	/* data pulse amplitude error */
#define BGATE		0x0040	/* data pulse width error */
#define	METRIC		0x0080	/* one or more stations heard */
#define LEPSEC		0x1000	/* leap minute */

/*
 * Station scoreboard bits
 *
 * These are used to establish the signal quality for each of the five
 * frequencies and two stations.
 */
#define SELV		0x0100	/* WWV station select */
#define SELH		0x0200	/* WWVH station select */

/*
 * Alarm status bits (alarm)
 *
 * These bits indicate various alarm conditions, which are decoded to
 * form the quality character included in the timecode.
 */
#define CMPERR		0x1	/* digit or misc bit compare error */
#define LOWERR		0x2	/* low bit or digit amplitude or SNR */
#define NINERR		0x4	/* less than nine digits in minute */
#define SYNERR		0x8	/* not tracking second sync */

/*
 * Watchcat timeouts (watch)
 *
 * If these timeouts expire, the status bits are mashed to zero and the
 * driver starts from scratch. Suitably more refined procedures may be
 * developed in future. All these are in minutes.
 */
#define ACQSN		6	/* station acquisition timeout */
#define DATA		15	/* unit minutes timeout */
#define SYNCH		40	/* station sync timeout */
#define PANIC		(2 * 1440) /* panic timeout */

/*
 * Thresholds. These establish the minimum signal level, minimum SNR and
 * maximum jitter thresholds which establish the error and false alarm
 * rates of the driver. The values defined here may be on the
 * adventurous side in the interest of the highest sensitivity.
 */
#define MTHR		13.0f	/* minute sync gate (percent) */
#define TTHR		50.0f	/* minute sync threshold (percent) */
#define AWND		20	/* minute sync jitter threshold (ms) */
#define ATHR		2500.0f	/* QRZ minute sync threshold */
#define QTHR		2500.0f	/* QSY minute sync threshold */
#define STHR		2500.0f	/* second sync threshold */
#define ASNR		10.0f	/* QRZ minute sync SNR threshold (dB) */
#define QSNR		10.0f	/* QSY minute sync SNR threshold (dB) */
#define	SSNR		 7.5f	/* second sync SNR threshold (dB) */
#define SCMP		10 	/* second sync compare threshold */
#define DTHR		1000.0f	/* bit threshold */
#define DSNR		5.0f	/* bit SNR threshold (dB) */
#define AMIN		3	/* min bit count */
#define AMAX		6	/* max bit count */
#define BTHR		1000.0f	/* digit threshold */
#define BSNR		1.5f	/* digit likelihood threshold (dB) */
#define BCMP		3	/* digit compare threshold */
#define	MAXERR		40	/* maximum error alarm */

/*
 * Tone frequency definitions. The increments are for 4.5-deg sine
 * table.
 */
#define MS		(8) /* samples per millisecond */
#define IN100		((100 * 80) / SECOND) /* 100 Hz increment */
#define IN1000		((1000 * 80) / SECOND) /* 1000 Hz increment */
#define IN1200		((1200 * 80) / SECOND) /* 1200 Hz increment */

/*
 * Acquisition and tracking time constants
 */
#define MINAVG		8	/* min averaging time */
#define MAXAVG		1024	/* max averaging time */
#define FCONST		3	/* frequency time constant */
#define TCONST		16	/* data bit/digit time constant */

/*
 * Miscellaneous status bits (misc)
 *
 * These bits correspond to designated bits in the WWV/H timecode. The
 * bit probabilities are exponentially averaged over several minutes and
 * processed by a integrator and threshold.
 */
#define DUT1		0x01	/* 56 DUT .1 */
#define DUT2		0x02	/* 57 DUT .2 */
#define DUT4		0x04	/* 58 DUT .4 */
#define DUTS		0x08	/* 50 DUT sign */
#define DST1		0x10	/* 55 DST1 leap warning */
#define DST2		0x20	/* 2 DST2 DST1 delayed one day */
#define SECWAR		0x40	/* 3 leap second warning */

/*
 * The on-time synchronization point is the positive-going zero crossing
 * of the first cycle of the 5-ms second pulse. The IIR baseband filter
 * phase delay is 0.91 ms, while the receiver delay is approximately 4.7
 * ms at 1000 Hz. The fudge value -0.45 ms due to the codec and other
 * causes was determined by calibrating to a PPS signal from a GPS
 * receiver. The additional propagation delay specific to each receiver
 * location can be  programmed in the fudge time1 and time2 values for
 * WWV and WWVH, respectively.
 *
 * The resulting offsets with a 2.4-GHz P4 running FreeBSD 6.1 are
 * generally within .02 ms short-term with .02 ms jitter. The long-term
 * offsets vary up to 0.3 ms due to ionosperhic layer height variations.
 * The processor load due to the driver is 5.8 percent.
 */
#define PDELAY	((0.91f + 4.7f - 0.45f) / 1000) /* system delay (s) */

/*
 * Table of sine values at 4.5-degree increments. This is used by the
 * synchronous matched filter demodulators.
 */
float sintab[] = {
 0.000000e+00,  7.845910e-02,  1.564345e-01,  2.334454e-01, /* 0-3 */
 3.090170e-01,  3.826834e-01,  4.539905e-01,  5.224986e-01, /* 4-7 */
 5.877853e-01,  6.494480e-01,  7.071068e-01,  7.604060e-01, /* 8-11 */
 8.090170e-01,  8.526402e-01,  8.910065e-01,  9.238795e-01, /* 12-15 */
 9.510565e-01,  9.723699e-01,  9.876883e-01,  9.969173e-01, /* 16-19 */
 1.000000e+00,  9.969173e-01,  9.876883e-01,  9.723699e-01, /* 20-23 */
 9.510565e-01,  9.238795e-01,  8.910065e-01,  8.526402e-01, /* 24-27 */
 8.090170e-01,  7.604060e-01,  7.071068e-01,  6.494480e-01, /* 28-31 */
 5.877853e-01,  5.224986e-01,  4.539905e-01,  3.826834e-01, /* 32-35 */
 3.090170e-01,  2.334454e-01,  1.564345e-01,  7.845910e-02, /* 36-39 */
-0.000000e+00, -7.845910e-02, -1.564345e-01, -2.334454e-01, /* 40-43 */
-3.090170e-01, -3.826834e-01, -4.539905e-01, -5.224986e-01, /* 44-47 */
-5.877853e-01, -6.494480e-01, -7.071068e-01, -7.604060e-01, /* 48-51 */
-8.090170e-01, -8.526402e-01, -8.910065e-01, -9.238795e-01, /* 52-55 */
-9.510565e-01, -9.723699e-01, -9.876883e-01, -9.969173e-01, /* 56-59 */
-1.000000e+00, -9.969173e-01, -9.876883e-01, -9.723699e-01, /* 60-63 */
-9.510565e-01, -9.238795e-01, -8.910065e-01, -8.526402e-01, /* 64-67 */
-8.090170e-01, -7.604060e-01, -7.071068e-01, -6.494480e-01, /* 68-71 */
-5.877853e-01, -5.224986e-01, -4.539905e-01, -3.826834e-01, /* 72-75 */
-3.090170e-01, -2.334454e-01, -1.564345e-01, -7.845910e-02, /* 76-79 */
 0.000000e+00
};

/*
 * Decoder operations at the end of each second are driven by a state
 * machine. The transition matrix consists of a dispatch table indexed
 * by second number. Each entry in the table contains a case switch
 * number and argument.
 */
struct progx {
	unsigned char sw;            /* case switch number */
	unsigned char arg; /* argument */
};

/*
 * Case switch numbers
 */
#define IDLE		0	/* no operation */
#define COEF		1	/* BCD bit */
#define COEF1		2	/* BCD bit for minute unit */
#define COEF2		3	/* BCD bit not used */
#define DECIM9		4	/* BCD digit 0-9 */
#define DECIM6		5	/* BCD digit 0-6 */
#define DECIM3		6	/* BCD digit 0-3 */
#define DECIM2		7	/* BCD digit 0-2 */
#define MSCBIT		8	/* miscellaneous bit */
#define MSC20		9	/* miscellaneous bit */
#define MSC21		10	/* QSY probe channel */
#define MIN1		11	/* latch time */
#define MIN2		12	/* leap second */
#define SYNC2		13	/* latch minute sync pulse */
#define SYNC3		14	/* latch data pulse */

/*
 * Offsets in decoding matrix
 */
#define MN		0	/* minute digits (2) */
#define HR		2	/* hour digits (2) */
#define DA		4	/* day digits (3) */
#define YR		7	/* year digits (2) */

struct progx progx[] = {
	{SYNC2,	0},		/* 0 latch minute sync pulse */
	{SYNC3,	0},		/* 1 latch data pulse */
	{MSCBIT, DST2},		/* 2 dst2 */
	{MSCBIT, SECWAR},	/* 3 lw */
	{COEF,	0},		/* 4 1 year units */
	{COEF,	1},		/* 5 2 */
	{COEF,	2},		/* 6 4 */
	{COEF,	3},		/* 7 8 */
	{DECIM9, YR},		/* 8 */
	{IDLE,	0},		/* 9 p1 */
	{COEF1,	0},		/* 10 1 minute units */
	{COEF1,	1},		/* 11 2 */
	{COEF1,	2},		/* 12 4 */
	{COEF1,	3},		/* 13 8 */
	{DECIM9, MN},		/* 14 */
	{COEF,	0},		/* 15 10 minute tens */
	{COEF,	1},		/* 16 20 */
	{COEF,	2},		/* 17 40 */
	{COEF2,	3},		/* 18 80 (not used) */
	{DECIM6, MN + 1},	/* 19 p2 */
	{COEF,	0},		/* 20 1 hour units */
	{COEF,	1},		/* 21 2 */
	{COEF,	2},		/* 22 4 */
	{COEF,	3},		/* 23 8 */
	{DECIM9, HR},		/* 24 */
	{COEF,	0},		/* 25 10 hour tens */
	{COEF,	1},		/* 26 20 */
	{COEF2,	2},		/* 27 40 (not used) */
	{COEF2,	3},		/* 28 80 (not used) */
	{DECIM2, HR + 1},	/* 29 p3 */
	{COEF,	0},		/* 30 1 day units */
	{COEF,	1},		/* 31 2 */
	{COEF,	2},		/* 32 4 */
	{COEF,	3},		/* 33 8 */
	{DECIM9, DA},		/* 34 */
	{COEF,	0},		/* 35 10 day tens */
	{COEF,	1},		/* 36 20 */
	{COEF,	2},		/* 37 40 */
	{COEF,	3},		/* 38 80 */
	{DECIM9, DA + 1},	/* 39 p4 */
	{COEF,	0},		/* 40 100 day hundreds */
	{COEF,	1},		/* 41 200 */
	{COEF2,	2},		/* 42 400 (not used) */
	{COEF2,	3},		/* 43 800 (not used) */
	{DECIM3, DA + 2},	/* 44 */
	{IDLE,	0},		/* 45 */
	{IDLE,	0},		/* 46 */
	{IDLE,	0},		/* 47 */
	{IDLE,	0},		/* 48 */
	{IDLE,	0},		/* 49 p5 */
	{MSCBIT, DUTS},		/* 50 dut+- */
	{COEF,	0},		/* 51 10 year tens */
	{COEF,	1},		/* 52 20 */
	{COEF,	2},		/* 53 40 */
	{COEF,	3},		/* 54 80 */
	{MSC20, DST1},		/* 55 dst1 */
	{MSCBIT, DUT1},		/* 56 0.1 dut */
	{MSCBIT, DUT2},		/* 57 0.2 */
	{MSC21, DUT4},		/* 58 0.4 QSY probe channel */
	{MIN1,	0},		/* 59 p6 latch time */
	{MIN2,	0}		/* 60 leap second */
};

/*
 * BCD coefficients for maximum-likelihood digit decode
 */
#define P15	1.		/* max positive number */
#define N15	-1.		/* max negative number */

/*
 * Digits 0-9
 */
#define P9	(P15 / 4)	/* mark (+1) */
#define N9	(N15 / 4)	/* space (-1) */

float bcd9[][4] = {
	{N9, N9, N9, N9}, 	/* 0 */
	{P9, N9, N9, N9}, 	/* 1 */
	{N9, P9, N9, N9}, 	/* 2 */
	{P9, P9, N9, N9}, 	/* 3 */
	{N9, N9, P9, N9}, 	/* 4 */
	{P9, N9, P9, N9}, 	/* 5 */
	{N9, P9, P9, N9}, 	/* 6 */
	{P9, P9, P9, N9}, 	/* 7 */
	{N9, N9, N9, P9}, 	/* 8 */
	{P9, N9, N9, P9}, 	/* 9 */
	{0, 0, 0, 0}		/* backstop */
};

/*
 * Digits 0-6 (minute tens)
 */
#define P6	(P15 / 3)	/* mark (+1) */
#define N6	(N15 / 3)	/* space (-1) */

float bcd6[][4] = {
	{N6, N6, N6, 0}, 	/* 0 */
	{P6, N6, N6, 0}, 	/* 1 */
	{N6, P6, N6, 0}, 	/* 2 */
	{P6, P6, N6, 0}, 	/* 3 */
	{N6, N6, P6, 0}, 	/* 4 */
	{P6, N6, P6, 0}, 	/* 5 */
	{N6, P6, P6, 0}, 	/* 6 */
	{0, 0, 0, 0}		/* backstop */
};

/*
 * Digits 0-3 (day hundreds)
 */
#define P3	(P15 / 2)	/* mark (+1) */
#define N3	(N15 / 2)	/* space (-1) */

float bcd3[][4] = {
	{N3, N3, 0, 0}, 	/* 0 */
	{P3, N3, 0, 0}, 	/* 1 */
	{N3, P3, 0, 0}, 	/* 2 */
	{P3, P3, 0, 0}, 	/* 3 */
	{0, 0, 0, 0}		/* backstop */
};

/*
 * Digits 0-2 (hour tens)
 */
#define P2	(P15 / 2)	/* mark (+1) */
#define N2	(N15 / 2)	/* space (-1) */

float bcd2[][4] = {
	{N2, N2, 0, 0}, 	/* 0 */
	{P2, N2, 0, 0}, 	/* 1 */
	{N2, P2, 0, 0}, 	/* 2 */
	{0, 0, 0, 0}		/* backstop */
};

/*
 * DST decode (DST2 DST1) for prettyprint
 */
char dstcod[] = {
	'S',			/* 00 standard time */
	'I',			/* 01 set clock ahead at 0200 local */
	'O',			/* 10 set clock back at 0200 local */
	'D'			/* 11 daylight time */
};

/*
 * The decoding matrix consists of nine row vectors, one for each digit
 * of the timecode. The digits are stored from least to most significant
 * order. The maximum-likelihood timecode is formed from the digits
 * corresponding to the maximum-likelihood values reading in the
 * opposite order: yy ddd hh:mm.
 */
struct decvec {
	int radix;		/* radix (3, 4, 6, 10) */
	int digit;		/* current clock digit */
	int count;		/* match count */
	float digprb;		/* max digit probability */
	float digsnr;		/* likelihood function (dB) */
	float like[10];	/* likelihood integrator 0-9 */
};

/*
 * The station structure (sp) is used to acquire the minute pulse from
 * WWV and/or WWVH. These stations are distinguished by the frequency
 * used for the second and minute sync pulses, 1000 Hz for WWV and 1200
 * Hz for WWVH. Other than frequency, the format is the same.
 */
struct sync {
	double	epoch;		/* accumulated epoch differences */
	float	maxeng;		/* sync max energy */
	float	noieng;		/* sync noise energy */
	long	pos;		/* max amplitude position */
	long	lastpos;	/* last max position */
	long	mepoch;		/* minute synch epoch */

	float	amp;		/* sync signal */
	float	syneng;		/* sync signal max */
	float	synmax;		/* sync signal max latched at 0 s */
	float	synsnr;		/* sync signal SNR */
	float	metric;		/* signal quality metric */
	int	reach;		/* reachability register */
	int	count;		/* bit counter */
	int	select;		/* select bits */
	char	refid[5];	/* reference identifier */
};

/*
 * The channel structure (cp) is used to mitigate between channels.
 */
struct chan {
	int	gain;		/* audio gain */
	struct sync wwv;	/* wwv station */
	struct sync wwvh;	/* wwvh station */
};

/*
 * WWV unit control structure (up)
 */
struct wwvunit {
	const char *clockdesc;	/* clock description */
	l_fp	timestamp;	/* audio sample timestamp */
	l_fp	tick;		/* audio sample increment */
	float	phase, freq;	/* logical clock phase and frequency */
	float	monitor;	/* audio monitor point */
	float	pdelay;		/* propagation delay (s) */
	int	errflg;		/* error flags */
	int	watch;		/* watchcat */

	/*
	 * Audio codec variables
	 */
	int	gain;		/* codec gain */
	int	clipcnt;	/* sample clipped count */

	/*
	 * Variables used to establish basic system timing
	 */
	int	avgint;		/* master time constant */
	int	yepoch;		/* sync epoch */
	int	repoch;		/* buffered sync epoch */
	float	epomax;		/* second sync amplitude */
	float	eposnr;		/* second sync SNR */
	float	irig;		/* data I channel amplitude */
	float	qrig;		/* data Q channel amplitude */
	int	datapt;		/* 100 Hz ramp */
	float	datpha;		/* 100 Hz VFO control */
	int	rphase;		/* second sample counter */
	long	mphase;		/* minute sample counter */

	/*
	 * Variables used to mitigate which channel to use
	 */
	struct chan mitig[NCHAN]; /* channel data */
	struct sync *sptr;	/* station pointer */
	int	dchan;		/* data channel */
	int	schan;		/* probe channel */
	int	achan;		/* active channel */

	/*
	 * Variables used by the clock state machine
	 */
	struct decvec decvec[9]; /* decoding matrix */
	int	rsec;		/* seconds counter */
	int	digcnt;		/* count of digits synchronized */

	/*
	 * Variables used to estimate signal levels and bit/digit
	 * probabilities
	 */
	float	datsig;		/* data signal max */
	float	datsnr;		/* data signal SNR (dB) */

	/*
	 * Variables used to establish status and alarm conditions
	 */
	int	status;		/* status bits */
	int	alarm;		/* alarm flashers */
	int	misc;		/* miscellaneous timecode bits */
	int	errcnt;		/* data bit error counter */

    char a_lastcode[BMAX]; /* last timecode received */
    unsigned int lencode;    /* length of last timecode */

    struct calendar jt; /* year, yearday, month, monthday */
    uint8_t hour;       /* hour of day */
    uint8_t min;        /* minute of hour */
    uint8_t sec;        /* second of minute */
    uint32_t    yearstart;  /* beginning of year */

    /* Median filter */
    double filter[64]; /* median filter sample array */
    unsigned int coderecv;   /* put pointer */
    unsigned int codeproc;   /* get pointer */
    double jitter; /* jitter */
    double disp; /* dispersion */

    /*
     * Configuration data
     */
    float fudgetime1; /* fudge time1 */
    float fudgetime2; /* fudge time2 */

    /* NTP-SHM buffer */
    struct shmTime *shmTime;
};

/*
 * Function prototypes
 */

static void wwv_epoch(struct wwvunit *up);
void wwv_receive(struct wwvunit *up, int16_t *recv_buffer, unsigned int recv_length, l_fp recv_time);
void wwv_rf(struct wwvunit *up, float isig);
static void wwv_endpoc(struct wwvunit *up, int epopos);
static void wwv_rsec(struct wwvunit *up, float bit);
static void wwv_qrz(struct wwvunit *up, struct sync *sp, int pdelay);
static void wwv_corr4(struct wwvunit *up, struct decvec *vp, float	data[], float tab[][4]);
static void wwv_gain(struct wwvunit *up);
static void wwv_tsec(struct wwvunit *up);
static int timecode(struct wwvunit *, char *);
static int carry(struct decvec *);
static int wwv_newchan(struct wwvunit *up);
static void wwv_newgame(struct wwvunit *up);
static float wwv_metric(struct sync *);
static void wwv_clock(struct wwvunit *up);

static uint8_t qsy[NCHAN] = {5, 10, 15, 20}; /* frequencies (MHz) */

static inline uint32_t IsLeapYear(uint32_t Year)
{
    return (((Year % 4 == 0) && ((Year % 100 != 0) || (Year % 400 == 0))) ? 1 : 0);
}

typedef union _f32 {
    float f;
    uint32_t i;
} _f32;

static inline float to_dB(float in){
  _f32 tmp;
  tmp.f = in;
  tmp.i &= 0x7fffffff;
  return ((float)(tmp.i * 3.58855719e-7f - 382.3080943f));
}

/*
 * wwv_snr - compute SNR or likelihood function
 */
static float wwv_snr(float signal, float noise)
{
	float rval;

	/*
	 * This is a little tricky. Due to the way things are measured,
	 * either or both the signal or noise amplitude can be negative
	 * or zero. The intent is that, if the signal is negative or
	 * zero, the SNR must always be zero. This can happen with the
	 * subcarrier SNR before the phase has been aligned. On the
	 * other hand, in the likelihood function the "noise" is the
	 * next maximum down from the peak and this could be negative.
	 * However, in this case the SNR is truly stupendous, so we
	 * simply cap at MAXSNR dB (40).
	 */
	if (signal <= 0) {
		rval = 0;
	} else if (noise <= 0) {
		rval = MAXSNR;
	} else {
		//rval = 20.0f * (float)M_LOG10E * logf(signal / noise);
		//rval = 10.0f * (float)M_LOG10E * logf(signal / noise);
		rval = to_dB(signal / noise);
		if (rval > MAXSNR)
			rval = MAXSNR;
	}
	return (rval);
}

/*
 * wwv_start - open the devices and initialize data for processing
 */
struct wwvunit *wwv_start(int unit)
{
    struct wwvunit *up;

    /*
     * Allocate and initialize unit structure
     */
    if (!(up = (struct wwvunit *)malloc(sizeof(struct wwvunit)))) {
        return (0);
    }
	memset(up, 0, sizeof(struct wwvunit));

	/*
	 * Initialize miscellaneous variables
	 */
	up->clockdesc = DESCRIPTION;
	DTOLFP(1. / SECOND, &up->tick);

	/*
	 * Initialize the decoding matrix with the radix for each digit
	 * position.
	 */
	up->decvec[MN].radix = 10;	/* minutes */
	up->decvec[MN + 1].radix = 6;
	up->decvec[HR].radix = 10;	/* hours */
	up->decvec[HR + 1].radix = 3;
	up->decvec[DA].radix = 10;	/* days */
	up->decvec[DA + 1].radix = 10;
	up->decvec[DA + 2].radix = 4;
	up->decvec[YR].radix = 10;	/* years */
	up->decvec[YR + 1].radix = 10;

	/*
	 * Let the games begin.
	 */
	wwv_newgame(up);
	return up;
}

/*
 * wwv_shutdown - shut down the clock
 */
void wwv_shutdown(int unit, struct wwvunit *up)
{
    if (up) {
	    free(up);
    }
}

/*
 * wwv_receive - receive data from the audio device
 *
 * This routine reads input samples and adjusts the logical clock to
 * track the A/D sample clock by dropping or duplicating codec samples.
 * It also controls the A/D signal level with an AGC loop to mimimize
 * quantization noise and avoid overload.
 */
void wwv_receive(struct wwvunit *up, int16_t *recv_buffer, unsigned int recv_length, l_fp recv_time)
{
	/*
	 * Local variables
	 */
	float sample;		/* codec sample */
	uint32_t bufcnt;		/* buffer counter */
	l_fp	ltemp;

	/*
	 * Main loop - read until there ain't no more. Note codec
	 * samples are bit-inverted.
	 */
	DTOLFP((double)recv_length / SECOND, &ltemp);
	//L_SUB(&recv_time, &ltemp);
	up->timestamp = recv_time;
	for (bufcnt = 0; bufcnt < recv_length; bufcnt++) {
		sample = ((float)recv_buffer[bufcnt]);

		/*
		 * Clip noise spikes greater than MAXAMP (6000) and
		 * record the number of clips to be used later by the
		 * AGC.
		 */
		if (sample > MAXAMP) {
			sample = MAXAMP;
			up->clipcnt++;
		} else if (sample < -MAXAMP) {
			sample = -MAXAMP;
			up->clipcnt++;
		}

		/*
		 * Variable frequency oscillator. The codec oscillator
		 * runs at the nominal rate of 8000 samples per second,
		 * or 125 us per sample. A frequency change of one unit
		 * results in either duplicating or deleting one sample
		 * per second, which results in a frequency change of
		 * 125 PPM.
		 */
		up->phase += (up->freq + CLOCK_CODEC_OFFSET) / SECOND;
		if (up->phase >= .5) {
			up->phase -= 1.;
		} else if (up->phase < -.5) {
			up->phase += 1.;
			wwv_rf(up, sample);
			wwv_rf(up, sample);
		} else {
			wwv_rf(up, sample);
		}
		L_ADD(&up->timestamp, &up->tick);
	}
}

static void memzero_float(float *data, unsigned int len)
{
	unsigned int i;
	for (i = 0; i < (len/sizeof(float)); i++) {
		data[i] = 0.0f;
	}
}

/*
 * wwv_rf - process signals and demodulate to baseband
 *
 * This routine grooms and filters decompanded raw audio samples. The
 * output signal is the 100-Hz filtered baseband data signal in
 * quadrature phase. The routine also determines the minute synch epoch,
 * as well as certain signal maxima, minima and related values.
 *
 * There are two 1-s ramps used by this program. Both count the 8000
 * logical clock samples spanning exactly one second. The epoch ramp
 * counts the samples starting at an arbitrary time. The rphase ramp
 * counts the samples starting at the 5-ms second sync pulse found
 * during the epoch ramp.
 *
 * There are two 1-m ramps used by this program. The mphase ramp counts
 * the 480,000 logical clock samples spanning exactly one minute and
 * starting at an arbitrary time. The rsec ramp counts the 60 seconds of
 * the minute starting at the 800-ms minute sync pulse found during the
 * mphase ramp. The rsec ramp drives the seconds state machine to
 * determine the bits and digits of the timecode.
 *
 * Demodulation operations are based on three synthesized quadrature
 * sinusoids: 100 Hz for the data signal, 1000 Hz for the WWV sync
 * signal and 1200 Hz for the WWVH sync signal. These drive synchronous
 * matched filters for the data signal (170 ms at 100 Hz), WWV minute
 * sync signal (800 ms at 1000 Hz) and WWVH minute sync signal (800 ms
 * at 1200 Hz). Two additional matched filters are switched in
 * as required for the WWV second sync signal (5 cycles at 1000 Hz) and
 * WWVH second sync signal (6 cycles at 1200 Hz).
 */
void wwv_rf(struct wwvunit *up, float isig)
{
	struct sync *sp, *rp;

	static float lpf[5];	/* 150-Hz lpf delay line */
	float data;		/* lpf output */
	static float bpf[9];	/* 1000/1200-Hz bpf delay line */
	float syncx;		/* bpf output */
	static float mf[41];	/* 1000/1200-Hz mf delay line */
	float mfsync;		/* mf output */

	static int iptr;	/* data channel pointer */
	static float ibuf[DATSIZ]; /* data I channel delay line */
	static float qbuf[DATSIZ]; /* data Q channel delay line */

	static int jptr;	/* sync channel pointer */
	static int kptr;	/* tick channel pointer */

	static int csinptr;	/* wwv channel phase */
	static float cibuf[SYNSIZ]; /* wwv I channel delay line */
	static float cqbuf[SYNSIZ]; /* wwv Q channel delay line */
	static float ciamp;	/* wwv I channel amplitude */
	static float cqamp;	/* wwv Q channel amplitude */

	static float csibuf[TCKSIZ]; /* wwv I tick delay line */
	static float csqbuf[TCKSIZ]; /* wwv Q tick delay line */
	static float csiamp;	/* wwv I tick amplitude */
	static float csqamp;	/* wwv Q tick amplitude */

	static int hsinptr;	/* wwvh channel phase */
	static float hibuf[SYNSIZ]; /* wwvh I channel delay line */
	static float hqbuf[SYNSIZ]; /* wwvh Q channel delay line */
	static float hiamp;	/* wwvh I channel amplitude */
	static float hqamp;	/* wwvh Q channel amplitude */

	static float hsibuf[TCKSIZ]; /* wwvh I tick delay line */
	static float hsqbuf[TCKSIZ]; /* wwvh Q tick delay line */
	static float hsiamp;	/* wwvh I tick amplitude */
	static float hsqamp;	/* wwvh Q tick amplitude */

	static float epobuf[SECOND]; /* second sync comb filter */
	static float epomax, nxtmax; /* second sync amplitude buffer */
	static int epopos;	/* epoch second sync position buffer */

	static int iniflg;	/* initialization flag */
	int	epoch;		/* comb filter index */
	float	dtemp;
	int	i;

	if (!iniflg) {
		iniflg = 1;
		lpf[4] = lpf[3] = lpf[2] = lpf[1] = lpf[0] = 0.0f;
		bpf[8] = bpf[7] = bpf[6] = bpf[5] = bpf[4] = bpf[3] = bpf[2] = bpf[1] = bpf[0] = 0.0f;
		memzero_float(mf, sizeof(mf));
		memzero_float(ibuf, sizeof(ibuf));
		memzero_float(qbuf, sizeof(qbuf));
		memzero_float(cibuf, sizeof(cibuf));
		memzero_float(cqbuf, sizeof(cqbuf));
		memzero_float(csibuf, sizeof(csibuf));
		memzero_float(csqbuf, sizeof(csqbuf));
		memzero_float(hibuf, sizeof(hibuf));
		memzero_float(hqbuf, sizeof(hqbuf));
		memzero_float(hsibuf, sizeof(hsibuf));
		memzero_float(hsqbuf, sizeof(hsqbuf));
		memzero_float(epobuf, sizeof(epobuf));
	}

	/*
	 * Baseband data demodulation. The 100-Hz subcarrier is
	 * extracted using a 150-Hz IIR lowpass filter. This attenuates
	 * the 1000/1200-Hz sync signals, as well as the 440-Hz and
	 * 600-Hz tones and most of the noise and voice modulation
	 * components.
	 *
	 * The subcarrier is transmitted 10 dB down from the carrier.
	 * The DGAIN parameter can be adjusted for this and to
	 * compensate for the radio audio response at 100 Hz.
	 *
	 * Matlab IIR 4th-order IIR elliptic, 150 Hz lowpass, 0.2 dB
	 * passband ripple, -50 dB stopband ripple, phase delay 0.97 ms.
	 */
	data  = (lpf[4] = lpf[3]) *  0.8360961f;
	data += (lpf[3] = lpf[2]) * -3.481740f;
	data += (lpf[2] = lpf[1]) *  5.452988f;
	data += (lpf[1] = lpf[0]) * -3.807229f;
	lpf[0] = isig * DGAIN - data;
	data = (lpf[0] + lpf[4]) * 3.281435e-03f - (lpf[1] + lpf[3]) * 1.149947e-02f + lpf[2] * 1.654858e-02f;

	/*
	 * The 100-Hz data signal is demodulated using a pair of
	 * quadrature multipliers, matched filters and a phase lock
	 * loop. The I and Q quadrature data signals are produced by
	 * multiplying the filtered signal by 100-Hz sine and cosine
	 * signals, respectively. The signals are processed by 170-ms
	 * synchronous matched filters to produce the amplitude and
	 * phase signals used by the demodulator. The signals are scaled
	 * to produce unit energy at the maximum value.
	 */
	i = up->datapt;
	up->datapt = (up->datapt + IN100) % 80;
	dtemp = sintab[i] * data / (MS / 2. * DATCYC);
	up->irig -= ibuf[iptr];
	ibuf[iptr] = dtemp;
	up->irig += dtemp;

	i = (i + 20) % 80;
	dtemp = sintab[i] * data / (MS / 2. * DATCYC);
	up->qrig -= qbuf[iptr];
	qbuf[iptr] = dtemp;
	up->qrig += dtemp;
	iptr = (iptr + 1) % DATSIZ;

	/*
	 * Baseband sync demodulation. The 1000/1200 sync signals are
	 * extracted using a 600-Hz IIR bandpass filter. This removes
	 * the 100-Hz data subcarrier, as well as the 440-Hz and 600-Hz
	 * tones and most of the noise and voice modulation components.
	 *
	 * Matlab 4th-order IIR elliptic, 800-1400 Hz bandpass, 0.2 dB
	 * passband ripple, -50 dB stopband ripple, phase delay 0.91 ms.
	 */
	syncx = (bpf[8] = bpf[7]) * 0.4897278f;
	syncx += (bpf[7] = bpf[6]) * -2.765914f;
	syncx += (bpf[6] = bpf[5]) * 8.110921f;
	syncx += (bpf[5] = bpf[4]) * -15.17732f;
	syncx += (bpf[4] = bpf[3]) * 19.75197f;
	syncx += (bpf[3] = bpf[2]) * -18.14365f;
	syncx += (bpf[2] = bpf[1]) * 11.59783f;
	syncx += (bpf[1] = bpf[0]) * -4.735040f;
	bpf[0] = isig - syncx;
	syncx = (bpf[0] + bpf[8]) * 8.203628e-03
	      + (bpf[1] + bpf[7]) * -2.375732e-02
	      + (bpf[2] + bpf[6]) * 3.353214e-02
	      + (bpf[3] + bpf[5]) * -4.080258e-02
	      +  bpf[4] * 4.605479e-02;

	/*
	 * The 1000/1200 sync signals are demodulated using a pair of
	 * quadrature multipliers and matched filters. However,
	 * synchronous demodulation at these frequencies is impractical,
	 * so only the signal amplitude is used. The I and Q quadrature
	 * sync signals are produced by multiplying the filtered signal
	 * by 1000-Hz (WWV) and 1200-Hz (WWVH) sine and cosine signals,
	 * respectively. The WWV and WWVH signals are processed by 800-
	 * ms synchronous matched filters and combined to produce the
	 * minute sync signal and detect which one (or both) the WWV or
	 * WWVH signal is present. The WWV and WWVH signals are also
	 * processed by 5-ms synchronous matched filters and combined to
	 * produce the second sync signal. The signals are scaled to
	 * produce unit energy at the maximum value.
	 *
	 * Note the master timing ramps, which run continuously. The
	 * minute counter (mphase) counts the samples in the minute,
	 * while the second counter (epoch) counts the samples in the
	 * second.
	 */
	up->mphase = (up->mphase + 1) % MINUTE;
	epoch = up->mphase % SECOND;

	/*
	 * WWV
	 */
	i = csinptr;
	csinptr = (csinptr + IN1000) % 80;

	dtemp = sintab[i] * syncx / (MS / 2.);
	ciamp -= cibuf[jptr];
	cibuf[jptr] = dtemp;
	ciamp += dtemp;
	csiamp -= csibuf[kptr];
	csibuf[kptr] = dtemp;
	csiamp += dtemp;

	i = (i + 20) % 80;
	dtemp = sintab[i] * syncx / (MS / 2.);
	cqamp -= cqbuf[jptr];
	cqbuf[jptr] = dtemp;
	cqamp += dtemp;
	csqamp -= csqbuf[kptr];
	csqbuf[kptr] = dtemp;
	csqamp += dtemp;

	sp = &up->mitig[up->achan].wwv;
	sp->amp = sqrtf(ciamp * ciamp + cqamp * cqamp) / SYNCYC;
	if (!(up->status & MSYNC))
		wwv_qrz(up, sp, (int)(up->fudgetime1 * SECOND));

	/*
	 * WWVH
	 */
	i = hsinptr;
	hsinptr = (hsinptr + IN1200) % 80;

	dtemp = sintab[i] * syncx / (MS / 2.);
	hiamp -= hibuf[jptr];
	hibuf[jptr] = dtemp;
	hiamp += dtemp;
	hsiamp -= hsibuf[kptr];
	hsibuf[kptr] = dtemp;
	hsiamp += dtemp;

	i = (i + 20) % 80;
	dtemp = sintab[i] * syncx / (MS / 2.);
	hqamp -= hqbuf[jptr];
	hqbuf[jptr] = dtemp;
	hqamp += dtemp;
	hsqamp -= hsqbuf[kptr];
	hsqbuf[kptr] = dtemp;
	hsqamp += dtemp;

	rp = &up->mitig[up->achan].wwvh;
	rp->amp = sqrtf(hiamp * hiamp + hqamp * hqamp) / SYNCYC;
	if (!(up->status & MSYNC))
		wwv_qrz(up, rp, (int)(up->fudgetime2 * SECOND));
	jptr = (jptr + 1) % SYNSIZ;
	kptr = (kptr + 1) % TCKSIZ;

	/*
	 * The following section is called once per minute. It does
	 * housekeeping and timeout functions and empties the dustbins.
	 */
	if (up->mphase == 0) {
		up->watch++;
		if (!(up->status & MSYNC)) {
			/*
			 * If minute sync has not been acquired before
			 * ACQSN timeout (6 min), or if no signal is
			 * heard, the program cycles to the next
			 * frequency and tries again.
			 */
			if (!wwv_newchan(up))
				up->watch = 0;
		}
	}

	/*
	 * When the channel metric reaches threshold and the second
	 * counter matches the minute epoch within the second, the
	 * driver has synchronized to the station. The second number is
	 * the remaining seconds until the next minute epoch, while the
	 * sync epoch is zero. Watch out for the first second; if
	 * already synchronized to the second, the buffered sync epoch
	 * must be set.
	 *
	 * Note the guard interval is 200 ms; if for some reason the
	 * clock drifts more than that, it might wind up in the wrong
	 * second. If the maximum frequency error is not more than about
	 * 1 PPM, the clock can go as much as two days while still in
	 * the same second.
	 */
	if (up->status & MSYNC) {
		wwv_epoch(up);
	} else if (up->sptr != NULL) {
		sp = up->sptr;
		if (sp->metric >= TTHR && epoch == sp->mepoch % SECOND)
 		    {
			up->rsec = (60 - sp->mepoch / SECOND) % 60;
			up->rphase = 0;
			up->status |= MSYNC;
			up->watch = 0;
			if (!(up->status & SSYNC))
				up->repoch = up->yepoch = epoch;
			else
				up->repoch = up->yepoch;

		}
	}

	/*
	 * The second sync pulse is extracted using 5-ms (40 sample) FIR
	 * matched filters at 1000 Hz for WWV or 1200 Hz for WWVH. This
	 * pulse is used for the most precise synchronization, since if
	 * provides a resolution of one sample (125 us). The filters run
	 * only if the station has been reliably determined.
	 */
	if (up->status & SELV)
		mfsync = sqrtf(csiamp * csiamp + csqamp * csqamp) / TCKCYC;
	else if (up->status & SELH)
		mfsync = sqrtf(hsiamp * hsiamp + hsqamp * hsqamp) / TCKCYC;
	else
		mfsync = 0;

	/*
	 * Enhance the seconds sync pulse using a 1-s (8000-sample) comb
	 * filter. Correct for the FIR matched filter delay, which is 5
	 * ms for both the WWV and WWVH filters, and also for the
	 * propagation delay. Once each second look for second sync. If
	 * not in minute sync, fiddle the codec gain. Note the SNR is
	 * computed from the maximum sample and the envelope of the
	 * sample 6 ms before it, so if we slip more than a cycle the
	 * SNR should plummet. The signal is scaled to produce unit
	 * energy at the maximum value.
	 */
	dtemp = (epobuf[epoch] += (mfsync - epobuf[epoch]) / up->avgint);
	if (dtemp > epomax) {
		int	j;

		epomax = dtemp;
		epopos = epoch;
		j = epoch - 6 * MS;
		if (j < 0)
			j += SECOND;
		nxtmax = fabsf(epobuf[j]);
	}
	if (epoch == 0) {
		up->epomax = epomax;
		up->eposnr = wwv_snr(epomax, nxtmax);
		epopos -= TCKCYC * MS;
		if (epopos < 0)
			epopos += SECOND;
		wwv_endpoc(up, epopos);
		if (!(up->status & SSYNC))
			up->alarm |= SYNERR;
		epomax = 0;
		if (!(up->status & MSYNC))
			wwv_gain(up);
	}
}

/*
 * wwv_qrz - identify and acquire WWV/WWVH minute sync pulse
 *
 * This routine implements a virtual station process used to acquire
 * minute sync and to mitigate among the ten frequency and station
 * combinations. During minute sync acquisition the process probes each
 * frequency and station in turn for the minute pulse, which
 * involves searching through the entire 480,000-sample minute. The
 * process finds the maximum signal and RMS noise plus signal. Then, the
 * actual noise is determined by subtracting the energy of the matched
 * filter.
 *
 * Students of radar receiver technology will discover this algorithm
 * amounts to a range-gate discriminator. A valid pulse must have peak
 * amplitude at least QTHR (2500) and SNR at least QSNR (20) dB and the
 * difference between the current and previous epoch must be less than
 * AWND (20 ms). Note that the discriminator peak occurs about 800 ms
 * into the second, so the timing is retarded to the previous second
 * epoch.
 */
void wwv_qrz(struct wwvunit *up,
	struct sync *sp,	/* sync channel structure */
	int	pdelay		/* propagation delay (samples) */
	)
{
	char tbuf[TBUF];	/* monitor buffer */
	int tbuf_len;
	long	epoch;

	/*
	 * Find the sample with peak amplitude, which defines the minute
	 * epoch. Accumulate all samples to determine the total noise
	 * energy.
	 */
	epoch = up->mphase - pdelay - SYNSIZ;
	if (epoch < 0)
		epoch += MINUTE;
	if (sp->amp > sp->maxeng) {
		sp->maxeng = sp->amp;
		sp->pos = epoch;
	}
	sp->noieng += sp->amp;

	/*
	 * At the end of the minute, determine the epoch of the minute
	 * sync pulse, as well as the difference between the current and
	 * previous epoches due to the intrinsic frequency error plus
	 * jitter. When calculating the SNR, subtract the pulse energy
	 * from the total noise energy and then normalize.
	 */
	if (up->mphase == 0) {
		sp->synmax = sp->maxeng;
		sp->synsnr = wwv_snr(sp->synmax, (sp->noieng - sp->synmax) / MINUTE);
		if (sp->count == 0)
			sp->lastpos = sp->pos;
		epoch = (sp->pos - sp->lastpos) % MINUTE;
		sp->reach <<= 1;
		if (sp->reach & (1 << AMAX))
			sp->count--;
		if (sp->synmax > ATHR && sp->synsnr > ASNR) {
			if (abs(epoch) < AWND * MS) {
				sp->reach |= 1;
				sp->count++;
				sp->mepoch = sp->lastpos = sp->pos;
			} else if (sp->count == 1) {
				sp->lastpos = sp->pos;
			}
		}
		if (up->watch > ACQSN)
			sp->metric = 0;
		else
			sp->metric = wwv_metric(sp);
		{
        	tbuf_len = snprintf(tbuf, TBUF-1, "wwv8 %04x %3d %s %04x %.0f %.0f/%.1f %ld %ld\n",
                                up->status, up->gain, sp->refid, sp->reach & 0xffff, sp->metric,
							    sp->synmax, sp->synsnr, sp->pos % SECOND, epoch);
        	write(2, tbuf, tbuf_len);
		}
		sp->maxeng = sp->noieng = 0;
	}
}


/*
 * wwv_endpoc - identify and acquire second sync pulse
 *
 * This routine is called at the end of the second sync interval. It
 * determines the second sync epoch position within the second and
 * disciplines the sample clock using a frequency-lock loop (FLL).
 *
 * Second sync is determined in the RF input routine as the maximum
 * over all 8000 samples in the second comb filter. To assure accurate
 * and reliable time and frequency discipline, this routine performs a
 * great deal of heavy-handed heuristic data filtering and grooming.
 */
static void wwv_endpoc(struct wwvunit *up, int epopos)
{
	static int epoch_mf[3]; /* epoch median filter */
	static int tepoch;	/* current second epoch */
 	static int xepoch;	/* last second epoch */
 	static int zepoch;	/* last run epoch */
	static int zcount;	/* last run end time */
	static int scount;	/* seconds counter */
	static int syncnt;	/* run length counter */
	static int maxrun;	/* longest run length */
	static int mepoch;	/* longest run end epoch */
	static int mcount;	/* longest run end time */
	static int avgcnt;	/* averaging interval counter */
	static int avginc;	/* averaging ratchet */
	static int iniflg;	/* initialization flag */
	char tbuf[TBUF];		/* monitor buffer */
	float dtemp;
	int tmp2;

	if (!iniflg) {
		iniflg = 1;
		memset((char *)epoch_mf, 0, sizeof(epoch_mf));
	}

	/*
	 * If the signal amplitude or SNR fall below thresholds, dim the
	 * second sync lamp and wait for hotter ions. If no stations are
	 * heard, we are either in a probe cycle or the ions are really
	 * cold.
	 */
	scount++;
	if (up->epomax < STHR || up->eposnr < SSNR) {
		up->status &= ~(SSYNC | FGATE);
		avgcnt = syncnt = maxrun = 0;
		return;
	}
	if (!(up->status & (SELV | SELH)))
		return;

	/*
	 * A three-stage median filter is used to help denoise the
	 * second sync pulse. The median sample becomes the candidate
	 * epoch.
	 */
	epoch_mf[2] = epoch_mf[1];
	epoch_mf[1] = epoch_mf[0];
	epoch_mf[0] = epopos;
	if (epoch_mf[0] > epoch_mf[1]) {
		if (epoch_mf[1] > epoch_mf[2])
			tepoch = epoch_mf[1];	/* 0 1 2 */
		else if (epoch_mf[2] > epoch_mf[0])
			tepoch = epoch_mf[0];	/* 2 0 1 */
		else
			tepoch = epoch_mf[2];	/* 0 2 1 */
	} else {
		if (epoch_mf[1] < epoch_mf[2])
			tepoch = epoch_mf[1];	/* 2 1 0 */
		else if (epoch_mf[2] < epoch_mf[0])
			tepoch = epoch_mf[0];	/* 1 0 2 */
		else
			tepoch = epoch_mf[2];	/* 1 2 0 */
	}


	/*
	 * If the epoch candidate is the same as the last one, increment
	 * the run counter. If not, save the length, epoch and end
	 * time of the current run for use later and reset the counter.
	 * The epoch is considered valid if the run is at least SCMP
	 * (10) s, the minute is synchronized and the interval since the
	 * last epoch  is not greater than the averaging interval. Thus,
	 * after a long absence, the program will wait a full averaging
	 * interval while the comb filter charges up and noise
	 * dissapates..
	 */
	tmp2 = (tepoch - xepoch) % SECOND;
	if (tmp2 == 0) {
		syncnt++;
		if (syncnt > SCMP && up->status & MSYNC && (up->status &
		    FGATE || scount - zcount <= up->avgint)) {
			up->status |= SSYNC;
			up->yepoch = tepoch;
		}
	} else if (syncnt >= maxrun) {
		maxrun = syncnt;
		mcount = scount;
		mepoch = xepoch;
		syncnt = 0;
	}
	if (!(up->status & MSYNC)) {
		int tbuf_len = snprintf(tbuf, TBUF-1, "wwv1 %04x %3d %4d %5.0f %5.1f %5d %4d %4d %4d\n",
		    up->status, up->gain, tepoch, up->epomax,
		    up->eposnr, tmp2, avgcnt, syncnt,
		    maxrun);
		write(1, tbuf, tbuf_len);
	}
	avgcnt++;
	if (avgcnt < up->avgint) {
		xepoch = tepoch;
		return;
	}

	/*
	 * The sample clock frequency is disciplined using a first-order
	 * feedback loop with time constant consistent with the Allan
	 * intercept of typical computer clocks. During each averaging
	 * interval the candidate epoch at the end of the longest run is
	 * determined. If the longest run is zero, all epoches in the
	 * interval are different, so the candidate epoch is the current
	 * epoch. The frequency update is computed from the candidate
	 * epoch difference (125-us units) and time difference (seconds)
	 * between updates.
	 */
	if (syncnt >= maxrun) {
		maxrun = syncnt;
		mcount = scount;
		mepoch = xepoch;
	}
	xepoch = tepoch;
	if (maxrun == 0) {
		mepoch = tepoch;
		mcount = scount;
	}

	/*
	 * The master clock runs at the codec sample frequency of 8000
	 * Hz, so the intrinsic time resolution is 125 us. The frequency
	 * resolution ranges from 18 PPM at the minimum averaging
	 * interval of 8 s to 0.12 PPM at the maximum interval of 1024
	 * s. An offset update is determined at the end of the longest
	 * run in each averaging interval. The frequency adjustment is
	 * computed from the difference between offset updates and the
	 * interval between them.
	 *
	 * The maximum frequency adjustment ranges from 187 PPM at the
	 * minimum interval to 1.5 PPM at the maximum. If the adjustment
	 * exceeds the maximum, the update is discarded and the
	 * hysteresis counter is decremented. Otherwise, the frequency
	 * is incremented by the adjustment, but clamped to the maximum
	 * 187.5 PPM. If the update is less than half the maximum, the
	 * hysteresis counter is incremented. If the counter increments
	 * to +3, the averaging interval is doubled and the counter set
	 * to zero; if it decrements to -3, the interval is halved and
	 * the counter set to zero.
	 */
	dtemp = (mepoch - zepoch) % SECOND;
	if (up->status & FGATE) {
		if (abs(dtemp) < MAXFREQ * MINAVG) {
			up->freq += (dtemp / 2.) / ((mcount - zcount) *
			    FCONST);
			if (up->freq > MAXFREQ)
				up->freq = MAXFREQ;
			else if (up->freq < -MAXFREQ)
				up->freq = -MAXFREQ;
			if (abs(dtemp) < MAXFREQ * MINAVG / 2.) {
				if (avginc < 3) {
					avginc++;
				} else {
					if (up->avgint < MAXAVG) {
						up->avgint <<= 1;
						avginc = 0;
					}
				}
			}
		} else {
			if (avginc > -3) {
				avginc--;
			} else {
				if (up->avgint > MINAVG) {
					up->avgint >>= 1;
					avginc = 0;
				}
			}
		}
	}
	{
		int tbuf_len = snprintf(tbuf, TBUF-1,
		    "wwv2 %04x %5.0f %5.1f %5d %4d %4d %4d %4.0f %7.2f\n",
		    up->status, up->epomax, up->eposnr, mepoch,
		    up->avgint, maxrun, mcount - zcount, dtemp,
		    up->freq * 1e6 / SECOND);
		write(1, tbuf, tbuf_len);
	}

	/*
	 * This is a valid update; set up for the next interval.
	 */
	up->status |= FGATE;
	zepoch = mepoch;
	zcount = mcount;
	avgcnt = syncnt = maxrun = 0;
}


/*
 * wwv_epoch - epoch scanner
 *
 * This routine extracts data signals from the 100-Hz subcarrier. It
 * scans the receiver second epoch to determine the signal amplitudes
 * and pulse timings. Receiver synchronization is determined by the
 * minute sync pulse detected in the wwv_rf() routine and the second
 * sync pulse detected in the wwv_epoch() routine. The transmitted
 * signals are delayed by the propagation delay, receiver delay and
 * filter delay of this program. Delay corrections are introduced
 * separately for WWV and WWVH.
 *
 * Most communications radios use a highpass filter in the audio stages,
 * which can do nasty things to the subcarrier phase relative to the
 * sync pulses. Therefore, the data subcarrier reference phase is
 * disciplined using the hardlimited quadrature-phase signal sampled at
 * the same time as the in-phase signal. The phase tracking loop uses
 * phase adjustments of plus-minus one sample (125 us).
 */
static void
wwv_epoch(struct wwvunit *up)
{
	struct chan *cp;
	static float sigmin, sigzer, sigone, engmax, engmin;

	/*
	 * Find the maximum minute sync pulse energy for both the
	 * WWV and WWVH stations. This will be used later for channel
	 * and station mitigation. Also set the seconds epoch at 800 ms
	 * well before the end of the second to make sure we never set
	 * the epoch backwards.
	 */
	cp = &up->mitig[up->achan];
	if (cp->wwv.amp > cp->wwv.syneng)
		cp->wwv.syneng = cp->wwv.amp;
	if (cp->wwvh.amp > cp->wwvh.syneng)
		cp->wwvh.syneng = cp->wwvh.amp;
	if (up->rphase == 800 * MS)
		up->repoch = up->yepoch;

	/*
	 * Use the signal amplitude at epoch 15 ms as the noise floor.
	 * This gives a guard time of +-15 ms from the beginning of the
	 * second until the second pulse rises at 30 ms. There is a
	 * compromise here; we want to delay the sample as long as
	 * possible to give the radio time to change frequency and the
	 * AGC to stabilize, but as early as possible if the second
	 * epoch is not exact.
	 */
	if (up->rphase == 15 * MS)
		sigmin = sigzer = sigone = up->irig;

	/*
	 * Latch the data signal at 200 ms. Keep this around until the
	 * end of the second. Use the signal energy as the peak to
	 * compute the SNR. Use the Q sample to adjust the 100-Hz
	 * reference oscillator phase.
	 */
	if (up->rphase == 200 * MS) {
		sigzer = up->irig;
		engmax = sqrtf(up->irig * up->irig + up->qrig * up->qrig);
		up->datpha = up->qrig / up->avgint;
		if (up->datpha >= 0) {
			up->datapt++;
			if (up->datapt >= 80)
				up->datapt -= 80;
		} else {
			up->datapt--;
			if (up->datapt < 0)
				up->datapt += 80;
		}
	}


	/*
	 * Latch the data signal at 500 ms. Keep this around until the
	 * end of the second.
	 */
	else if (up->rphase == 500 * MS)
		sigone = up->irig;

	/*
	 * At the end of the second crank the clock state machine and
	 * adjust the codec gain. Note the epoch is buffered from the
	 * center of the second in order to avoid jitter while the
	 * seconds synch is diddling the epoch. Then, determine the true
	 * offset and update the median filter in the driver interface.
	 *
	 * Use the energy at the end of the second as the noise to
	 * compute the SNR for the data pulse. This gives a better
	 * measurement than the beginning of the second, especially when
	 * returning from the probe channel. This gives a guard time of
	 * 30 ms from the decay of the longest pulse to the rise of the
	 * next pulse.
	 */
	up->rphase++;
	if (up->mphase % SECOND == up->repoch) {
		up->status &= ~(DGATE | BGATE);
		engmin = sqrtf(up->irig * up->irig + up->qrig * up->qrig);
		up->datsig = engmax;
		up->datsnr = wwv_snr(engmax, engmin);

		/*
		 * If the amplitude or SNR is below threshold, average a
		 * 0 in the the integrators; otherwise, average the
		 * bipolar signal. This is done to avoid noise polution.
		 */
		if (engmax < DTHR || up->datsnr < DSNR) {
			up->status |= DGATE;
			wwv_rsec(up, 0);
		} else {
			sigzer -= sigone;
			sigone -= sigmin;
			wwv_rsec(up, sigone - sigzer);
		}
		if (up->status & (DGATE | BGATE))
			up->errcnt++;
		if (up->errcnt > MAXERR)
			up->alarm |= LOWERR;
		wwv_gain(up);
		cp = &up->mitig[up->achan];
		cp->wwv.syneng = 0;
		cp->wwvh.syneng = 0;
		up->rphase = 0;
	}
}


/*
 * wwv_rsec - process receiver second
 *
 * This routine is called at the end of each receiver second to
 * implement the per-second state machine. The machine assembles BCD
 * digit bits, decodes miscellaneous bits and dances the leap seconds.
 *
 * Normally, the minute has 60 seconds numbered 0-59. If the leap
 * warning bit is set, the last minute (1439) of 30 June (day 181 or 182
 * for leap years) or 31 December (day 365 or 366 for leap years) is
 * augmented by one second numbered 60. This is accomplished by
 * extending the minute interval by one second and teaching the state
 * machine to ignore it.
 */
static void wwv_rsec(struct wwvunit *up, float bit)
{
	static int iniflg;	/* initialization flag */
	static float bcddld[4]; /* BCD data bits */
	static float bitvec[61]; /* bit integrator for misc bits */
	struct chan *cp;
	struct sync *sp, *rp;
	char	tbuf[TBUF];	/* monitor buffer */
	int	sw, arg, nsec;

	if (!iniflg) {
		iniflg = 1;
		memset((char *)bitvec, 0, sizeof(bitvec));
	}

	/*
	 * The bit represents the probability of a hit on zero (negative
	 * values), a hit on one (positive values) or a miss (zero
	 * value). The likelihood vector is the exponential average of
	 * these probabilities. Only the bits of this vector
	 * corresponding to the miscellaneous bits of the timecode are
	 * used, but it's easier to do them all. After that, crank the
	 * seconds state machine.
	 */
	nsec = up->rsec;
	up->rsec++;
	bitvec[nsec] += (bit - bitvec[nsec]) / TCONST;
	sw = progx[nsec].sw;
	arg = progx[nsec].arg;

	/*
	 * The minute state machine. Fly off to a particular section as
	 * directed by the transition matrix and second number.
	 */
	switch (sw) {

	/*
	 * Ignore this second.
	 */
	case IDLE:			/* 9, 45-49 */
		break;

	/*
	 * Probe channel stuff
	 *
	 * The WWV/H format contains data pulses in second 59 (position
	 * identifier) and second 1, but not in second 0. The minute
	 * sync pulse is contained in second 0. At the end of second 58
	 * QSY to the probe channel, which rotates in turn over all
	 * WWV/H frequencies. At the end of second 0 measure the minute
	 * sync pulse. At the end of second 1 measure the data pulse and
	 * QSY back to the data channel. Note that the actions commented
	 * here happen at the end of the second numbered as shown.
	 *
	 * At the end of second 0 save the minute sync amplitude latched
	 * at 800 ms as the signal later used to calculate the SNR.
	 */
	case SYNC2:			/* 0 */
		cp = &up->mitig[up->achan];
		cp->wwv.synmax = cp->wwv.syneng;
		cp->wwvh.synmax = cp->wwvh.syneng;
		break;

	/*
	 * At the end of second 1 use the minute sync amplitude latched
	 * at 800 ms as the noise to calculate the SNR. If the minute
	 * sync pulse and SNR are above thresholds and the data pulse
	 * amplitude and SNR are above thresolds, shift a 1 into the
	 * station reachability register; otherwise, shift a 0. The
	 * number of 1 bits in the last six intervals is a component of
	 * the channel metric computed by the wwv_metric() routine.
	 * Finally, QSY back to the data channel.
	 */
	case SYNC3:			/* 1 */
		cp = &up->mitig[up->achan];

		/*
		 * WWV station
		 */
		sp = &cp->wwv;
		sp->synsnr = wwv_snr(sp->synmax, sp->amp);
		sp->reach <<= 1;
		if (sp->reach & (1 << AMAX))
			sp->count--;
		if (sp->synmax >= QTHR && sp->synsnr >= QSNR &&
		    !(up->status & (DGATE | BGATE))) {
			sp->reach |= 1;
			sp->count++;
		}
		sp->metric = wwv_metric(sp);

		/*
		 * WWVH station
		 */
		rp = &cp->wwvh;
		rp->synsnr = wwv_snr(rp->synmax, rp->amp);
		rp->reach <<= 1;
		if (rp->reach & (1 << AMAX))
			rp->count--;
		if (rp->synmax >= QTHR && rp->synsnr >= QSNR &&
		    !(up->status & (DGATE | BGATE))) {
			rp->reach |= 1;
			rp->count++;
		}
		rp->metric = wwv_metric(rp);
#if 0
        {
			int tbuf_len = snprintf(tbuf, TBUF-1,
			    "wwv5 %04x %3d %4d %.0f/%.1f %.0f/%.1f %s %04x %.0f %.0f/%.1f %s %04x %.0f %.0f/%.1f\n",
			    up->status, up->gain, up->yepoch,
			    up->epomax, up->eposnr, up->datsig,
			    up->datsnr,
			    sp->refid, sp->reach & 0xffff,
			    sp->metric, sp->synmax, sp->synsnr,
			    rp->refid, rp->reach & 0xffff,
			    rp->metric, rp->synmax, rp->synsnr);
			write(1, tbuf, tbuf_len);
		}
#endif
		up->errcnt = up->digcnt = up->alarm = 0;

		/*
		 * If synchronized to a station, restart if no stations
		 * have been heard within the PANIC timeout (2 days). If
		 * not and the minute digit has been found, restart if
		 * not synchronized withing the SYNCH timeout (40 m). If
		 * not, restart if the unit digit has not been found
		 * within the DATA timeout (15 m).
		 */
		if (up->status & INSYNC) {
			if (up->watch > PANIC) {
				wwv_newgame(up);
				return;
			}
		} else if (up->status & DSYNC) {
			if (up->watch > SYNCH) {
				wwv_newgame(up);
				return;
			}
		} else if (up->watch > DATA) {
			wwv_newgame(up);
			return;
		}
		wwv_newchan(up);
		break;

	/*
	 * Save the bit probability in the BCD data vector at the index
	 * given by the argument. Bits not used in the digit are forced
	 * to zero.
	 */
	case COEF1:			/* 4-7 */
		bcddld[arg] = bit;
		break;

	case COEF:			/* 10-13, 15-17, 20-23, 25-26,
					   30-33, 35-38, 40-41, 51-54 */
		if (up->status & DSYNC)
			bcddld[arg] = bit;
		else
			bcddld[arg] = 0;
		break;

	case COEF2:			/* 18, 27-28, 42-43 */
		bcddld[arg] = 0;
		break;

	/*
	 * Correlate coefficient vector with each valid digit vector and
	 * save in decoding matrix. We step through the decoding matrix
	 * digits correlating each with the coefficients and saving the
	 * greatest and the next lower for later SNR calculation.
	 */
	case DECIM2:			/* 29 */
		wwv_corr4(up, &up->decvec[arg], bcddld, bcd2);
		break;

	case DECIM3:			/* 44 */
		wwv_corr4(up, &up->decvec[arg], bcddld, bcd3);
		break;

	case DECIM6:			/* 19 */
		wwv_corr4(up, &up->decvec[arg], bcddld, bcd6);
		break;

	case DECIM9:			/* 8, 14, 24, 34, 39 */
		wwv_corr4(up, &up->decvec[arg], bcddld, bcd9);
		break;

	/*
	 * Miscellaneous bits. If above the positive threshold, declare
	 * 1; if below the negative threshold, declare 0; otherwise
	 * raise the BGATE bit. The design is intended to avoid
	 * integrating noise under low SNR conditions.
	 */
	case MSC20:			/* 55 */
		wwv_corr4(up, &up->decvec[YR + 1], bcddld, bcd9);
		/* fall through */

	case MSCBIT:			/* 2-3, 50, 56-57 */
		if (bitvec[nsec] > BTHR) {
			if (!(up->misc & arg))
				up->alarm |= CMPERR;
			up->misc |= arg;
		} else if (bitvec[nsec] < -BTHR) {
			if (up->misc & arg)
				up->alarm |= CMPERR;
			up->misc &= ~arg;
		} else {
			up->status |= BGATE;
		}
		break;

	/*
	 * Save the data channel gain, then QSY to the probe channel and
	 * dim the seconds comb filters. The www_newchan() routine will
	 * light them back up.
	 */
	case MSC21:			/* 58 */
		if (bitvec[nsec] > BTHR) {
			if (!(up->misc & arg))
				up->alarm |= CMPERR;
			up->misc |= arg;
		} else if (bitvec[nsec] < -BTHR) {
			if (up->misc & arg)
				up->alarm |= CMPERR;
			up->misc &= ~arg;
		} else {
			up->status |= BGATE;
		}
		up->status &= ~(SELV | SELH);
		up->mitig[up->achan].gain = up->gain;
		break;

	/*
	 * The endgames
	 *
	 * During second 59 the receiver and codec AGC are settling
	 * down, so the data pulse is unusable as quality metric. If
	 * LEPSEC is set on the last minute of 30 June or 31 December,
	 * the transmitter and receiver insert an extra second (60) in
	 * the timescale and the minute sync repeats the second. Once
	 * leaps occurred at intervals of about 18 months, but the last
	 * leap before the most recent leap in 1995 was in  1998.
	 */
	case MIN1:			/* 59 */
		if (up->status & LEPSEC)
			break;

		/* fall through */

	case MIN2:			/* 60 */
		up->status &= ~LEPSEC;
		wwv_tsec(up);
		up->rsec = 0;
		wwv_clock(up);
		break;
	}
	if (!(up->status & DSYNC)) {
		int tbuf_len = snprintf(tbuf, TBUF-1, "wwv3 %2d %04x %3d %4d %5.0f %5.1f %5.0f %5.1f %5.0f\n",
		    nsec, up->status, up->gain, up->yepoch, up->epomax,
		    up->eposnr, up->datsig, up->datsnr, bit);
		write(1, tbuf, tbuf_len);
		//record_clock_stats(&peer->srcadr, tbuf);
	}
	up->disp += AUDIO_PHI;
}

/*
 * The radio clock is set if the alarm bits are all zero. After that,
 * the time is considered valid if the second sync bit is lit. It should
 * not be a surprise, especially if the radio is not tunable, that
 * sometimes no stations are above the noise and the integrators
 * discharge below the thresholds. We assume that, after a day of signal
 * loss, the minute sync epoch will be in the same second. This requires
 * the codec frequency be accurate within 6 PPM. Practical experience
 * shows the frequency typically within 0.1 PPM, so after a day of
 * signal loss, the time should be within 8.6 ms..
 */

/* put a received fix time into shared memory for NTP */
void ntp_write(volatile struct shmTime *shmseg, time_t tv_sec, l_fp *rtv, int precision)
{
    shmseg->mode = 0;
    shmseg->valid = 0;
    /* We need a memory barrier here to prevent write reordering by
     * the compiler or CPU cache */
    __asm__ __volatile__("sfence \n\t":::"memory");
    shmseg->ct_Sec = tv_sec;
    shmseg->ct_USec = 0;;
    shmseg->rt_Sec = (time_t)rtv->l_ui;
    shmseg->rt_Sec -= JAN_1970;
    TSFTOTVU(rtv->l_uf, shmseg->rt_USec);
    if (shmseg->rt_USec >= 1000000) {
        shmseg->rt_Sec++;
        shmseg->rt_USec = 0;
    }
    shmseg->leap = 0;
    shmseg->precision = -precision;
    __asm__ __volatile__("":::"memory");
    shmseg->valid = 1;
}

void Shellsort_dbl(double *in, unsigned int n)
{
  int i, j;
  double v;
  unsigned int inc = 1;

  do {
    inc = 3 * inc + 1;
  } while (inc <= n);

  do {
    inc = inc / 3;
    for (i = inc + 1; i <= n; i++) {
      v = in[i-1];
      j = i;
      while (in[j-inc-1] > v) {
        in[j-1] = in[j-inc-1];
        j -= inc;
        if (j <= inc)
          break;
      }
      in[j-1] = v;
    }
  } while (inc > 1);
}

/*
 * wwv_process_offset - update median filter
 *
 * This routine uses the given offset and timestamps to construct a new
 * entry in the median filter circular buffer. Samples that overflow the
 * filter are quietly discarded.
 */
/* lasttim: last timecode timestamp */
/* lastrec: last receive timestamp */
void wwv_process_offset(struct wwvunit *up, time_t lasttim)
{
	l_fp lftemp;
	double doffset;

	lftemp.l_ui = lasttim;
    lftemp.l_uf = 0;
	L_SUB(&lftemp, &up->timestamp);
	LFPTOD(&lftemp, doffset);
    up->coderecv = (up->coderecv + 1) & 63;
    up->filter[up->coderecv] = (doffset + PDELAY + up->pdelay);
    if (up->coderecv == up->codeproc)
        up->codeproc = (up->codeproc + 1) & 63;
}

unsigned int wwv_sample(struct wwvunit *up)
{
	unsigned int i, j, k, m, n = 0;
	double	off[64];
	double	offset, offs2;

	/*
	 * Copy the raw offsets and sort into ascending order. Don't do
	 * anything if the buffer is empty.
	 */
	while (up->codeproc != up->coderecv) {
		up->codeproc = (up->codeproc + 1) & 63;
		off[n++] = up->filter[up->codeproc];
	}
	if (!n) return (0);

    Shellsort_dbl(off, n);

	/*
	 * Reject the furthest from the median of the samples until
	 * approximately 60 percent of the samples remain.
	 */
	i = 0; j = n;
	m = n - (n * 2) / 5;
	while ((j - i) > m) {
		offset = off[(j + i) >> 1];
		if (off[j - 1] - offset < offset - off[i]) i++;	/* reject low end */
		else j--;	/* reject high end */
	}

	/* Determine the offset and jitter. */
	offs2 = 0;
	up->jitter = 0;
	for (k = i; k < j; k++) {
		offs2 += off[k];
		if (k > i) {
            double tmp = (off[k] - off[k - 1]);
			up->jitter += tmp*tmp;
        }
	}
	offs2 /= m;
	//up->jitter = sqrt(1.0 / up->jitter);
    up->jitter *= m;
	up->jitter = m*sqrt(1.0 / up->jitter);
	{
	    char tbuf[TBUF];	/* monitor buffer */
       	int tbuf_len = snprintf(tbuf, TBUF-1, "refclock_sample: n: %u offset: %.6f disp: %.6f jitter: %.6f\n",
                                n, offs2, up->disp, 1.0/up->jitter);
        write(2, tbuf, tbuf_len);
	}
	return (unsigned int)n;
}

static inline unsigned int av_log2(unsigned int x) {
    unsigned int ret;
    __asm__ volatile("or $1, %1 \n\t"
             "bsr %1, %0 \n\t"
             :"=a"(ret) : "D"(x));
    return ret;
}

static void wwv_clock(struct wwvunit *up)
{
    unsigned int hms = 0;
	time_t offset; /* offset in NTP seconds */

	if (!(up->status & SSYNC))
		up->alarm |= SYNERR;
	if (up->digcnt < 9)
		up->alarm |= NINERR;
	if (!(up->alarm))
		up->status |= INSYNC;
	if (up->status & INSYNC && up->status & SSYNC) {
		up->sec = up->rsec;
		up->min = up->decvec[MN].digit + up->decvec[MN + 1].digit * 10;
		up->hour = up->decvec[HR].digit + up->decvec[HR + 1].digit * 10;
		up->jt.yearday = up->decvec[DA].digit + up->decvec[DA + 1].digit * 10 + up->decvec[DA + 2].digit * 100;
		up->jt.year = up->decvec[YR].digit + up->decvec[YR + 1].digit * 10;
		up->jt.year += 2000;
        hms = ((3600 * up->hour) + (60 * up->min) + up->sec);
		up->yearstart = calyearstart(up->timestamp.l_ui);
	    offset = (int32_t)86400*(up->jt.yearday-1);
        {
	        char tbuf[TBUF];	/* monitor buffer */
       	    int tbuf_len = snprintf(tbuf, TBUF-1, "offset: %lu\n", offset);
            write(2, tbuf, tbuf_len);
        }
        offset += (int32_t)hms;
        //offset += (int32_t)up->yearstart;
		up->watch = 0;
		up->disp = 0;
		wwv_process_offset(up, offset);
		//wwv_process_offset(up, hms);
        if (!(hms & 7)) {
            unsigned int n = 0, codeproc = up->codeproc;
	        while (codeproc != up->coderecv) {
		        codeproc = (codeproc + 1) & 63;
                n++;
	        }
            if (n >= 4) {
                if (wwv_sample(up)) {
                    l_fp curtime;
                    get_systime(&curtime);
                    ntp_write(up->shmTime, offset, &curtime, av_log2((unsigned int)up->jitter));
                }
            }
        }
	}
	up->lencode = timecode(up, up->a_lastcode);
}

/*
 * wwv_corr4 - determine maximum-likelihood digit
 *
 * This routine correlates the received digit vector with the BCD
 * coefficient vectors corresponding to all valid digits at the given
 * position in the decoding matrix. The maximum value corresponds to the
 * maximum-likelihood digit, while the ratio of this value to the next
 * lower value determines the likelihood function. Note that, if the
 * digit is invalid, the likelihood vector is averaged toward a miss.
 */
static void
wwv_corr4(struct wwvunit *up,
	struct decvec *vp,	/* decoding table pointer */
	float	data[],		/* received data vector */
	float	tab[][4]	/* correlation vector array */
	)
{
	float	topmax, nxtmax;	/* metrics */
	float	acc;		/* accumulator */
	char	tbuf[TBUF];	/* monitor buffer */
	int	mldigit;	/* max likelihood digit */
	int	i, j;

	/*
	 * Correlate digit vector with each BCD coefficient vector. If
	 * any BCD digit bit is bad, consider all bits a miss. Until the
	 * minute units digit has been resolved, don't to anything else.
	 * Note the SNR is calculated as the ratio of the largest
	 * likelihood value to the next largest likelihood value.
 	 */
	mldigit = 0;
	topmax = nxtmax = -MAXAMP;
	for (i = 0; tab[i][0] != 0; i++) {
		acc = 0;
		for (j = 0; j < 4; j++)
			acc += data[j] * tab[i][j];
		acc = (vp->like[i] += (acc - vp->like[i]) / TCONST);
		if (acc > topmax) {
			nxtmax = topmax;
			topmax = acc;
			mldigit = i;
		} else if (acc > nxtmax) {
			nxtmax = acc;
		}
	}
	vp->digprb = topmax;
	vp->digsnr = wwv_snr(topmax, nxtmax);

	/*
	 * The current maximum-likelihood digit is compared to the last
	 * maximum-likelihood digit. If different, the compare counter
	 * and maximum-likelihood digit are reset.  When the compare
	 * counter reaches the BCMP threshold (3), the digit is assumed
	 * correct. When the compare counter of all nine digits have
	 * reached threshold, the clock is assumed correct.
	 *
	 * Note that the clock display digit is set before the compare
	 * counter has reached threshold; however, the clock display is
	 * not considered correct until all nine clock digits have
	 * reached threshold. This is intended as eye candy, but avoids
	 * mistakes when the signal is low and the SNR is very marginal.
	 */
	if (vp->digprb < BTHR || vp->digsnr < BSNR) {
		up->status |= BGATE;
	} else {
		if (vp->digit != mldigit) {
			up->alarm |= CMPERR;
			if (vp->count > 0)
				vp->count--;
			if (vp->count == 0)
				vp->digit = mldigit;
		} else {
			if (vp->count < BCMP)
				vp->count++;
			if (vp->count == BCMP) {
				up->status |= DSYNC;
				up->digcnt++;
			}
		}
	}
	if (!(up->status & INSYNC)) {
		int tbuf_len = snprintf(tbuf, TBUF-1,
		    "wwv4 %2d %04x %3d %4d %5.0f %2d %d %d %d %5.0f %5.1f\n",
		    up->rsec - 1, up->status, up->gain, up->yepoch,
		    up->epomax, vp->radix, vp->digit, mldigit,
		    vp->count, vp->digprb, vp->digsnr);
		write(1, tbuf, tbuf_len);
	}
}

/*
 * carry - process digit
 *
 * This routine rotates a likelihood vector one position and increments
 * the clock digit modulo the radix. It returns the new clock digit or
 * zero if a carry occurred. Once synchronized, the clock digit will
 * match the maximum-likelihood digit corresponding to that position.
 */
static int carry(struct decvec *dp) /* decoding table pointer */
{
	int temp;
	int j;

	dp->digit++;
	if (dp->digit == dp->radix)
		dp->digit = 0;
	temp = dp->like[dp->radix - 1];
	for (j = dp->radix - 1; j > 0; j--)
		dp->like[j] = dp->like[j - 1];
	dp->like[0] = temp;
	return (dp->digit);
}

/*
 * wwv_tsec - transmitter minute processing
 *
 * This routine is called at the end of the transmitter minute. It
 * implements a state machine that advances the logical clock subject to
 * the funny rules that govern the conventional clock and calendar.
 */
static void
wwv_tsec(struct wwvunit *up)
{
	int minute, day, isleap;
	int temp;

	/*
	 * Advance minute unit of the day. Don't propagate carries until
	 * the unit minute digit has been found.
	 */
	temp = carry(&up->decvec[MN]);	/* minute units */
	if (!(up->status & DSYNC))
		return;

	/*
	 * Propagate carries through the day.
	 */
	if (temp == 0)			/* carry minutes */
		temp = carry(&up->decvec[MN + 1]);
	if (temp == 0)			/* carry hours */
		temp = carry(&up->decvec[HR]);
	if (temp == 0)
		temp = carry(&up->decvec[HR + 1]);

	/*
	 * Decode the current minute and day. Set leap day if the
	 * timecode leap bit is set on 30 June or 31 December. Set leap
	 * minute if the last minute on leap day, but only if the clock
	 * is syncrhronized. This code fails in 2400 AD.
	 */
	minute = up->decvec[MN].digit + up->decvec[MN + 1].digit *
	    10 + up->decvec[HR].digit * 60 + up->decvec[HR +
	    1].digit * 600;
	day = up->decvec[DA].digit + up->decvec[DA + 1].digit * 10 +
	    up->decvec[DA + 2].digit * 100;

	/*
	 * Roll the day if this the first minute and propagate carries
	 * through the year.
	 */
	if (minute != 1440)
		return;

	minute = 0;
	while (carry(&up->decvec[HR]) != 0); /* advance to minute 0 */
	while (carry(&up->decvec[HR + 1]) != 0);
	day++;
	temp = carry(&up->decvec[DA]);	/* carry days */
	if (temp == 0)
		temp = carry(&up->decvec[DA + 1]);
	if (temp == 0)
		temp = carry(&up->decvec[DA + 2]);

	/*
	 * Roll the year if this the first day and propagate carries
	 * through the century.
	 */
	if (day != (isleap ? 365 : 366))
		return;

	day = 1;
	while (carry(&up->decvec[DA]) != 1); /* advance to day 1 */
	while (carry(&up->decvec[DA + 1]) != 0);
	while (carry(&up->decvec[DA + 2]) != 0);
	temp = carry(&up->decvec[YR]);	/* carry years */
	if (temp == 0)
		carry(&up->decvec[YR + 1]);
}

/*
 * wwv_newchan - change to new data channel
 *
 * The radio actually appears to have ten channels, one channel for each
 * of five frequencies and each of two stations (WWV and WWVH), although
 * if not tunable only the DCHAN channel appears live. While the radio
 * is tuned to the working data channel frequency and station for most
 * of the minute, during seconds 59, 0 and 1 the radio is tuned to a
 * probe frequency in order to search for minute sync pulse and data
 * subcarrier from other transmitters.
 *
 * The search for WWV and WWVH operates simultaneously, with WWV minute
 * sync pulse at 1000 Hz and WWVH at 1200 Hz. The probe frequency
 * rotates each minute over 2.5, 5, 10, 15 and 20 MHz in order and yes,
 * we all know WWVH is dark on 20 MHz, but few remember when WWV was lit
 * on 25 MHz.
 *
 * This routine selects the best channel using a metric computed from
 * the reachability register and minute pulse amplitude. Normally, the
 * award goes to the the channel with the highest metric; but, in case
 * of ties, the award goes to the channel with the highest minute sync
 * pulse amplitude and then to the highest frequency.
 *
 * The routine performs an important squelch function to keep dirty data
 * from polluting the integrators. In order to consider a station valid,
 * the metric must be at least MTHR (13); otherwise, the station select
 * bits are cleared so the second sync is disabled and the data bit
 * integrators averaged to a miss.
 */
static int
wwv_newchan(struct wwvunit *up)
{
	struct sync *sp, *rp;
	float rank, dtemp;
	int i, j, rval;

	/*
	 * Search all four station pairs looking for the channel with maximum metric.
	 */
	sp = NULL;
	j = 0;
	rank = 0;
	for (i = 0; i < NCHAN; i++) {
		rp = &up->mitig[i].wwvh;
		dtemp = rp->metric;
		if (dtemp >= rank) {
			rank = dtemp;
			sp = rp;
			j = i;
		}
		rp = &up->mitig[i].wwv;
		dtemp = rp->metric;
		if (dtemp >= rank) {
			rank = dtemp;
			sp = rp;
			j = i;
		}
	}

	/*
	 * If the strongest signal is less than the MTHR threshold (13),
	 * we are beneath the waves, so squelch the second sync and
	 * advance to the next station. This makes sure all stations are
	 * scanned when the ions grow dim. If the strongest signal is
	 * greater than the threshold, tune to that frequency and
	 * transmitter QTH.
	 */
	up->status &= ~(SELV | SELH);
	if (rank < MTHR) {
		up->dchan = (up->dchan + 1) % NCHAN;
		if (up->status & METRIC) {
			up->status &= ~METRIC;
		}
		rval = FALSE;
	} else {
		up->dchan = j;
		up->sptr = sp;
		//memcpy(&pp->refid, sp->refid, 4);
		up->status |= METRIC;
		if (sp->select & SELV) {
			up->status |= SELV;
			up->pdelay = up->fudgetime1;
		} else if (sp->select & SELH) {
			up->status |= SELH;
			up->pdelay = up->fudgetime2;
		} else {
			up->pdelay = 0;
		}
		rval = TRUE;
	}
	return (rval);
}

/*
 * wwv_newgame - reset and start over
 *
 * There are three conditions resulting in a new game:
 *
 * 1	After finding the minute pulse (MSYNC lit), going 15 minutes
 *	(DATA) without finding the unit seconds digit.
 *
 * 2	After finding good data (DSYNC lit), going more than 40 minutes
 *	(SYNCH) without finding station sync (INSYNC lit).
 *
 * 3	After finding station sync (INSYNC lit), going more than 2 days
 *	(PANIC) without finding any station.
 */
static void
wwv_newgame(struct wwvunit *up)
{
	struct chan *cp;
	int i;

	/*
	 * Initialize strategic values. Note we set the leap bits
	 * NOTINSYNC and the refid "NONE".
	 */
	if (up->status)
		up->errflg = 1; // Connection Timed Out
	//peer->leap = LEAP_NOTINSYNC;
	up->watch = up->status = up->alarm = 0;
	up->avgint = MINAVG;
	up->freq = 0;
	up->gain = MAXGAIN / 2;

	/*
	 * Initialize the station processes for audio gain, select bit,
	 * station/frequency identifier and reference identifier. Start
	 * probing at the strongest channel or the default channel if
	 * nothing heard.
	 */
	memset(up->mitig, 0, sizeof(up->mitig));
	for (i = 0; i < NCHAN; i++) {
		cp = &up->mitig[i];
		cp->gain = up->gain;
		cp->wwv.select = SELV;
		snprintf(cp->wwv.refid, 5, "WV%.0f", floorf(qsy[i]));
		cp->wwvh.select = SELH;
		snprintf(cp->wwvh.refid, 5, "WH%.0f", floorf(qsy[i]));
	}
	up->dchan = (DCHAN + NCHAN - 1) % NCHAN;
	wwv_newchan(up);
	up->schan = up->dchan;
}

/*
 * wwv_metric - compute station metric
 *
 * The most significant bits represent the number of ones in the
 * station reachability register. The least significant bits represent
 * the minute sync pulse amplitude. The combined value is scaled 0-100.
 */
float wwv_metric(struct sync *sp)
{
	float dtemp = sp->count * MAXAMP;
	if (sp->synmax < MAXAMP)
		dtemp += sp->synmax;
	else
		dtemp += MAXAMP - 1;
	dtemp /= (AMAX + 1) * MAXAMP;
	return (dtemp * 100.0f);
}

/*
 * wwv_gain - adjust codec gain
 *
 * This routine is called at the end of each second. During the second
 * the number of signal clips above the MAXAMP threshold (6000). If
 * there are no clips, the gain is bumped up; if there are more than
 * MAXCLP clips (100), it is bumped down. The decoder is relatively
 * insensitive to amplitude, so this crudity works just peachy.
 */
static void wwv_gain(struct wwvunit *up)
{
	/*
	 * Apparently, the codec uses only the high order bits of the
	 * gain control field. Thus, it may take awhile for changes to
	 * wiggle the hardware bits.
	 */
	if (up->clipcnt == 0) {
		up->gain += 4;
		if (up->gain > MAXGAIN)
			up->gain = MAXGAIN;
	} else if (up->clipcnt > MAXCLP) {
		up->gain -= 4;
		if (up->gain < 0)
			up->gain = 0;
	}
	up->clipcnt = 0;
}

void d2md(unsigned int day, unsigned int isleap, unsigned int *m, unsigned int *d)
{
	/*
	 * The following code is according to the excellent book
	 * 'Calendrical Calculations' by Nachum Dershowitz and Edward
	 * Reingold. It converts the day-of-year into month and
	 * day-of-month, using a linear transformation with integer truncation.
	 */
	unsigned int sclday = day * 7 + 217;
	if (day >= (uint32_t)(31 + 28 + isleap))
		sclday += (2 - isleap) * 7;
	*m = (uint8_t)((sclday / 214));
	*d = (uint8_t)((sclday % 214) / 7 + 1);
}

/*
 * timecode - assemble timecode string and length
 *
 * Prettytime format - similar to Spectracom
 *
 * sq yy ddd hh:mm:ss ld dut lset agc iden sig errs freq avgt
 *
 * s    sync indicator ('?' or ' ')
 * q    error bits (hex 0-F)
 * yyyy year of century
 * ddd  day of year
 * hh   hour of day
 * mm   minute of hour
 * ss   second of minute)
 * d    DST state ('S', 'D', 'I', or 'O')
 * lset minutes since last clock update
 * agc  audio gain (0-255)
 * iden reference identifier (station and frequency)
 * sig  signal quality (0-100)
 * errs bit errors in last minute
 * freq frequency offset (PPM)
 * avgt averaging time (s)
 */
static int timecode(struct wwvunit *up, char *ptr)
{
    struct sync *sp;
    unsigned int year, day, month, mday, hour, minute, second, isleap;
    char synchar, dst;
    char cptr[50];
	unsigned int cptr_len = 0;

    /*
     * Common fixed-format fields
     */
    synchar = (up->status & INSYNC) ? ' ' : '?';
    year = up->decvec[YR].digit + up->decvec[YR + 1].digit * 10 + 2000;
    day = up->decvec[DA].digit + up->decvec[DA + 1].digit * 10 + up->decvec[DA + 2].digit * 100;
    hour = up->decvec[HR].digit + up->decvec[HR + 1].digit * 10;
    minute = up->decvec[MN].digit + up->decvec[MN + 1].digit * 10;
    second = 0;
    dst = dstcod[(up->misc >> 4) & 0x3];
    ptr[0] = synchar;
    ptr[1] = '\0';
    isleap = IsLeapYear(year);
    d2md(day, isleap, &month, &mday);
    cptr_len = snprintf(cptr, 48, "| %1X %4d %02d %02d %02u:%02u:%02u %c",
			            up->alarm, year, month, mday, hour, minute, second, dst);

    /*
     * Specific variable-format fields
     */
    sp = up->sptr;
    cptr_len += snprintf(cptr + cptr_len-1, 48, " | %d %d %s %.0f %d %.1f %d |\n",
                         up->watch, up->mitig[up->dchan].gain, sp->refid, sp->metric,
                         up->errcnt, up->freq / SECOND * 1e6, up->avgint);
	write(1, cptr, cptr_len);
	memcpy(ptr, cptr, cptr_len);
	ptr[cptr_len] = '\0';
	return cptr_len;
}

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

struct shmTime *getShmTime (unsigned int unit)
{
    int shmid=shmget (0x4e545030+unit, sizeof (struct shmTime), IPC_CREAT|0666);
    if (shmid==-1) {
        return 0;
    }
    else {
        struct shmTime *p=(struct shmTime *)shmat (shmid, 0, 0);
        if ((ssize_t)p==-1) {
            p=0;
        }
        return p;
    }
}

uint8_t *slurp_file(char *path, size_t *filesize);
#if 0
int main(int argc, char **argv) {
    size_t bufsize;
    unsigned int i = 0, frames;
    int16_t *buf;
    unsigned char *_buf = slurp_file(argv[1], &bufsize);
    struct wwvunit *up = wwv_start(2);
    l_fp l_curtime;
    up->shmTime = getShmTime(3);
    _buf += 44; bufsize -= 44;
    buf = (int16_t*)_buf;
    frames = bufsize/16000;
    while(i < frames) {
        get_systime(&l_curtime);
        wwv_receive(up, buf+i*8000, 8000, l_curtime);
        i++;
    }
    return 0;
}
#endif

int main(int argc, char **argv) {
    int in_fd = -1;
    unsigned int i = 0;
    int16_t buf[48000];
    struct wwvunit *up = wwv_start(2);
    l_fp l_curtime;
    if ((in_fd = open(argv[1], O_RDONLY)) < 0) {
        return -1;
    }
    up->shmTime = getShmTime(3);
    while(1) {
        if (read(in_fd, buf, 8000*sizeof(int16_t)) < 0) break;
        get_systime(&l_curtime);
        wwv_receive(up, buf, 8000, l_curtime);
        i++;
    }
    close(in_fd);
    return 0;
}
