/*
 * refclock_chu - clock driver for Canadian CHU time/frequency station
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
#define CLOCK_CODEC_OFFSET 0
#define MAXGAIN 16383
#define	LEAP_NOWARNING	0x0	/* normal */
#define	LEAP_NOTINSYNC	0x3	/* overload, clock is free running */

#ifndef TRUE
# define TRUE 1
#endif /* TRUE */
#ifndef FALSE
# define FALSE 0
#endif /* FALSE */

/*
 * Audio CHU demodulator/decoder
 *
 * This driver synchronizes the computer time using data encoded in
 * radio transmissions from Canadian time/frequency station CHU in
 * Ottawa, Ontario. Transmissions are made continuously on 3330 kHz,
 * 7850 kHz and 14670 kHz in upper sideband, compatible AM mode. An
 * ordinary shortwave receiver can be tuned manually to one of these
 * frequencies or, in the case of ICOM receivers, the receiver can be
 * tuned automatically as propagation conditions change throughout the
 * day and season.
 *
 * The driver requires an audio codec or sound card with sampling rate 8
 * kHz and mu-law companding. This is the same standard as used by the
 * telephone industry and is supported by most hardware and operating
 * systems, including Solaris, SunOS, FreeBSD, NetBSD and Linux. In this
 * implementation, only one audio driver and codec can be supported on a
 * single machine.
 *
 * The driver can be compiled to use a Bell 103 compatible modem or
 * modem chip to receive the radio signal and demodulate the data.
 * Alternatively, the driver can be compiled to use the audio codec of
 * the workstation or another with compatible audio drivers. In the
 * latter case, the driver implements the modem using DSP routines, so
 * the radio can be connected directly to either the microphone on line
 * input port. In either case, the driver decodes the data using a
 * maximum-likelihood technique which exploits the considerable degree
 * of redundancy available to maximize accuracy and minimize errors.
 *
 * The CHU time broadcast includes an audio signal compatible with the
 * Bell 103 modem standard (mark = 2225 Hz, space = 2025 Hz). The signal
 * consists of nine, ten-character bursts transmitted at 300 bps between
 * seconds 31 and 39 of each minute. Each character consists of eight
 * data bits plus one start bit and two stop bits to encode two hex
 * digits. The burst data consist of five characters (ten hex digits)
 * followed by a repeat of these characters. In format A, the characters
 * are repeated in the same polarity; in format B, the characters are
 * repeated in the opposite polarity.
 *
 * Format A bursts are sent at seconds 32 through 39 of the minute in
 * hex digits (nibble swapped)
 *
 *	6dddhhmmss6dddhhmmss
 *
 * The first ten digits encode a frame marker (6) followed by the day
 * (ddd), hour (hh in UTC), minute (mm) and the second (ss). Since
 * format A bursts are sent during the third decade of seconds the tens
 * digit of ss is always 3. The driver uses this to determine correct
 * burst synchronization. These digits are then repeated with the same
 * polarity.
 *
 * Format B bursts are sent at second 31 of the minute in hex digits
 *
 *	xdyyyyttaaxdyyyyttaa
 *
 * The first ten digits encode a code (x described below) followed by
 * the DUT1 (d in deciseconds), Gregorian year (yyyy), difference TAI -
 * UTC (tt) and daylight time indicator (aa) peculiar to Canada. These
 * digits are then repeated with inverted polarity.
 *
 * The x is coded
 *
 * 1 Sign of DUT (0 = +)
 * 2 Leap second warning. One second will be added.
 * 4 Leap second warning. One second will be subtracted.
 * 8 Even parity bit for this nibble.
 *
 * By design, the last stop bit of the last character in the burst
 * coincides with 0.5 second. Since characters have 11 bits and are
 * transmitted at 300 bps, the last stop bit of the first character
 * coincides with 0.5 - 9 * 11/300 = 0.170 second. Depending on the
 * UART, character interrupts can vary somewhere between the end of bit
 * 9 and end of bit 11. These eccentricities can be corrected along with
 * the radio propagation delay using fudge time 1.
 *
 * Debugging aids
 *
 * The timecode format used for debugging and data recording includes
 * data helpful in diagnosing problems with the radio signal and serial
 * connections. With debugging enabled (-d on the ntpd command line),
 * the driver produces one line for each burst in two formats
 * corresponding to format A and B.Each line begins with the format code
 * chuA or chuB followed by the status code and signal level (0-9999).
 * The remainder of the line is as follows.
 *
 * Following is format A:
 *
 *	n b f s m code
 *
 * where n is the number of characters in the burst (0-10), b the burst
 * distance (0-40), f the field alignment (-1, 0, 1), s the
 * synchronization distance (0-16), m the burst number (2-9) and code
 * the burst characters as received. Note that the hex digits in each
 * character are reversed, so the burst
 *
 *	10 38 0 16 9 06851292930685129293
 *
 * is interpreted as containing 10 characters with burst distance 38,
 * field alignment 0, synchronization distance 16 and burst number 9.
 * The nibble-swapped timecode shows day 58, hour 21, minute 29 and
 * second 39.
 *
 * Following is format B:
 *
 *	n b s code
 *
 * where n is the number of characters in the burst (0-10), b the burst
 * distance (0-40), s the synchronization distance (0-40) and code the
 * burst characters as received. Note that the hex digits in each
 * character are reversed and the last ten digits inverted, so the burst
 *
 *	10 40 1091891300ef6e76ec
 *
 * is interpreted as containing 10 characters with burst distance 40.
 * The nibble-swapped timecode shows DUT1 +0.1 second, year 1998 and TAI
 * - UTC 31 seconds.
 *
 * Each line is preceeded by the code chuA or chuB, as appropriate. If
 * the audio driver is compiled, the current gain (0-255) and relative
 * signal level (0-9999) follow the code. The receiver volume control
 * should be set so that the gain is somewhere near the middle of the
 * range 0-255, which results in a signal level near 1000.
 *
 * In addition to the above, the reference timecode is updated and
 * written to the clockstats file and debug score after the last burst
 * received in the minute. The format is
 *
 *	sq yyyy ddd hh:mm:ss l s dd t agc ident m b
 *
 * s	'?' before first synchronized and ' ' after that
 * q	status code (see below)
 * yyyy	year
 * ddd	day of year
 * hh:mm:ss time of day
 * l	leap second indicator (space, L or D)
 * dst	Canadian daylight code (opaque)
 * t	number of minutes since last synchronized
 * agc	audio gain (0 - 255)
 * ident identifier (CHU0 3330 kHz, CHU1 7850 kHz, CHU2 14670 kHz)
 * m	signal metric (0 - 100)
 * b	number of timecodes for the previous minute (0 - 59)
 *
 * Fudge factors
 *
 * For accuracies better than the low millisceconds, fudge time1 can be
 * set to the radio propagation delay from CHU to the receiver. This can
 * be done conviently using the minimuf program.
 *
 * Fudge flag4 causes the dubugging output described above to be
 * recorded in the clockstats file. When the audio driver is compiled,
 * fudge flag2 selects the audio input port, where 0 is the mike port
 * (default) and 1 is the line-in port. It does not seem useful to
 * select the compact disc player port. Fudge flag3 enables audio
 * monitoring of the input signal. For this purpose, the monitor gain is
 * set to a default value.
 *
 * The audio codec code is normally compiled in the driver if the
 * architecture supports it (HAVE_AUDIO defined), but is used only if
 * the link /dev/chu_audio is defined and valid. The serial port code is
 * always compiled in the driver, but is used only if the autdio codec
 * is not available and the link /dev/chu%d is defined and valid.
 *
 * The ICOM code is normally compiled in the driver if selected (ICOM
 * defined), but is used only if the link /dev/icom%d is defined and
 * valid and the mode keyword on the server configuration command
 * specifies a nonzero mode (ICOM ID select code). The C-IV speed is
 * 9600 bps if the high order 0x80 bit of the mode is zero and 1200 bps
 * if one. The C-IV trace is turned on if the debug level is greater
 * than one.
 *
 * Alarm codes
 *
 * CEVNT_BADTIME	invalid date or time
 * CEVNT_PROP		propagation failure - no stations heard
 */
/*
 * Interface definitions
 */
#define	PRECISION	(-10)	/* precision assumed (about 1 ms) */
#define	REFID		"CHU"	/* reference ID */

/*
 * Audio demodulator definitions
 */
#define SECOND		8000	/* nominal sample rate (Hz) */
#define BAUD		300	/* modulation rate (bps) */
#define OFFSET		128	/* companded sample offset */
#define SIZE		256	/* decompanding table size */
#define	MAXAMP		6000.0f	/* maximum signal level */
#define	MAXCLP		100	/* max clips above reference per s */
#define	SPAN		800.0f	/* min envelope span */
#define LIMIT		1000.0f	/* soft limiter threshold */
#define AGAIN		6.0f	/* baseband gain */
#define LAG		10	/* discriminator lag */
#define	DESCRIPTION	"CHU Audio Receiver" /* WRU */
#define	AUDIO_BUFSIZ 256 /* audio buffer size (30 ms) */
#define BMAX        128 /* max timecode length */
#define MAXSTAGE    60  /* max median filter stages  */

/*
 * Decoder definitions
 */
#define CHAR		(11.0f / 300.0f) /* character time (s) */
#define BURST		11	/* max characters per burst */
#define MINCHAR		9	/* min characters per burst */
#define MINDIST		28	/* min burst distance (of 40)  */
#define MINSYNC		8	/* min sync distance (of 16) */
#define MINSTAMP	20	/* min timestamps (of 60) */
#define MINMETRIC	50	/* min channel metric (of 160) */

/*
 * The on-time synchronization point for the driver is the last stop bit
 * of the first character 170 ms. The modem delay is 0.8 ms, while the
 * receiver delay is approxmately 4.7 ms at 2125 Hz. The fudge value 1.3
 * ms due to the codec and other causes was determined by calibrating to
 * a PPS signal from a GPS receiver. The additional propagation delay
 * specific to each receiver location can be programmed in the fudge
 * time1.
 *
 * The resulting offsets with a 2.4-GHz P4 running FreeBSD 6.1 are
 * generally within 0.5 ms short term with 0.3 ms jitter. The long-term
 * offsets vary up to 0.3 ms due to ionospheric layer height variations.
 * The processor load due to the driver is 0.4 percent.
 */
#define	PDELAY	((170 + 0.8f + 4.7f + 1.3f) / 1000)	/* system delay (s) */

/*
 * Status bits (status)
 */
#define RUNT		0x0001	/* runt burst */
#define NOISE		0x0002	/* noise burst */
#define BFRAME		0x0004	/* invalid format B frame sync */
#define BFORMAT		0x0008	/* invalid format B data */
#define AFRAME		0x0010	/* invalid format A frame sync */
#define AFORMAT		0x0020	/* invalid format A data */
#define DECODE		0x0040	/* invalid data decode */
#define STAMP		0x0080	/* too few timestamps */
#define AVALID		0x0100	/* valid A frame */
#define BVALID		0x0200	/* valid B frame */
#define INSYNC		0x0400	/* clock synchronized */
#define	METRIC		0x0800	/* one or more stations heard */

/*
 * Alarm status bits (alarm)
 *
 * These alarms are set at the end of a minute in which at least one
 * burst was received. SYNERR is raised if the AFRAME or BFRAME status
 * bits are set during the minute, FMTERR is raised if the AFORMAT or
 * BFORMAT status bits are set, DECERR is raised if the DECODE status
 * bit is set and TSPERR is raised if the STAMP status bit is set.
 */
#define SYNERR		0x01	/* frame sync error */
#define FMTERR		0x02	/* data format error */
#define DECERR		0x04	/* data decoding error */
#define TSPERR		0x08	/* insufficient data */

/*
 * Maximum-likelihood UART structure. There are eight of these
 * corresponding to the number of phases.
 */
struct surv {
	l_fp	cstamp;		/* last bit timestamp */
	float	shift[12];	/* sample shift register */
	float	span;		/* shift register envelope span */
	float	dist;		/* sample distance */
	int	uart;		/* decoded character */
};

/*
 * CHU unit control structure
 */
struct chuunit {
	uint8_t	decode[20][16];	/* maximum-likelihood decoding matrix */
	l_fp	cstamp[BURST];	/* character timestamps */
	l_fp	tstamp[MAXSTAGE]; /* timestamp samples */
	l_fp	timestamp;	/* current buffer timestamp */
	l_fp	laststamp;	/* last buffer timestamp */
	l_fp	charstamp;	/* character time as a l_fp */
	int	second;		/* counts the seconds of the minute */
	int	errflg;		/* error flags */
	int	status;		/* status bits */
	char	ident[5];	/* station ID and channel */

	/*
	 * Character burst variables
	 */
	int	cbuf[BURST];	/* character buffer */
	int	ntstamp;	/* number of timestamp samples */
	int	ndx;		/* buffer start index */
	int	prevsec;	/* previous burst second */
	int	burdist;	/* burst distance */
	int	syndist;	/* sync distance */
	int	burstcnt;	/* format A bursts this minute */

	/*
	 * Format particulars
	 */
	int	dst;		/* Canadian DST code */

	/*
	 * Audio codec variables
	 */
	float gain;
	int	fd;	        /* audio port file descriptor */
	int	mongain;	/* codec monitor gain */
	int	clipcnt;	/* sample clip count */
	int	seccnt;		/* second interval counter */

	/*
	 * Modem variables
	 */
	l_fp	tick;		/* audio sample increment */
	float	bpf[9];		/* IIR bandpass filter */
	float	disc[LAG];	/* discriminator shift register */
	float	lpf[27];	/* FIR lowpass filter */
	float	monitor;	/* audio monitor */
	int	discptr;	/* discriminator pointer */

	/*
	 * Maximum-likelihood UART variables
	 */
	float	baud;		/* baud interval */
	struct surv surv[8];	/* UART survivor structures */
	int	decptr;		/* decode pointer */
	int	decpha;		/* decode phase */
	int	dbrk;		/* holdoff counter */

    char a_lastcode[BMAX]; /* last timecode received */
    unsigned int lencode;    /* length of last timecode */

    int year;       /* year of eternity */
    int day;        /* day of year */
    int hour;       /* hour of day */
    int min;        /* minute of hour */
    int sec;        /* second of minute */
    uint32_t    yearstart;  /* beginning of year */
    uint8_t leap;       /* leap/synchronization code */

    /*
     * Configuration data
     */
    float fudgetime1; /* fudge time1 */
    float fudgetime2; /* fudge time2 */
};

/*
 * More function prototypes
 */
static void	chu_decode	(struct chuunit *up, int, l_fp);
static void	chu_burst	(struct chuunit *up);
static void	chu_clear	(struct chuunit *up);
void	chu_a		(struct chuunit *up, int);
void	chu_b		(struct chuunit *up, int);
static float	chu_major	(struct chuunit *up);
void	chu_uart	(struct surv *, float);
void	chu_rf		(struct chuunit *up, float sample);
static void	chu_gain	(struct chuunit *up);

/*
 * Global variables
 */
static char hexchar[] = "0123456789abcdef_*=";

/*
 * Note the tuned frequencies are 1 kHz higher than the carrier. CHU
 * transmits on USB with carrier so we can use AM and the narrow SSB
 * filter.
 */
static float qsy[3] = {3.33, 7.85, 14.67}; /* freq (MHz) */

/*
 * chu_dist - determine the distance of two octet arguments
 */
static int chu_dist(int x, int y)
{
	int	val;		/* bit count */
	int	temp;
	int	i;

	/*
	 * The distance is determined as the weight of the exclusive OR
	 * of the two arguments. The weight is determined by the number
	 * of one bits in the result. Each one bit increases the weight,
	 * while each zero bit decreases it.
	 */
	temp = x ^ y;
	val = 0;
	for (i = 0; i < 8; i++) {
		if ((temp & 0x1) == 0)
			val++;
		else
			val--;
		temp >>= 1;
	}
	return (val);
}

/*
 * chu_start - open the devices and initialize data for processing
 */
struct chuunit *chu_start(void)
{
	struct chuunit *up;

	/*
	 * Allocate and initialize unit structure
	 */
	up = malloc(sizeof(*up));
	memset(up, 0, sizeof(*up));

	/*
	 * Initialize miscellaneous variables
	 */
	//pp->clockdesc = DESCRIPTION;
	strcpy(up->ident, "CHU");
	DTOLFP(CHAR, &up->charstamp);

	up->gain = 127;
	DTOLFP(1. / SECOND, &up->tick);
    return up;
}

/*
 * chu_shutdown - shut down the clock
 */
void
chu_shutdown(struct chuunit *up)
{
	free(up);
}

/*
 * chu_audio_receive - receive data from the audio device
 */
void chu_receive(struct chuunit *up, int16_t *recv_buffer, unsigned int recv_length, l_fp recv_time)
{
	float	sample;		/* codec sample */
	unsigned int bufcnt;		/* buffer counter */
	l_fp	ltemp;		/* l_fp temp */

	/*
	 * Main loop - read until there ain't no more. Note codec
	 * samples are bit-inverted.
	 */
	DTOLFP((float)recv_length / SECOND, &ltemp);
	L_SUB(&recv_time, &ltemp);
	up->timestamp = recv_time;
	for (bufcnt = 0; bufcnt < recv_length; bufcnt++) {
		sample = ((float)recv_buffer[bufcnt]);

		/*
		 * Clip noise spikes greater than MAXAMP. If no clips,
		 * increase the gain a tad; if the clips are too high,
		 * decrease a tad.
		 */
		if (sample > MAXAMP) {
			sample = MAXAMP;
			up->clipcnt++;
		} else if (sample < -MAXAMP) {
			sample = -MAXAMP;
			up->clipcnt++;
		}
		chu_rf(up, sample);
		L_ADD(&up->timestamp, &up->tick);

		/*
		 * Once each second ride gain.
		 */
		up->seccnt = (up->seccnt + 1) % SECOND;
		if (up->seccnt == 0) {
			chu_gain(up);
		}
	}
}

/*
 * chu_rf - filter and demodulate the FSK signal
 *
 * This routine implements a 300-baud Bell 103 modem with mark 2225 Hz
 * and space 2025 Hz. It uses a bandpass filter followed by a soft
 * limiter, FM discriminator and lowpass filter. A maximum-likelihood
 * decoder samples the baseband signal at eight times the baud rate and
 * detects the start bit of each character.
 *
 * The filters are built for speed, which explains the rather clumsy
 * code. Hopefully, the compiler will efficiently implement the move-
 * and-muiltiply-and-add operations.
 */
void chu_rf(struct chuunit *up, float sample)
{
	struct surv *sp;

	/*
	 * Local variables
	 */
	float	signal;		/* bandpass signal */
	float	limit;		/* limiter signal */
	float	disc;		/* discriminator signal */
	float	lpf;		/* lowpass signal */
	float	dist;		/* UART signal distance */
	int	i, j;

	/*
	 * Bandpass filter. 4th-order elliptic, 500-Hz bandpass centered
	 * at 2125 Hz. Passband ripple 0.3 dB, stopband ripple 50 dB,
	 * phase delay 0.24 ms.
	 */
	signal  = (up->bpf[8] = up->bpf[7]) * 0.5844676f;
	signal += (up->bpf[7] = up->bpf[6]) * 0.488486f;
	signal += (up->bpf[6] = up->bpf[5]) * 2.704384f;
	signal += (up->bpf[5] = up->bpf[4]) * 1.645032f;
	signal += (up->bpf[4] = up->bpf[3]) * 4.644557f;
	signal += (up->bpf[3] = up->bpf[2]) * 1.879165f;
	signal += (up->bpf[2] = up->bpf[1]) * 3.522634f;
	signal += (up->bpf[1] = up->bpf[0]) * 0.7315738f;
	up->bpf[0] = sample - signal;
	signal = up->bpf[0] * 6.176213e-03f
	    + up->bpf[1] * 3.156599e-03f
	    + up->bpf[2] * 7.567487e-03f
	    + up->bpf[3] * 4.344580e-03f
	    + up->bpf[4] * 1.190128e-02f
	    + up->bpf[5] * 4.344580e-03f
	    + up->bpf[6] * 7.567487e-03f
	    + up->bpf[7] * 3.156599e-03f
	    + up->bpf[8] * 6.176213e-03f;

	up->monitor = signal * 0.25f;	/* note monitor after filter */

	/*
	 * Soft limiter/discriminator. The 11-sample discriminator lag
	 * interval corresponds to three cycles of 2125 Hz, which
	 * requires the sample frequency to be 2125 * 11 / 3 = 7791.7
	 * Hz. The discriminator output varies +-0.5 interval for input
	 * frequency 2025-2225 Hz. However, we don't get to sample at
	 * this frequency, so the discriminator output is biased. Life
	 * at 8000 Hz sucks.
	 */
	limit = signal;
	if (limit > LIMIT)
		limit = LIMIT;
	else if (limit < -LIMIT)
		limit = -LIMIT;
	disc = up->disc[up->discptr] * -limit;
	up->disc[up->discptr] = limit;
	up->discptr = (up->discptr + 1 ) % LAG;
	if (disc >= 0)
		disc = sqrtf(disc);
	else
		disc = -sqrtf(-disc);

	/*
	 * Lowpass filter. Raised cosine FIR, Ts = 1 / 300, beta = 0.1.
	 */
	lpf  = (up->lpf[26] = up->lpf[25]) * 2.538771e-02f;
	lpf += (up->lpf[25] = up->lpf[24]) * 1.084671e-01f;
	lpf += (up->lpf[24] = up->lpf[23]) * 2.003159e-01f;
	lpf += (up->lpf[23] = up->lpf[22]) * 2.985303e-01f;
	lpf += (up->lpf[22] = up->lpf[21]) * 4.003697e-01f;
	lpf += (up->lpf[21] = up->lpf[20]) * 5.028552e-01f;
	lpf += (up->lpf[20] = up->lpf[19]) * 6.028795e-01f;
	lpf += (up->lpf[19] = up->lpf[18]) * 6.973249e-01f;
	lpf += (up->lpf[18] = up->lpf[17]) * 7.831828e-01f;
	lpf += (up->lpf[17] = up->lpf[16]) * 8.576717e-01f;
	lpf += (up->lpf[16] = up->lpf[15]) * 9.183463e-01f;
	lpf += (up->lpf[15] = up->lpf[14]) * 9.631951e-01f;
	lpf += (up->lpf[14] = up->lpf[13]) * 9.907208e-01f;
	lpf += (up->lpf[13] = up->lpf[12]) * 1.000000e+00f;
	lpf += (up->lpf[12] = up->lpf[11]) * 9.907208e-01f;
	lpf += (up->lpf[11] = up->lpf[10]) * 9.631951e-01f;
	lpf += (up->lpf[10] = up->lpf[9]) * 9.183463e-01f;
	lpf += (up->lpf[9] = up->lpf[8]) * 8.576717e-01f;
	lpf += (up->lpf[8] = up->lpf[7]) * 7.831828e-01f;
	lpf += (up->lpf[7] = up->lpf[6]) * 6.973249e-01f;
	lpf += (up->lpf[6] = up->lpf[5]) * 6.028795e-01f;
	lpf += (up->lpf[5] = up->lpf[4]) * 5.028552e-01f;
	lpf += (up->lpf[4] = up->lpf[3]) * 4.003697e-01f;
	lpf += (up->lpf[3] = up->lpf[2]) * 2.985303e-01f;
	lpf += (up->lpf[2] = up->lpf[1]) * 2.003159e-01f;
	lpf += (up->lpf[1] = up->lpf[0]) * 1.084671e-01f;
	lpf += up->lpf[0] = disc * 2.538771e-02;

	/*
	 * Maximum-likelihood decoder. The UART updates each of the
	 * eight survivors and determines the span, slice level and
	 * tentative decoded character. Valid 11-bit characters are
	 * framed so that bit 10 and bit 11 (stop bits) are mark and bit
	 * 1 (start bit) is space. When a valid character is found, the
	 * survivor with maximum distance determines the final decoded
	 * character.
	 */
	up->baud += 1.0f / SECOND;
	if (up->baud > 1.0f / (BAUD * 8.0f)) {
		up->baud -= 1.0f / (BAUD * 8.0f);
		up->decptr = (up->decptr + 1) % 8;
		sp = &up->surv[up->decptr];
		sp->cstamp = up->timestamp;
		chu_uart(sp, -lpf * AGAIN);
		if (up->dbrk > 0) {
			up->dbrk--;
			if (up->dbrk > 0)
				return;

			up->decpha = up->decptr;
		}
		if (up->decptr != up->decpha)
			return;

		dist = 0;
		j = -1;
		for (i = 0; i < 8; i++) {
			/*
			 * The timestamp is taken at the last bit, so
			 * for correct decoding we reqire sufficient
			 * span and correct start bit and two stop bits.
			 */
			if ((up->surv[i].uart & 0x601) != 0x600 ||
			    up->surv[i].span < SPAN)
				continue;

			if (up->surv[i].dist > dist) {
				dist = up->surv[i].dist;
				j = i;
			}
		}
		if (j < 0)
			return;

		/*
		 * Process the character, then blank the decoder until
		 * the end of the next character.This sets the decoding
		 * phase of the entire burst from the phase of the first
		 * character.
		 */
		//up->maxsignal = up->surv[j].span;
		chu_decode(up, (up->surv[j].uart >> 1) & 0xff, up->surv[j].cstamp);
		up->dbrk = 88;
	}
}

/*
 * chu_uart - maximum-likelihood UART
 *
 * This routine updates a shift register holding the last 11 envelope
 * samples. It then computes the slice level and span over these samples
 * and determines the tentative data bits and distance. The calling
 * program selects over the last eight survivors the one with maximum
 * distance to determine the decoded character.
 */
void chu_uart(
	struct surv *sp,	/* survivor structure pointer */
	float sample		/* baseband signal */
	)
{
	float	es_max, es_min;	/* max/min envelope */
	float	slice;		/* slice level */
	float	dist;		/* distance */
	float	dtemp;
	int	i;

	/*
	 * Save the sample and shift right. At the same time, measure
	 * the maximum and minimum over all eleven samples.
	 */
	es_max = -1e6;
	es_min = 1e6;
	sp->shift[0] = sample;
	for (i = 11; i > 0; i--) {
		sp->shift[i] = sp->shift[i - 1];
		if (sp->shift[i] > es_max)
			es_max = sp->shift[i];
		if (sp->shift[i] < es_min)
			es_min = sp->shift[i];
	}

	/*
	 * Determine the span as the maximum less the minimum and the
	 * slice level as the minimum plus a fraction of the span. Note
	 * the slight bias toward mark to correct for the modem tendency
	 * to make more mark than space errors. Compute the distance on
	 * the assumption the last two bits must be mark, the first
	 * space and the rest either mark or space.
	 */
	sp->span = es_max - es_min;
	slice = es_min + 0.45f * sp->span;
	dist = 0;
	sp->uart = 0;
	for (i = 1; i < 12; i++) {
		sp->uart <<= 1;
		dtemp = sp->shift[i];
		if (dtemp > slice)
			sp->uart |= 0x1;
		if (i == 1 || i == 2) {
			dist += dtemp - es_min;
		} else if (i == 11) {
			dist += es_max - dtemp;
		} else {
			if (dtemp > slice)
				dist += dtemp - es_min;
			else
				dist += es_max - dtemp;
		}
	}
	sp->dist = dist / (11 * sp->span);
}

/*
 * chu_decode - decode the character data
 */
/* data character */
/* data character timestamp */
static void chu_decode(struct chuunit *up, int	hexhex, l_fp cstamp)
{
	l_fp	tstmp;		/* timestamp temp */
	float	dtemp;

	/*
	 * If the interval since the last character is greater than the
	 * longest burst, process the last burst and start a new one. If
	 * the interval is less than this but greater than two
	 * characters, consider this a noise burst and reject it.
	 */
	tstmp = up->timestamp;
	if (L_ISZERO(&up->laststamp))
		up->laststamp = up->timestamp;
	L_SUB(&tstmp, &up->laststamp);
	up->laststamp = up->timestamp;
	LFPTOD(&tstmp, dtemp);
	if (dtemp > BURST * CHAR) {
		chu_burst(up);
		up->ndx = 0;
	} else if (dtemp > 2.5f * CHAR) {
		up->ndx = 0;
	}

	/*
	 * Append the character to the current burst and append the
	 * character timestamp to the timestamp list.
	 */
	if (up->ndx < BURST) {
		up->cbuf[up->ndx] = hexhex & 0xff;
		up->cstamp[up->ndx] = cstamp;
		up->ndx++;

	}
}

/*
 * chu_burst - search for valid burst format
 */
static void chu_burst(struct chuunit *up)
{
	int	i;

	/*
	 * Correlate a block of five characters with the next block of
	 * five characters. The burst distance is defined as the number
	 * of bits that match in the two blocks for format A and that
	 * match the inverse for format B.
	 */
	if (up->ndx < MINCHAR) {
		up->status |= RUNT;
		return;
	}
	up->burdist = 0;
	for (i = 0; i < 5 && i < up->ndx - 5; i++)
		up->burdist += chu_dist(up->cbuf[i], up->cbuf[i + 5]);

	/*
	 * If the burst distance is at least MINDIST, this must be a
	 * format A burst; if the value is not greater than -MINDIST, it
	 * must be a format B burst. If the B burst is perfect, we
	 * believe it; otherwise, it is a noise burst and of no use to
	 * anybody.
	 */
	if (up->burdist >= MINDIST) {
		chu_a(up, up->ndx);
	} else if (up->burdist <= -MINDIST) {
		chu_b(up, up->ndx);
	} else {
		up->status |= NOISE;
		return;
	}
}

/*
 * chu_b - decode format B burst
 */
void chu_b(struct chuunit *up, int nchar)
{
	//uint8_t	code[11];	/* decoded timecode */
	char	tbuf[80];	/* trace buffer */
	char *	p;
	size_t	chars;
	size_t	cb;
	int	i;

	/*
	 * In a format B burst, a character is considered valid only if
	 * the first occurence matches the last occurence. The burst is
	 * considered valid only if all characters are valid; that is,
	 * only if the distance is 40. Note that once a valid frame has
	 * been found errors are ignored.
	 */
	snprintf(tbuf, sizeof(tbuf), "chuB %04x %4.0f %2d %2d ",
		     up->status, /*up->maxsignal*/255.0, nchar, -up->burdist);
	cb = sizeof(tbuf);
	p = tbuf;
	for (i = 0; i < nchar; i++) {
		chars = strlen(p);
		if (cb < chars + 1) {
			//msyslog(LOG_ERR, "chu_b() fatal out buffer");
			return;
		}
		cb -= chars;
		p += chars;
		snprintf(p, cb, "%02x", up->cbuf[i]);
	}
#ifdef DEBUG
	if (debug) {
        write(1, tbuf, strlen(tbuf));
    }
#endif
	if (up->burdist > -40) {
		up->status |= BFRAME;
		return;
	}

	/*
	 * Convert the burst data to internal format. Don't bother with
	 * the timestamps.
	 */
    up->year = ((up->cbuf[3] << 8)|up->cbuf[2]);
    up->dst = up->cbuf[4];
}

/*
 * chu_a - decode format A burst
 */
void chu_a(struct chuunit *up, int nchar)
{
	char	tbuf[80];	/* trace buffer */
	char *	p;
	size_t	chars;
	size_t	cb;
	l_fp	offset;		/* timestamp offset */
	int	val;		/* distance */
	int	temp;
	int	i, j, k;

	/*
	 * Determine correct burst phase. There are three cases
	 * corresponding to in-phase, one character early or one
	 * character late. These cases are distinguished by the position
	 * of the framing digits 0x6 at positions 0 and 5 and 0x3 at
	 * positions 4 and 9. The correct phase is when the distance
	 * relative to the framing digits is maximum. The burst is valid
	 * only if the maximum distance is at least MINSYNC.
	 */
	up->syndist = k = 0;
	val = -16;
	for (i = -1; i < 2; i++) {
		temp = up->cbuf[i + 4] & 0xf;
		if (i >= 0)
			temp |= (up->cbuf[i] & 0xf) << 4;
		val = chu_dist(temp, 0x63);
		temp = (up->cbuf[i + 5] & 0xf) << 4;
		if (i + 9 < nchar)
			temp |= up->cbuf[i + 9] & 0xf;
		val += chu_dist(temp, 0x63);
		if (val > up->syndist) {
			up->syndist = val;
			k = i;
		}
	}

	/*
	 * Extract the second number; it must be in the range 2 through
	 * 9 and the two repititions must be the same.
	 */
	temp = (up->cbuf[k + 4] >> 4) & 0xf;
	if (temp < 2 || temp > 9 || k + 9 >= nchar || temp !=
	    ((up->cbuf[k + 9] >> 4) & 0xf))
		temp = 0;
	snprintf(tbuf, sizeof(tbuf),
		 "chuA %04x %4.0f %2d %2d %2d %2d %1d ", up->status,
		 /*up->maxsignal*/255.0, nchar, up->burdist, k, up->syndist,
		 temp);
	cb = sizeof(tbuf);
	p = tbuf;
	for (i = 0; i < nchar; i++) {
		chars = strlen(p);
		if (cb < chars + 1) {
			//msyslog(LOG_ERR, "chu_a() fatal out buffer");
			return;
		}
		cb -= chars;
		p += chars;
		snprintf(p, cb, "%02x", up->cbuf[i]);
	}
    *p++ = '\n';
    *p = '\0';
    write(1, tbuf, strlen(tbuf));
	if (up->syndist < MINSYNC) {
		up->status |= AFRAME;
		return;
	}

	/*
	 * A valid burst requires the first seconds number to match the
	 * last seconds number. If so, the burst timestamps are
	 * corrected to the current minute and saved for later
	 * processing. In addition, the seconds decode is advanced from
	 * the previous burst to the current one.
	 */
	if (temp == 0) {
		up->status |= AFORMAT;
	} else {
		up->status |= AVALID;
		up->sec = 30 + temp;
		offset.l_ui = 30 + temp;
		offset.l_f = 0;
		i = 0;
		if (k < 0)
			offset = up->charstamp;
		else if (k > 0)
			i = 1;
		for (; i < nchar && i < k + 10; i++) {
			up->tstamp[up->ntstamp] = up->cstamp[i];
			L_SUB(&up->tstamp[up->ntstamp], &offset);
			L_ADD(&offset, &up->charstamp);
			if (up->ntstamp < MAXSTAGE - 1)
				up->ntstamp++;
		}
		while (temp > up->prevsec) {
			for (j = 15; j > 0; j--) {
				up->decode[9][j] = up->decode[9][j - 1];
				up->decode[19][j] =
				    up->decode[19][j - 1];
			}
			up->decode[9][j] = up->decode[19][j] = 0;
			up->prevsec++;
		}
	}

	/*
	 * Stash the data in the decoding matrix.
	 */
	i = -(2 * k);
	for (j = 0; j < nchar; j++) {
		if (i < 0 || i > 18) {
			i += 2;
			continue;
		}
		up->decode[i][up->cbuf[j] & 0xf]++;
		i++;
		up->decode[i][(up->cbuf[j] >> 4) & 0xf]++;
		i++;
	}
	up->burstcnt++;
}

int clocktime(int yday, int hour, int minute, int second, uint32_t rec_ui, uint32_t *yearstart, uint32_t *ts_ui)
{
	register int32_t tmp;
	register uint32_t yst;

	/*
	 * Compute the offset into the year in seconds.  Note that
	 * this could come out to be a negative number.
	 */
	tmp = (int32_t)((24 * (yday-1)) + hour);
	tmp = (60 * tmp) + (int32_t)minute;
	tmp = (60 * tmp) + (int32_t)second;

	/*
	 * Initialize yearstart, if necessary.
	 */
	yst = *yearstart;
	if (yst == 0) {
		struct calendar jt;
		caljulian(rec_ui,&jt);
		/* The NTP time stamps (l_fp) count seconds unsigned mod 2**32, so we
	 	 * have to calculate this in the proper way!
	 	 */
		yst = (uint32_t)(rec_ui - (uint32_t)jt.yearday);
		*yearstart = yst;
	}

	*ts_ui = (uint32_t)(tmp + (int32_t)yst);
	return 1;
}

/*
 * chu_second - process minute data
 */
void chu_second(int unit, struct chuunit *up, unsigned int update)
{
	l_fp	offset, curtime;
	char	synchar, qual, leapchar;
	int	minset, i;
	float	dtemp;

	/*
	 * This routine is called once per minute to process the
	 * accumulated burst data. We do a bit of fancy footwork so that
	 * this doesn't run while burst data are being accumulated.
	 */
	up->sec = (up->sec + 1) % 60;
	if (up->sec != 0)
		return;

	/*
	 * Process the last burst, if still in the burst buffer.
	 * If the minute contains a valid B frame with sufficient A
	 * frame metric, it is considered valid. However, the timecode
	 * is sent to clockstats even if invalid.
	 */
	chu_burst(up);
    get_systime(&curtime);
	//minset = ((current_time - update) + 30) / 60;
	minset = ((curtime.l_ui - update) + 30) / 60;
	dtemp = chu_major(up);
	qual = 0;
	if (up->status & (BFRAME | AFRAME))
		qual |= SYNERR;
	if (up->status & (BFORMAT | AFORMAT))
		qual |= FMTERR;
	if (up->status & DECODE)
		qual |= DECERR;
	if (up->status & STAMP)
		qual |= TSPERR;
	if (up->status & BVALID && dtemp >= MINMETRIC)
		up->status |= INSYNC;
	synchar = leapchar = ' ';
	if (!(up->status & INSYNC)) {
		up->leap = LEAP_NOTINSYNC;
		synchar = '?';
	} else {
		up->leap = LEAP_NOWARNING;
	}
	up->lencode = snprintf(up->a_lastcode, sizeof(up->a_lastcode), "%c%1X %04d %03d %02u:%02u:%02u %c%x %d %d %s %.0f %d",
	    synchar, qual, up->year, up->day, up->hour, up->min,
	    up->sec, leapchar, up->dst, minset, up->gain,
	    up->ident, dtemp, up->ntstamp);

	/*
	 * If in sync and the signal metric is above threshold, the
	 * timecode is ipso fatso valid and can be selected to
	 * discipline the clock.
	 */
	if (up->status & INSYNC && !(up->status & (DECODE | STAMP)) && dtemp > MINMETRIC) {
		clocktime(up->day, up->hour, up->min, 0, up->tstamp[0].l_ui, &up->yearstart, &offset.l_ui);
		offset.l_uf = 0;
#if 0
		for (i = 0; i < up->ntstamp; i++)
			refclock_process_offset(pp, offset, up->tstamp[i], PDELAY + pp->fudgetime1);
		pp->lastref = up->timestamp;
		refclock_receive(peer);
#endif
	}
	printf("chu: timecode %d %s\n", up->lencode, up->a_lastcode);
	chu_clear(up);
	up->errflg = 0;
}

/*
 * chu_major - majority decoder
 */
static float
chu_major(struct chuunit *up)
{
	uint8_t	code[11];	/* decoded timecode */
	int	metric;		/* distance metric */
	int	val1;		/* maximum distance */
	int	synchar;	/* stray cat */
	int	temp;
	int	i, j, k;

	/*
	 * Majority decoder. Each burst encodes two replications at each
	 * digit position in the timecode. Each row of the decoding
	 * matrix encodes the number of occurences of each digit found
	 * at the corresponding position. The maximum over all
	 * occurrences at each position is the distance for this
	 * position and the corresponding digit is the maximum-
	 * likelihood candidate. If the distance is not more than half
	 * the total number of occurences, a majority has not been found
	 * and the data are discarded. The decoding distance is defined
	 * as the sum of the distances over the first nine digits. The
	 * tenth digit varies over the seconds, so we don't count it.
	 */
	metric = 0;
	for (i = 0; i < 9; i++) {
		val1 = 0;
		k = 0;
		for (j = 0; j < 16; j++) {
			temp = up->decode[i][j] + up->decode[i + 10][j];
			if (temp > val1) {
				val1 = temp;
				k = j;
			}
		}
		if (val1 <= up->burstcnt)
			up->status |= DECODE;
		metric += val1;
		code[i] = hexchar[k];
	}

	/*
	 * Compute the timecode timestamp from the days, hours and
	 * minutes of the timecode. Use clocktime() for the aggregate
	 * minutes and the minute offset computed from the burst
	 * seconds. Note that this code relies on the filesystem time
	 * for the years and does not use the years of the timecode.
	 */
    synchar = (((code[0] - 0x30) <= 9) ? (code[0] - 0x30) : ((code[0]|0x20) - 'a' + 10));
    up->day = ((code[1]*100) + (code[2]*10) + code[3]);
    up->hour = ((code[4]*10) + code[5]);
    up->min = ((code[6]*10) + code[7]);
	if (up->ntstamp < MINSTAMP)
		up->status |= STAMP;
	return (metric);
}

/*
 * chu_clear - clear decoding matrix
 */
static void
chu_clear(struct chuunit *up)
{
	int	i, j;

	/*
	 * Clear stuff for the minute.
	 */
	up->ndx = up->prevsec = 0;
	up->burstcnt = up->ntstamp = 0;
	up->status &= INSYNC | METRIC;
	for (i = 0; i < 20; i++) {
		for (j = 0; j < 16; j++)
			up->decode[i][j] = 0;
	}
}

/*
 * chu_gain - adjust codec gain
 *
 * This routine is called at the end of each second. During the second
 * the number of signal clips above the MAXAMP threshold (6000). If
 * there are no clips, the gain is bumped up; if there are more than
 * MAXCLP clips (100), it is bumped down. The decoder is relatively
 * insensitive to amplitude, so this crudity works just peachy. The
 * routine also jiggles the input port and selectively mutes the
 */
static void
chu_gain(struct chuunit *up)
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

