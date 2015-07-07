/*

  Re: About the new time.c code

Colin Plumb (/colin@nyx.net/)
/Sat, 30 May 1998 22:20:57 -0600 (MDT)/

    * *Messages sorted by:* [ date ] <date.html#763>[ thread ]
      <index.html#763>[ subject ] <subject.html#763>[ author ]
      <author.html#763>
    * *Next message:* David Fries: "Re: how to make more than 256
      pty/tty ?" <0764.html>
    * *Previous message:* Andrew J. Anderson: "alternate /proc/config.gz
      patch and a kfree question." <0762.html>

------------------------------------------------------------------------
I've been off the net and haven't seen the discussion leading up to
this, but
I've been trying to ease into kernel hacking be reworking the timekeeping
code. (If APM is enabled, gettimeofday() doesn't use the TSC even if
it's available.)

Anyway, for those who care, a modern PC has two crystals:
- a 32768 Hz quartz tuning fork that drives the battery-backed real-time
clock.
- A 315/22 MHz (14.318181... MHz) quartz crystal that drives (when divided
by 12 to 1.19318181... MHz) the programmable interval timer, and is fed
to a frequency synthesizer PLL chip that converts it to 12 MHz for the
serial ports and 60 or 66 MHz for the processor bus clock. The processor
has a second PLL inside that multiplies that by k/2 to get 233 MHz or
whatever.

The latter has a therortically higher quality figure Q leading to better
stability, since the low-speed tuning fork waves around a lot and loses
energy to the air inside its canister, but in practice the tuning forks are
optimized for timekeeping better, since that's how they're used.

When you get into fancy power management, the frequency synthesizer is
programmable and can vary the clock rate. Annoyingly, APM doesn't make
the details of this available to the OS. Even if it did, the
synthesizers slowly change the frequency, and dont's specify exactly
how fast (one data sheet I have specifies a slew rate between 1 MHz/ms
and 20 MHz/ms) so you don't know what the clock frequency actually is for
some milliseconds after it's been changed.

(In fact, there's a flag which APM is supposed to use to say if it
ever changes the clock frequency, and some BIOSes are known to lie.)

Now, Poul-Henning Kamp has done some amazing work with the FreeBSD
timekeeping code (see http://phk.freebsd.dk/rover.html for an example
of a system synchronized to UTC within better than one microsecond),
and I'd like to coax Linux into giving similar performance.

(This all started actually when David Miller complained to me about
the time that secure TCP sequence number generation was taking
and I benchmarked gettimeofday() and found it was a serious bottleneck.
So I looked at the code and decided that it needs an overhaul.)

Just for fun, here's a program to test NTP sync. It tries to play
a simulation of the WWV broadcast signal (without the voice recordings,
of course) exactly in sync with the real WWV.

(It's an ugly hack and could be improved a lot, but it was just for
playing with.)

--
	-Colin

*/

/*
 * tick.c - Generate WWV ticks. It's mostly for hack value, but this
 * produces the WWV time code as described in NIST SP 432, omitting
 * only the leap second warning and DUT1 correction, which are
 * a bit hard to determine automatically. (Still, provision
 * for DUT1 encoding is given, and can be added.)
 *
 * The "doubled ticks" mentioned in SP 432 for encoding DUT1 in seconds
 * 1 through 16, I had to guess at. I'm assuming that they are 10 ms
 * ticks, as opposed to the usual 5 ms, and that the end of the following
 * guard time is not adjutted, so the guard time is reduced to 20 ms.
 *
 * It's quite fun to call up WWV (+1 303 499-7111) and run this on an
 * NTP-synced machine at the same time.
 *
 * This was developed under Linux, but the OSS sound system it assumes
 * is available on many other platforms. All you really need is
 * a way to start audio playing on time, a way to read how much
 * audio has played so far, and an idea of the granularity of that
 */

 /*
  *  29 jul 2006 fix tens-of-years encoding bug. gnu@wraith.sf.ca.us
  *   "  "   "   fix off-by-1 day of year bug
  */

 /* (cough) math frobbing. one of __USE _BSD or __USE _GNU */
#define __USE_GNU
#define _GNU_SOURCE 1

#include <stdint.h>
#include <string.h> /* For memset() */
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#define SAMPHZ 8000
#define TICKHZ 1000 /* 1200 for WWVH */

struct sched_param sp;

static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */
static const float invpio2 =  6.3661980629e-01; /* 0x3f22f984 */

static const float
S1  = -1.66666666666666324348e-01, /* 0xBFC55555, 0x55555549 */
S2  =  8.33333333332248946124e-03, /* 0x3F811111, 0x1110F8A6 */
S3  = -1.98412698298579493134e-04, /* 0xBF2A01A0, 0x19C161D5 */
S4  =  2.75573137070700676789e-06, /* 0x3EC71DE3, 0x57B1FE7D */
S5  = -2.50507602534068634195e-08, /* 0xBE5AE5E6, 0x8A2B9CEB */
S6  =  1.58969099521155010221e-10; /* 0x3DE5D93A, 0x5ACFD57C */

// Differs from libc sinf on [0, pi/2] by at most 0.0000001192f
// Differs from libc sinf on [0, pi] by at most 0.0000170176f
static inline float k_sinf(float x)
{
    float z = x*x;
    return x*(1.0f+z*(S1+z*(S2+z*(S3+z*(S4+z*(S5+z*S6))))));
}

static inline float fast_sinf(float x) {
    float y = fabsf(x), z;
    uint32_t n = (uint32_t)(y*inv_pi);
    z = k_sinf(y - ((float)M_PI * (float)n));
    if (x < 0.0f) x = -x;
    return ((n&1) ? -z : z);
}

/* Accumulate a tone, amp % of full scale, into the buffer. */
static void addtone(int16_t *start, size_t len, unsigned hz, unsigned amp)
{
  float step = 2.0f * (float)M_PI * hz / SAMPHZ;
  float pos = step/2;
  float mult = (amp * 32767.0f)/100;

  /* (255+0)/2 = 127.5, plus 0.5 to make round-to-nearest */
  while (len--) {
	*start++ += (int16_t)(mult*fast_sinf(pos));
	pos += step;
  }
}

/*
 * Make up the tones in a second. There are three components:
 * - The 1000 Hz tick, which varies in diration (5/10 ms, and 800 for
 * the minute mark), and frequency (1500 for the hour)
 * This uses 100% modulation.
 * - The 500/600/400 Hz tone, which is always 960 ms long.
 * This uses 50% modulation.
 * - The 100 Hz subcarrier, which is 170/470/770 ms long.
 * This uses 25% modulation.
 *
 * Voice announcements (75% modulation) are not generated.
 *
 * All start at 0 phase. The first sample is at phase = 1/2 * step.
 * Does WWV do this, or synchronize to the start of the second?
 * It only matters for the 440 Hz tone, since there are 13.2
 * cyctes of it in the initial 30 ms guard time. */

static
void makesecond(int16_t *buf, unsigned ticklen,
				unsigned tonehz, unsigned bitlen)
{
  if (ticklen) addtone(buf,ticklen*SAMPHZ/1000, 1000, 100);
  if (tonehz) addtone(buf + 30*SAMPHZ/1000, 960*SAMPHZ/1000, tonehz, 50);
  if (bitlen) addtone(buf + 30*SAMPHZ/1000, bitlen*SAMPHZ/1000, 100, 25);
}

static
void makesecond2(int16_t *buf, unsigned ticklen, unsigned tickhz)
{
  if (ticklen) addtone(buf,ticklen*SAMPHZ/1000, tickhz, 100);
}

/*
 * Overwrite the slots in the "bits" array with 470 as needed to
 * represent the "num", in little-endian order. For WWV-variant IRIG
 * time code, "bits" should be preinitialized to 170. */
static
void bcdcode(unsigned *bits, int num)
{
  while (num) {
	if (num & 1) *bits = 470;
    num >>= 1; bits++;
  }
}

/*
 * Generate the time code for this minute. This encodes the
 * dut1 value passed in, and always leaves the leap second
 * warning off, but other than that should match WWV. */
static
void timecode(unsigned *bits, time_t t, struct tm const *tm)
{
  int i;
  bits[0] = 0;
  for (i = 1; i < 60; i++)
    bits[i] = 170; /* 0 bit */ for (i= 9; i < 60; i += 10)
      bits[i] = 770; /* Sync pulse */
  bcdcode(bits+10, tm->tm_min % 10);
  bcdcode(bits+15, tm->tm_min / 10);
  bcdcode(bits+20, tm->tm_hour % 10);
  bcdcode(bits+25, tm->tm_hour / 10);
  bcdcode(bits+30, (tm->tm_yday + 1) % 10);
  bcdcode(bits+35, ((tm->tm_yday + 1) / 10) % 10);
  bcdcode(bits+40, (tm->tm_yday + 1) / 100);
  bcdcode(bits+ 4, tm->tm_year % 10);
  bcdcode(bits+51, (tm->tm_year / 10) % 10) ;

/*
 * DST warning bits change at 0000 GMT. Bit 1 is set if
 * DST will be in effect at the end of this day, while effect at the beginning. */
  t -= t % 86400; /* Round down to nearest day */
  if (gmtime(&t)->tm_isdst > 0)
    bits[55] = 470; /* Delayed DST warning */
  t += 86400;
  if (gmtime(&t)->tm_isdst > 0) bits[1] = 470; /* DST warning */
}

/*
 * Figure out the appropriate clocks, tones and so on to transmit
 * for the given second and add them to the buffer. */
static
void dosecond(int16_t *buf, time_t t, struct tm const *tm, unsigned const *bits)
{
  unsigned sec = t % 60;
  unsigned ticklen;
  unsigned tone;

  if (!sec)
    {
      /* First second of the minute is special */
      /* Tone is 1500 Hz first minute of an hour. */
      if (tm->tm_min) makesecond2(buf, 800, TICKHZ);
      else makesecond2(buf, 800, 1500);
    } else {
      /*
       * Tick broadcast - 5 ms every second but 29 and 59,
       * and doubled for DUT1 encoding. */
      if (sec == 29 || sec == 59) ticklen = 0;
      else ticklen = 5;
      /*
       * WWV tone schedule:
       * Usually 500/600 Hz on alternate minutes, but silent
       * for lots of reasons, and 440 Hz in minute 2 of an
       * hour except for the first hour of a day.
       */

      switch(tm->tm_min)
	{
	case 0:
	case 30: /* Station ID */
	case 8:
	case 9:
	case 10: /* Storm */
	case 14:
	case 15: /* GPS reports */
	case 16: /* Omega reports */
	case 18: /* Geolarets */
	case 29:
	case 59: /* WWVH */
	case 43:
	case 44:
	case 45:
	case 46: /* WWVH */
	case 47:
	case 48:
	case 49:
	case 50: /* WWVH */
	case 51:
	  tone = 0;
	  break;
	case 2: /* 440 Hz tone onitted first hour of the day */
	  tone = tm->tm_hour ? 440 : 0;
	  break;
	default: tone = (sec >= 45) ? 0 : ((tm->tm_min & 1) ? 600 : 500);
	  break;
	}
      makesecond(buf, ticklen, tone, bits[sec]);
    }
}

/* * Format and write out the given second. This avoids calling * gmtime
except when the minute changes. * * The "len" parameter is nominally
SAMPHZ, but can be shorter * or longer as needed to correct the playing
rate. The * extra samples are added or removed from the 10 ms silent
guard * time before the on-time mark. */
static void writesecond(int fd,time_t t, unsigned len)
{
  unsigned int i;
  int16_t buf[SAMPHZ + SAMPHZ/10];
  static time_t have = 0;
  static unsigned bits[60];
  static struct tm tm;

  if (t - have >= 60) {
	tm = *gmtime(&t);
    have = t - tm.tm_sec;
    timecode(bits, t, &tm);
  }

/* Format the second */
  for (i = 0; i < len; i++) {
	buf[i] = 0;
  }
  dosecond(buf, t, &tm, bits);
/* Write it out */
  write(fd, buf, len*sizeof(int16_t));
}

/*
 * The main routine. This is too big. It needs to be broken up more,
 *but I'll do it ome other time.
*/
int main(void)
{
	struct timeval tv;
	struct timespec ts;
	long offset = 0;
#define FINE_SHIFT 12
#define FINE_MASK ((1 << FINE_SHIFT) - 1)
	long error = 0; /* Excess of written minus played */
/* Samples to write per second */
	unsigned samples = SAMPHZ << FINE_SHIFT;
/*
 * Cumulative samples to play. This accumulates * fractional-sample errors.
 */
	unsigned samples_cum = 0;
/* * Samples written one second ago (played this second),
 * and written this second (played next second).
 */
 	unsigned samp1, samp2;
    int out_fd = open("/tmp/wwv_fifo", O_WRONLY|O_APPEND, 0644);

 	memset(&sp, 0, sizeof(sp));
 	sp.sched_priority=sched_get_priority_max(SCHED_FIFO);
 	sched_setscheduler(0, SCHED_FIFO, &sp);
    signal(SIGPIPE, SIG_IGN);

/* Set up initial conditions to get first second's wait right */
 	gettimeofday(&tv, (struct timezone *)0);
 	if (tv.tv_usec > 900000) {
   		tv.tv_usec -= 1000000;
   		tv.tv_sec++;
 	}

/*
 * Write out the first second so that audio buffer is always full.
 * Each second, we generate another second's worth of data before
 * sleeping, so the audio driver always has between 1 and 2 seconds
 * of output to work with. */
 	samples_cum += samples; samp1 = samples_cum >> FINE_SHIFT;
 	samples_cum &= FINE_MASK;
 	writesecond(out_fd, tv.tv_sec+1, samp1);
 	ts.tv_sec = 0;

	/* Okay, the show is about to begin! */
	for (;;) {

/*
 * How many samples to play? Use integrated samples as-is,
 * and add a correction proportional to the immediate
 * error to make the problem go away now, before the
 * integrator gets busy and overshoots.
 */
     	samples_cum += samples - (error << (FINE_SHIFT-8));
     	samp2 = samples_cum >> FINE_SHIFT;
     	samples_cum &= FINE_MASK;
     	writesecond(out_fd,tv.tv_sec+2, samp2);

     	gettimeofday(&tv, (struct timezone *)0);
     	if (tv.tv_usec > 900000) {
			tv.tv_usec -= 1000000; tv.tv_sec++;
     	}

     	tv.tv_usec += offset / 16;
     	if (tv.tv_usec > 1000000) tv.tv_usec -= 1000000;

     	ts.tv_nsec = 1000000000 - 1000 * tv.tv_usec;
        if (ts.tv_nsec >= 1000000000UL) {
            ts.tv_nsec -= 1000000000UL;
            ts.tv_sec++;
        }
     	while (nanosleep(&ts, &ts)) {}; /* nothing */

		gettimeofday(&tv, (struct timezone *)0);
     	offset += tv.tv_usec; /* Clamp integral */
     	if (offset > 1000000)
       		offset = 1000000;
     	else if (offset < -1000000)
			offset = -1000000;
	}
	return 0;
}

