/*
 * systime -- routines to fiddle a UNIX clock.
 *
 * ATTENTION: Get approval from Dave Mills on all changes to this file!
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timex.h>
#include "ntp_fp.h"
#include <sys/param.h>
#define	FUZZ	500e-6		/* fuzz pivot */
void MD5(unsigned char *dst, const unsigned char *src, unsigned int len);

/*
 * These routines (get_systime, step_systime, adj_systime) implement an
 * interface between the system independent NTP clock and the Unix
 * system clock in various architectures and operating systems. Time is
 * a precious quantity in these routines and every effort is made to
 * minimize errors by unbiased rounding and amortizing adjustment
 * residues.
 *
 * In order to improve the apparent resolution, provide unbiased
 * rounding and insure that the readings cannot be predicted, the low-
 * order unused portion of the time below the resolution limit is filled
 * with an unbiased random fuzz.
 *
 * The sys_tick variable secifies the system clock tick interval in
 * seconds. For systems that can interpolate between timer interrupts,
 * the resolution is presumed much less than the time to read the system
 * clock, which is the value of sys_tick after the precision has been
 * determined. For those systems that cannot interpolate between timer
 * interrupts, sys_tick will be much larger in the order of 10 ms, so the
 * fuzz should be that value. For Sunses the tick is not interpolated, but
 * the system clock is derived from a 2-MHz oscillator, so the resolution
 * is 500 ns and sys_tick is 500 ns.
 */
double sys_tick = 0;		/* precision (time to read the clock) */

typedef struct {
    unsigned int state[64];
    unsigned int index;
} AVLFG;
static AVLFG c;

void ntp_srandom(unsigned int seed){
    uint32_t i, tmp[4]={0};

    for(i=0; i<64; i+=4){
        tmp[0]=seed; tmp[3]=i;
        MD5((uint8_t*)tmp, (uint8_t*)tmp, 16);
        c.state[i  ]= tmp[0];
        c.state[i+1]= tmp[1];
        c.state[i+2]= tmp[2];
        c.state[i+3]= tmp[3];
    }
    c.index=0;
}

/**
 * Get the next random unsigned 32-bit number using an ALFG.
 */
unsigned int ntp_random(void){
    c.state[c.index & 63] = c.state[(c.index-24) & 63] + c.state[(c.index-55) & 63];
    return c.state[c.index++ & 63];
}

/*
 * get_systime - return system time in NTP timestamp format.
 */
void get_systime(l_fp *now) /* system time */
{
	double	dtemp;
	struct timeval tv;	/* seconds and microseconds */

	/*
	 * Convert Unix timeval from seconds and microseconds to NTP
	 * seconds and fraction.
	 */
	gettimeofday(&tv, NULL);
	now->l_i = tv.tv_sec + JAN_1970;
	dtemp = 0;
	if (sys_tick > FUZZ)
		dtemp = ntp_random() * 2. / FRAC * sys_tick * 1e6;
	else if (sys_tick > 0)
		dtemp = ntp_random() * 2. / FRAC;
	dtemp = (tv.tv_usec + dtemp) * 1e-6;
	if (dtemp >= 1.) {
		dtemp -= 1.;
		now->l_i++;
	} else if (dtemp < 0) {
		dtemp += 1.;
		now->l_i--;
	}
	now->l_uf = (uint32_t)(dtemp * FRAC);
}

/*
 * adj_systime - adjust system time by the argument.
 */
int adj_systime(double now) /* adjustment (s) */
{
	struct timeval adjtv;	/* new adjustment */
	double	dtemp;
	long	ticks;
	int	isneg = 0;

	/*
	 * Most Unix adjtime() implementations adjust the system clock
	 * in microsecond quanta, but some adjust in 10-ms quanta. We
	 * carefully round the adjustment to the nearest quantum, then
	 * adjust in quanta and keep the residue for later.
	 */
	dtemp = now;
	if (dtemp < 0) {
		isneg = 1;
		dtemp = -dtemp;
	}
	adjtv.tv_sec = (long)dtemp;
	dtemp -= adjtv.tv_sec;
	ticks = (long)(dtemp / sys_tick + .5);
	adjtv.tv_usec = (long)(ticks * sys_tick * 1e6);
	dtemp -= adjtv.tv_usec / 1e6;

	/*
	 * Convert to signed seconds and microseconds for the Unix
	 * adjtime() system call. Note we purposely lose the adjtime()
	 * leftover.
	 */
	if (isneg) {
		adjtv.tv_sec = -adjtv.tv_sec;
		adjtv.tv_usec = -adjtv.tv_usec;
	}
	if (adjtv.tv_sec != 0 || adjtv.tv_usec != 0) {
        struct timex tntx;
        tntx.offset = adjtv.tv_usec + adjtv.tv_sec * 1000000L;
        tntx.modes = ADJ_OFFSET_SINGLESHOT;
		if (ntp_adjtime(&tntx) < 0) {
			//msyslog(LOG_ERR, "adj_systime: failed to set system time adjustment");
			return (0);
		}
	}
	return (1);
}

/*
 * step_systime - step the system clock.
 */
int step_systime(double now)
{
	struct timeval timetv, adjtv, oldtimetv;
	int isneg = 0;
	double dtemp;

	dtemp = now;
	if (dtemp < 0) {
		isneg = 1;
		dtemp = - dtemp;
		adjtv.tv_sec = (int32_t)dtemp;
		adjtv.tv_usec = (uint32_t)((dtemp -
		    (double)adjtv.tv_sec) * 1e6 + .5);
	} else {
		adjtv.tv_sec = (int32_t)dtemp;
		adjtv.tv_usec = (uint32_t)((dtemp -
		    (double)adjtv.tv_sec) * 1e6 + .5);
	}
    gettimeofday(&timetv, NULL);
	oldtimetv = timetv;

	if (isneg) {
		timetv.tv_sec -= adjtv.tv_sec;
		timetv.tv_usec -= adjtv.tv_usec;
		if (timetv.tv_usec < 0) {
			timetv.tv_sec--;
			timetv.tv_usec += 1000000;
		}
	} else {
		timetv.tv_sec += adjtv.tv_sec;
		timetv.tv_usec += adjtv.tv_usec;
		if (timetv.tv_usec >= 1000000) {
			timetv.tv_sec++;
			timetv.tv_usec -= 1000000;
		}
	}

	/*
	 * Some broken systems don't reset adjtime() when the
	 * clock is stepped.
	 */
	adjtv.tv_sec = adjtv.tv_usec = 0;
	adjtime(&adjtv, NULL);

	if (settimeofday(&timetv, NULL) < 0) {
		//msyslog(LOG_ERR, "step-systime: failed to set system time");
		return (0);
	}
	return (1);
}

