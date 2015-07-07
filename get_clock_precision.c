#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "ntp_fp.h"
double sys_tick = 0;		/* precision (time to read the clock) */

void get_normalized_timeofday(struct timespec *ts) /* system time */
{
	/*
	 * Convert Unix timeval from seconds and microseconds to NTP
	 * seconds and fraction.
	 */
	clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_sec += JAN_1970;
	if (ts->tv_nsec >= 1000000000L) {
		ts->tv_nsec -= 1000000000L;
        ts->tv_sec++;
	} else if (ts->tv_nsec < 0) {
		ts->tv_nsec += 1000000000L;
        ts->tv_sec++;
	}
}

static inline unsigned long av_log2(unsigned long x) {
    unsigned long ret;
    __asm__ volatile("or $1, %1 \n\t"
             "bsr %1, %0 \n\t"
             :"=a"(ret) : "D"(x));
    return ret;
}

/*
 * Find the precision of this particular machine
 */
#define MINSTEP 100      /* minimum clock increment (s) */
#define MAXSTEP 86400000000ULL    /* maximum clock increment (s) */
#define MINLOOPS 255      /* minimum number of step samples */

/*
 * This routine measures the system precision defined as the minimum of
 * a sequence of differences between successive readings of the system
 * clock. However, if a difference is less than MINSTEP, the clock has
 * been read more than once during a clock tick and the difference is
 * ignored. We set MINSTEP greater than zero in case something happens
 * like a cache miss.
 */
int default_get_precision(void)
{
    struct timespec val;        /* current seconds fraction */
    struct timespec last;       /* last seconds fraction */
    struct timespec diff;       /* difference */
    int64_t tick = MAXSTEP; /* computed tick value */
    int64_t dtemp;      /* scratch */
    int64_t i = 0;      /* log2 precision */

    /*
     * Loop to find precision value in seconds.
     */
    get_normalized_timeofday(&last);
    while (1) {
        get_normalized_timeofday(&val);
        if (!last.tv_nsec) {
            diff.tv_sec = val.tv_sec - last.tv_sec;
            diff.tv_nsec = 0;
        } else {
            uint32_t sub_t = val.tv_nsec - last.tv_nsec;
            diff.tv_nsec = val.tv_nsec - last.tv_nsec;
            diff.tv_sec = val.tv_sec - last.tv_sec + ((uint32_t)diff.tv_nsec > sub_t);
        }
        last = val;
		//dtemp = 1000000UL*diff.tv_sec + diff.tv_usec;
        dtemp = diff.tv_nsec;
        if (dtemp < MINSTEP)
            continue;

        if (dtemp < tick)
            tick = dtemp;
        if (++i >= MINLOOPS)
            break;
    }

    /*
     * Find the nearest power of two.
     */
    i = av_log2(tick);
    printf("proto: precision = %lu nsec (log2: %lu)\n", tick, i);
    sys_tick = (double)tick * 10e-10f;
    return (i);
}

int main(void) {
    default_get_precision();
    return 0;
}

