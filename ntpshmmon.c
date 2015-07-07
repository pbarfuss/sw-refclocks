/* ntpshmmon.c -- monitor the inner end of an ntpshmwrite.connection
 *
 * This file is Copyright (c) 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <limits.h>
#include <unistd.h>
#include "ntpshm.h"
#define NTPSEGMENTS	64 /* NTPx for x any byte */

/* difference between timespecs in nanoseconds */
/* int is too small, avoid floats  */
/* WARNING!  this will overflow if x and y differ by more than a few seconds */
#define timespec_diff_ns(x, y)	(long)(((x).tv_sec-(y).tv_sec)*1000000000+(x).tv_nsec-(y).tv_nsec)

static struct shmTime *segments[NTPSEGMENTS + 1];
static unsigned char readseg[NTPSEGMENTS + 1];
static struct timespec tick[NTPSEGMENTS + 1];

int main(int argc, char **argv)
{
    const char *usage_str = "Usage: ntpshmmon [-n max] [-t timeout] [-v] [-h] [-V]\n";
    int option;
    unsigned char verbose = 0;
    unsigned int i, nsamples = INT_MAX;
    time_t timeout = (time_t)INT_MAX, starttime = time(NULL);
    double cycle = 100.0;

    while ((option = getopt(argc, argv, "c:hn:t:vV")) != -1) {
	switch (option) {
	case 'c':
	    cycle = atof(optarg);
	    break;
	case 'n':
	    nsamples = atoi(optarg);
	    break;
	case 't':
	    timeout = (time_t)atoi(optarg);
	    break;
	case 'v':
	    verbose = 1;
	    break;
	case 'V':
	    write(2, "ntpshmmon: version 3.14\n", 24);
        return 0;
	    break;
	case 'h':
	default:
	    write(1, usage_str, strlen(usage_str));
        return 1;
	    break;
	}
    }

    for (i = 0; i < NTPSEGMENTS; i++) {
        readseg[i] = 0;
    }

    /* grab all segments, keep the non-null ones */
    for (i = 0; i < NTPSEGMENTS; i++) {
	    segments[i] = shm_get(i << 2, 0);
	    if (verbose && segments[i] != NULL) {
	        fprintf(stderr, "unit %u opened\n", i);
            readseg[i] = 1;
        }
    }

    printf("ntpshmmon version 1\n");
    printf("#      Name   Seen@                Clock                Real               L Prec\n");

    do {
	    struct shm_stat_t	shm_stat;
	    for (i = 0; i < NTPSEGMENTS; i++) {
            if (readseg[i]) {
	        enum segstat_t status = ntp_read(segments[i], &shm_stat, 0);
	        if (verbose)
		        fprintf(stderr, "unit %u status %d\n", i, status);
	        switch(status)
	        {
	        case OK:
		        if (timespec_diff_ns(shm_stat.tvc, tick[i]) >= cycle * 1000000000) {
		            printf("sample %s %ld.%09ld %ld.%09ld %ld.%09ld %d %3d\n", ntp_name(i),
			               (long)shm_stat.tvc.tv_sec, shm_stat.tvc.tv_nsec,
			               (long)shm_stat.tvr.tv_sec, shm_stat.tvr.tv_nsec,
			               (long)shm_stat.tvt.tv_sec, shm_stat.tvt.tv_nsec,
			               shm_stat.leap, shm_stat.precision);
		            tick[i] = shm_stat.tvc;
		            --nsamples;
		        }
		        break;
	        case NO_SEGMENT:
		        break;
		    /* do nothing, data not ready, wait another cycle */
	        case NOT_READY:
		        break;
	        case BAD_MODE:
		        fprintf(stderr, "ntpshmmon: unknown mode %d on segment %s\n", shm_stat.status, ntp_name(i));
		        break;
		    /* do nothing, data is corrupt, wait another cycle */
	        case CLASH:
		        break;
	        default:
		        fprintf(stderr, "ntpshmmon: unknown status %d on segment %s\n", status, ntp_name(i));
		        break;
	        }
        }
	    }

	    /*
	     * Even on a 1 Hz PPS, a sleep(1) may end up being sleep(1.1) and missing a beat.
         * Since we're ignoring duplicates via timestamp, polling at interval < 1 sec shouldn't be a problem.
	     */
	    usleep(cycle * 1000);
    } while(nsamples != 0 && time(NULL) - starttime < timeout);
    return 0;
}

