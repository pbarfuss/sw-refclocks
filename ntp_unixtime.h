/*
 * ntp_unixtime.h - contains constants and macros for converting between
 *		    NTP time stamps (l_fp) and Unix times (struct timeval)
 */

#ifndef __NTP_UNIXTIME_H__
#define __NTP_UNIXTIME_H__

#include <stdint.h>
#include <sys/time.h>
#include "ntp_fp.h"

/*
 * These constants are used to round the time stamps computed from
 * a struct timeval to the microsecond (more or less).  This keeps
 * things neat.
 */
#define	TS_MASK		0xfffff000	/* mask to usec, for time stamps */
#define	TS_ROUNDBIT	0x00000800	/* round at this bit */

/*
 * Convert usec to a time stamp fraction.  If you use this the program
 * must include the following declarations:
 */
#ifndef __amd64__
extern uint32_t ustotslo[];
extern uint32_t ustotsmid[];
extern uint32_t ustotshi[];

#define	TVUTOTSF(tvu, tsf) \
	(tsf) = ustotslo[(tvu) & 0xff] \
	    + ustotsmid[((tvu) >> 8) & 0xff] \
	    + ustotshi[((tvu) >> 16) & 0xf]
#else
#define TVUTOTSF(tvu, tsf) ((tsf) = (uint32_t)((((uint64_t)(tvu) << 32) + 500000) / 1000000))
#endif

/*
 * Convert a struct timeval to a time stamp.
 */
#define TVTOTS(tv, ts) \
	do { \
		(ts)->l_ui = (uint32_t)(tv)->tv_sec; \
		TVUTOTSF((tv)->tv_usec, (ts)->l_uf); \
	} while(0)

#define sTVTOTS(tv, ts) \
	do { \
		int isneg = 0; \
		long usec; \
		(ts)->l_ui = (tv)->tv_sec; \
		usec = (tv)->tv_usec; \
		if (((tv)->tv_sec < 0) || ((tv)->tv_usec < 0)) { \
			usec = -usec; \
			(ts)->l_ui = -(ts)->l_ui; \
			isneg = 1; \
		} \
		TVUTOTSF(usec, (ts)->l_uf); \
		if (isneg) { \
			L_NEG((ts)); \
		} \
	} while(0)

/*
 * TV_SHIFT is used to turn the table result into a usec value.  To round,
 * add in TV_ROUNDBIT before shifting
 */
#define	TV_SHIFT	3
#define	TV_ROUNDBIT	0x4

/*
 * Convert a time stamp fraction to microseconds.  The time stamp
 * fraction is assumed to be unsigned.  To use this in a program, declare:
 */
#ifndef __amd64__
extern unsigned char tstouslo[];
extern uint32_t tstousmid[];
extern uint32_t tstoushi[];

#define	TSFTOTVU(tsf, tvu) \
	(tvu) = (tstoushi[((tsf) >> 24) & 0xff] \
	    + tstousmid[((tsf) >> 16) & 0xff] \
	    + tstouslo[((tsf) >> 9) & 0x7f] \
	    + TV_ROUNDBIT) >> TV_SHIFT
#else
#define TSFTOTVU(tsf, tvu) ((tvu) = (int32_t)(((uint64_t)(tsf) * 1000000ULL + 0x80000000ULL) >> 32))
#endif

/*
 * Convert a time stamp to a struct timeval.  The time stamp
 * has to be positive.
 */
#define	TSTOTV(ts, tv) \
	do { \
		(tv)->tv_sec = (ts)->l_ui; \
		TSFTOTVU((ts)->l_uf, (tv)->tv_usec); \
		if ((tv)->tv_usec == 1000000) { \
			(tv)->tv_sec++; \
			(tv)->tv_usec = 0; \
		} \
	} while (0)

/*
 * Convert milliseconds to a time stamp fraction.  This shouldn't be
 * here, but it is convenient since the guys who use the definition will
 * often be including this file anyway.
 */
extern uint32_t msutotsflo[];
extern uint32_t msutotsfhi[];

#define	MSUTOTSF(msu, tsf) \
	(tsf) = msutotsfhi[((msu) >> 5) & 0x1f] + msutotsflo[(msu) & 0x1f]

#endif
