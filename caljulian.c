/*
 * caljulian - determine the Julian date from an NTP time.
 * Updated 2008-11-10 Juergen Perlinger <juergen.perlinger@t-online.de>
 */
#include <stdint.h>
#include <time.h>
#include <sys/types.h>
#include "ntp_fp.h"

/*
 * Start day of NTP time as days past the imaginary date 12/1/1 BC.
 * (This is the beginning of the Christian Era, or BCE.)
 */
#define DAY_NTP_STARTS 693596
//calendar_day = ((calendar_day / 675) >> 7);

void caljulian(time_t calendar_day, struct calendar *jt)
{
	uint32_t n400, n100, n4, n1; /* # of: Gregorian cycles, normal centuries, 4-year cycles, years into a leap year cycle */
	uint32_t sclday;  /* scaled days for month conversion */
    uint32_t isleap;

	/*
	 * Find the day past 1900/01/01 00:00 UTC
	 */
	calendar_day += DAY_NTP_STARTS - 1;	/* convert to days in CE */
    /* 146097: Number of days in each 400-year cycle of the proleptic Gregorian/Julian calendar. */
	n400	 = calendar_day / 146097; /* split off cycles */
	calendar_day %= 146097;
    /*
     * We lose a leap year in years divisible by 100 but not 400.
     * (Otherwise this would be 36525 here!)
     */
	n100	 = calendar_day / 36524;
	calendar_day %= 36524;
	n4	 = calendar_day / 1461; /* 1461: Days in a normal 4 year leap year calendar cycle. */
	calendar_day %= 1461;
	n1	 = calendar_day / 365;
	calendar_day %= 365; /* now zero-based day-of-year */

	/*
	 * Calculate the year and day-of-year
	 */
	jt->year = (400*n400 + 100*n100 + 4*n4 + n1 + 1);

	/* If the cycle year ever comes out to 4, it must be December 31st of a leap year. */
	if ((n100 | n1) > 3) {
        --jt->year;
		jt->yearday = 365;
	} else {
		jt->yearday = calendar_day;
	}

	/*
	 * The following code is according to the excellent book
	 * 'Calendrical Calculations' by Nachum Dershowitz and Edward
	 * Reingold. It converts the day-of-year into month and
	 * day-of-month, using a linear transformation with integer truncation.
	 */
	sclday = calendar_day * 7 + 217;
	isleap  = ((n1 == 3) && ((n4 != 24) || (n100 == 3))) ? 1 : 0;
	if (calendar_day >= (uint32_t)(31 + 28 + isleap))
		sclday += (2 - isleap) * 7;
	jt->month = (uint8_t)((sclday / 214) - 1);
	jt->monthday = (uint8_t)((sclday % 214) / 7 + 1);
}

static const unsigned int days_before[13] = {
  0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
};

time_t mktime_utc (struct calendar *jt)
{
  time_t retval;

  jt->year -= 68;
  retval = (jt->year * 365) + (jt->year >> 2) - 730;
  retval += days_before[jt->month] + jt->monthday - 1;

  if (jt->year % 4 == 0 && jt->month < 2)
    retval -= 1;

  return retval;
}

/*
 * Table to compute the day of year from yyyymmdd timecode.
 */
int ymd2yd(uint32_t y, uint32_t m, uint32_t d)
{
    d += days_before[m];
	if (((y%4 == 0) && (y%100 != 0)) || (y%400 == 0)) {
	    /* leap year */
        if (m == 2) d++;
    }
	return d;
}

uint32_t calyearstart(uint32_t rec_ui)
{
    struct calendar jt;
    uint32_t delta;

    caljulian(rec_ui,&jt);

    /*
    * Now we have days since yearstart (unity-based).
    */
    delta = (uint32_t)(jt.yearday-1);
    delta *= 86400;

    /* The NTP time stamps (l_fp) count seconds unsigned mod 2**32, so we
     * have to calculate this in the proper way!
     */
    return (uint32_t)(rec_ui - delta);
}

