/*
 * atouint - convert an ascii string to an unsigned int, with error checking
 */
#include <sys/types.h>
#include "ntp_fp.h"

int atouint(const char *str, uint32_t *uval)
{
	register uint32_t u;
	register const char *cp;

	cp = str;
	if (*cp == '\0')
	    return 0;

	u = 0;
	while (*cp != '\0') {
		if (!ntp_isdigit((int)*cp))
		    return 0;
		if (u > 429496729 || (u == 429496729 && *cp >= '6'))
		    return 0;	/* overflow */
		u = (u << 3) + (u << 1);
		u += *cp++ - '0';	/* ascii dependent */
	}

	*uval = u;
	return 1;
}

int atoint(const char *str, int32_t *ival)
{
    register int32_t u;
    register const char *cp;
    register int isneg;
    register int oflow_digit;

    cp = str;

    if (*cp == '-') {
        cp++;
        isneg = 1;
        oflow_digit = '8';
    } else {
        isneg = 0;
        oflow_digit = '7';
    }

    if (*cp == '\0')
        return 0;

    u = 0;
    while (*cp != '\0') {
        if (!ntp_isdigit((int)*cp))
            return 0;
        if (u > 214748364 || (u == 214748364 && *cp > oflow_digit))
            return 0;   /* overflow */
        u = (u << 3) + (u << 1);
        u += *cp++ - '0';   /* ascii dependent */
    }

    if (isneg)
        *ival = -u;
    else
        *ival = u;
    return 1;
}

int hextoint(const char *str, uint32_t *pu)
{
    register uint32_t u;
    register const char *cp;

    cp = str;

    if (*cp == '\0')
        return 0;

    u = 0;
    while (*cp != '\0') {
        if (!ntp_isxdigit(*cp))
            return 0;
        if (u & 0xF0000000)
            return 0;   /* overflow */
        u <<= 4;
        if ('0' <= *cp && *cp <= '9')
            u += *cp++ - '0';
        else if ('a' <= *cp && *cp <= 'f')
            u += *cp++ - 'a' + 10;
        else if ('A' <= *cp && *cp <= 'F')
            u += *cp++ - 'A' + 10;
        else
            return 0;
    }
    *pu = u;
    return 1;
}

unsigned int itoa10(char *__restrict bufend, uint32_t uval)
{
    unsigned int digit;
    unsigned int i = 0, j = 0;
    unsigned char _buf[24];
    do {
        digit = uval % 10;
        uval /= 10;
        _buf[i++] = (digit + '0');
    } while (uval);
    while(i-- > 0) { bufend[j++] = _buf[i]; }
    bufend[j] = '\0';
    return j;
}

unsigned int itoahex(char *__restrict bufend, uint32_t uval, char alphacase)
{
    static const char hex[]="0123456789abcdef";
    unsigned int digit;
    unsigned int i = 0, j = 0;
    unsigned char _buf[24];
    do {
        digit = uval & 15;
        uval >>= 4;
        _buf[i++] = hex[digit];
    } while (uval);
    while(i-- > 0) { bufend[j++] = _buf[i]|alphacase; }
    bufend[j] = '\0';
    return j;
}

