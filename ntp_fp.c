/*
 * dofptoa - do the grunge work to convert an fp number to ascii
 */

#include <stdint.h>
#include <string.h>
#include "ntp_fp.h"
#include "lib_strbuf.h"
#include "ntp_stdlib.h"

/*
 * Powers of 10
 */
static uint32_t ten_to_the_n[10] = {
	0,
	10,
	100,
	1000,
	10000,
	100000,
	1000000,
	10000000,
	100000000,
	1000000000,
};

char *
dofptoa(u_fp fpv, int neg, short ndec, int msec)
{
	register uint8_t *cp, *cpend;
	register uint32_t val;
	register short dec;
	uint8_t cbuf[12];
	uint8_t *cpdec;
	char *buf;
	char *bp;

	/*
	 * Get a string buffer before starting
	 */
	LIB_GETBUF(buf);

	/*
	 * Zero out the buffer
	 */
	memset((char *)cbuf, 0, sizeof(cbuf));

	/*
	 * Set the pointers to point at the first
	 * decimal place.  Get a local copy of the value.
	 */
	cp = cpend = &cbuf[5];
	val = fpv;

	/*
	 * If we have to, decode the integral part
	 */
	if (!(val & 0xffff0000))
	    cp--;
	else {
		register uint16_t sv = (uint16_t)(val >> 16);
		register uint16_t tmp;
		register uint16_t ten = 10;

		do {
			tmp = sv;
			sv = (uint16_t) (sv/ten);
			*(--cp) = (uint8_t)(tmp - ((sv<<3) + (sv<<1)));
		} while (sv != 0);
	}

	/*
	 * Figure out how much of the fraction to do
	 */
	if (msec) {
		dec = (short)(ndec + 3);
		if (dec < 3)
		    dec = 3;
		cpdec = &cbuf[8];
	} else {
		dec = ndec;
		cpdec = cpend;
	}

	if (dec > 6)
	    dec = 6;
	
	if (dec > 0) {
		do {
			val &= 0xffff;
			val = (val << 3) + (val << 1);
			*cpend++ = (uint8_t)(val >> 16);
		} while (--dec > 0);
	}

	if (val & 0x8000) {
		register uint8_t *tp;
		/*
		 * Round it. Ick.
		 */
		tp = cpend;
		*(--tp) += 1;
		while (*tp >= 10) {
			*tp = 0;
			*(--tp) += 1;
		}
	}

	/*
	 * Remove leading zeroes if necessary
	 */
	while (cp < (cpdec -1) && *cp == 0)
	    cp++;
	
	/*
	 * Copy it into the buffer, asciizing as we go.
	 */
	bp = buf;
	if (neg)
	    *bp++ = '-';
	
	while (cp < cpend) {
		if (cp == cpdec)
		    *bp++ = '.';
		*bp++ = (char)(*cp++ + '0');
	}
	*bp = '\0';
	return buf;
}

char *
dolfptoa(uint32_t fpi, uint32_t fpv, int neg, short ndec, int msec)
{
	register uint8_t *cp, *cpend;
	register uint32_t lwork;
	register int dec;
	uint8_t cbuf[24];
	uint8_t *cpdec;
	char *buf;
	char *bp;

	/*
	 * Get a string buffer before starting
	 */
	LIB_GETBUF(buf);

	/*
	 * Zero the character buffer
	 */
	memset((char *) cbuf, 0, sizeof(cbuf));

	/*
	 * safeguard against sign extensions and other mishaps on 64 bit platforms
	 * the code following is designed for and only for 32-bit inputs and
	 * only 32-bit worth of input are supplied.
         */
	fpi &= 0xffffffff;
	fpv &= 0xffffffff;

	/*
	 * Work on the integral part.  This is biased by what I know
	 * compiles fairly well for a 68000.
	 */
	cp = cpend = &cbuf[10];
	lwork = fpi;
	if (lwork & 0xffff0000) {
		register uint32_t lten = 10;
		register uint32_t ltmp;

		do {
			ltmp = lwork;
			lwork /= lten;
			ltmp -= (lwork << 3) + (lwork << 1);
			*--cp = (uint8_t)ltmp;
		} while (lwork & 0xffff0000);
	}
	if (lwork != 0) {
		register uint16_t sten = 10;
		register uint16_t stmp;
		register uint16_t swork = (uint16_t)lwork;

		do {
			stmp = swork;
			swork = (uint16_t) (swork/sten);
			stmp = (uint16_t)(stmp - ((swork<<3) + (swork<<1)));
			*--cp = (uint8_t)stmp;
		} while (swork != 0);
	}

	/*
	 * Done that, now deal with the problem of the fraction.  First
	 * determine the number of decimal places.
	 */
	if (msec) {
		dec = ndec + 3;
		if (dec < 3)
		    dec = 3;
		cpdec = &cbuf[13];
	} else {
		dec = ndec;
		if (dec < 0)
		    dec = 0;
		cpdec = &cbuf[10];
	}
	if (dec > 12)
	    dec = 12;
	
	/*
	 * If there's a fraction to deal with, do so.
	 */
	if (fpv != 0) {
		l_fp work;

		work.l_ui = 0;
		work.l_uf = fpv;
		while (dec > 0) {
			l_fp ftmp;

			dec--;
			/*
			 * The scheme here is to multiply the
			 * fraction (0.1234...) by ten.  This moves
			 * a junk of BCD into the units part.
			 * record that and iterate.
			 */
			work.l_ui = 0;
			L_LSHIFT(&work);
			ftmp = work;
			L_LSHIFT(&work);
			L_LSHIFT(&work);
			L_ADD(&work, &ftmp);
			*cpend++ = (uint8_t)work.l_ui;
			if (work.l_uf == 0)
			    break;
		}

		/*
		 * Rounding is rotten
		 */
		if (work.l_uf & 0x80000000) {
			register uint8_t *tp = cpend;

			*(--tp) += 1;
			while (*tp >= 10) {
				*tp = 0;
				*(--tp) += 1;
			};
			if (tp < cp)
			    cp = tp;
		}
	}
	cpend += dec;


	/*
	 * We've now got the fraction in cbuf[], with cp pointing at
	 * the first character, cpend pointing past the last, and
	 * cpdec pointing at the first character past the decimal.
	 * Remove leading zeros, then format the number into the
	 * buffer.
	 */
	while (cp < cpdec) {
		if (*cp != 0)
		    break;
		cp++;
	}
	if (cp == cpdec)
	    --cp;

	bp = buf;
	if (neg)
	    *bp++ = '-';
	while (cp < cpend) {
		if (cp == cpdec)
		    *bp++ = '.';
		*bp++ = (char)(*cp++ + '0');	/* ascii dependent? */
	}
	*bp = '\0';

	/*
	 * Done!
	 */
	return buf;
}

char *
fptoa(s_fp fpv, short ndec)
{
	u_fp plusfp;
	int neg;

	if (fpv < 0) {
		plusfp = (u_fp)(-fpv);
		neg = 1;
	} else {
		plusfp = (u_fp)fpv;
		neg = 0;
	}

	return dofptoa(plusfp, neg, ndec, 0);
}

char *
fptoms(s_fp fpv, short ndec)
{
	u_fp plusfp;
	int neg;

	if (fpv < 0) {
		plusfp = (u_fp)(-fpv);
		neg = 1;
	} else {
		plusfp = (u_fp)fpv;
		neg = 0;
	}

	return dofptoa(plusfp, neg, ndec, 1);
}

char *
mfptoa(uint32_t fpi, uint32_t fpf, short ndec)
{
	int isneg;

	if (M_ISNEG(fpi, fpf)) {
		isneg = 1;
		M_NEG(fpi, fpf);
	} else {
	    isneg = 0;
    }

	return dolfptoa(fpi, fpf, isneg, ndec, 0);
}

char *
mfptoms(uint32_t fpi, uint32_t fpf, short ndec)
{
	int isneg;

	if (M_ISNEG(fpi, fpf)) {
		isneg = 1;
		M_NEG(fpi, fpf);
	} else {
	    isneg = 0;
    }

	return dolfptoa(fpi, fpf, isneg, ndec, 1);
}

/*
 * atolfp - convert an ascii string to an l_fp number
 */
int atolfp(const char *str, l_fp *lfp)
{
	register const unsigned char *cp;
	register uint32_t dec_i;
	register uint32_t dec_f;
    unsigned char ind;
	int ndec;
	int isneg;

	isneg = 0;
	dec_i = dec_f = 0;
	ndec = 0;
	cp = (const unsigned char*)str;

	/*
	 * We understand numbers of the form:
	 *
	 * [spaces][-|+][digits][.][digits][spaces|\n|\0]
	 */
	while (ntp_isspace((int)*cp))
	    cp++;
	
	if (*cp == '-') {
		cp++;
		isneg = 1;
	}
	
	if (*cp == '+')
	    cp++;

	if (*cp != '.' && !ntp_isdigit((int)*cp))
	    return 0;

	while (*cp != '\0' && (ind = (*cp - 0x30)) <= 9) {
		dec_i = (dec_i << 3) + (dec_i << 1);	/* multiply by 10 */
		dec_i += ind;
		cp++;
	}

	if (*cp != '\0' && !ntp_isspace((int)*cp)) {
		if (*cp++ != '.')
		    return 0;
	
		while (ndec < 9 && *cp != '\0' && (ind = (*cp - 0x30)) <= 9) {    
			ndec++;
			dec_f = (dec_f << 3) + (dec_f << 1);	/* *10 */
			dec_f += ind;
			cp++;
		}

		while (ntp_isdigit((int)*cp))
		    cp++;
		
		if (*cp != '\0' && !ntp_isspace((int)*cp))
		    return 0;
	}

	if (ndec > 0) {
		register uint32_t tmp;
		register uint32_t bit;
		register uint32_t ten_fact;

		ten_fact = ten_to_the_n[ndec];

		tmp = 0;
		bit = 0x80000000;
		while (bit != 0) {
			dec_f <<= 1;
			if (dec_f >= ten_fact) {
				tmp |= bit;
				dec_f -= ten_fact;
			}
			bit >>= 1;
		}
		if ((dec_f << 1) > ten_fact)
		    tmp++;
		dec_f = tmp;
	}

	if (isneg)
	    M_NEG(dec_i, dec_f);
	
	lfp->l_ui = dec_i;
	lfp->l_uf = dec_f;
	return 1;
}

/*
 * hextolfp - convert an ascii hex string to an l_fp number
 */
int hextolfp(const char *str, l_fp *lfp)
{
	register const char *cp;
	register const char *cpstart;
	register uint32_t dec_i;
	register uint32_t dec_f;
	char *ind = NULL;
	static const char *digits = "0123456789abcdefABCDEF";

	dec_i = dec_f = 0;
	cp = str;

	/*
	 * We understand numbers of the form:
	 *
	 * [spaces]8_hex_digits[.]8_hex_digits[spaces|\n|\0]
	 */
	while (ntp_isspace((int)*cp))
	    cp++;
	
	cpstart = cp;
	while (*cp != '\0' && (cp - cpstart) < 8 &&
	       (ind = strchr(digits, *cp)) != NULL) {
		dec_i = dec_i << 4;	/* multiply by 16 */
		dec_i += ((ind - digits) > 15) ? (ind - digits) - 6
			: (ind - digits);
		cp++;
	}

	if ((cp - cpstart) < 8 || ind == NULL)
	    return 0;
	if (*cp == '.')
	    cp++;

	cpstart = cp;
	while (*cp != '\0' && (cp - cpstart) < 8 &&
	       (ind = strchr(digits, *cp)) != NULL) {
		dec_f = dec_f << 4;	/* multiply by 16 */
		dec_f += ((ind - digits) > 15) ? (ind - digits) - 6
			: (ind - digits);
		cp++;
	}

	if ((cp - cpstart) < 8 || ind == NULL)
	    return 0;
	
	if (*cp != '\0' && !ntp_isspace((int)*cp))
	    return 0;

	lfp->l_ui = dec_i;
	lfp->l_uf = dec_f;
	return 1;
}

/*
 * mstolfp - convert an ascii string in milliseconds to an l_fp number
 */
int mstolfp(const char *str, l_fp *lfp)
{
	register const char *cp;
	register char *bp;
	register const char *cpdec;
	char buf[100];

	/*
	 * We understand numbers of the form:
	 *
	 * [spaces][-][digits][.][digits][spaces|\n|\0]
	 *
	 * This is one enormous hack.  Since I didn't feel like
	 * rewriting the decoding routine for milliseconds, what
	 * is essentially done here is to make a copy of the string
	 * with the decimal moved over three places so the seconds
	 * decoding routine can be used.
	 */
	bp = buf;
	cp = str;
	while (ntp_isspace((int)*cp))
	    cp++;
	
	if (*cp == '-') {
		*bp++ = '-';
		cp++;
	}

	if (*cp != '.' && !ntp_isdigit((int)*cp))
	    return 0;


	/*
	 * Search forward for the decimal point or the end of the string.
	 */
	cpdec = cp;
	while (ntp_isdigit((int)*cpdec))
	    cpdec++;

	/*
	 * Found something.  If we have more than three digits copy the
	 * excess over, else insert a leading 0.
	 */
	if ((cpdec - cp) > 3) {
		do {
			*bp++ = (char)*cp++;
		} while ((cpdec - cp) > 3);
	} else {
		*bp++ = '0';
	}

	/*
	 * Stick the decimal in.  If we've got less than three digits in
	 * front of the millisecond decimal we insert the appropriate number
	 * of zeros.
	 */
	*bp++ = '.';
	if ((cpdec - cp) < 3) {
		register int i = 3 - (cpdec - cp);

		do {
			*bp++ = '0';
		} while (--i > 0);
	}

	/*
	 * Copy the remainder up to the millisecond decimal.  If cpdec
	 * is pointing at a decimal point, copy in the trailing number too.
	 */
	while (cp < cpdec)
	    *bp++ = (char)*cp++;
	
	if (*cp == '.') {
		cp++;
		while (ntp_isdigit((int)*cp))
		    *bp++ = (char)*cp++;
	}
	*bp = '\0';

	/*
	 * Check to make sure the string is properly terminated.  If
	 * so, give the buffer to the decoding routine.
	 */
	if (*cp != '\0' && !ntp_isspace((int)*cp))
	    return 0;
	return atolfp(buf, lfp);
}

#define LOW_MASK  (uint32_t)((1<<(FRACTION_PREC/2))-1)
#define HIGH_MASK (uint32_t)(LOW_MASK << (FRACTION_PREC/2))

/*
 * for those who worry about overflows (possibly triggered by static analysis tools):
 *
 * Largest value of a 2^n bit number is 2^n-1.
 * Thus the result is: (2^n-1)*(2^n-1) = 2^2n - 2^n - 2^n + 1 < 2^2n
 * Here overflow can not happen for 2 reasons:
 * 1) the code actually multiplies the absolute values of two signed
 *    64bit quantities.thus effectively multiplying 2 63bit quantities.
 * 2) Carry propagation is from low to high, building principle is
 *    addition, so no storage for the 2^2n term from above is needed.
 */

void
mfp_mul(int32_t *o_i, uint32_t *o_f, int32_t a_i, uint32_t a_f, int32_t b_i, uint32_t b_f)
{
  int32_t i, j;
  uint32_t  f;
  uint32_t a[4];			/* operand a */
  uint32_t b[4];			/* operand b */
  uint32_t c[5];			/* result c - 5 items for performance - see below */
  uint32_t carry;
  
  int neg = 0;

  if (a_i < 0) {			/* examine sign situation */
      neg = 1;
      M_NEG(a_i, a_f);
  }

  if (b_i < 0) {			/* examine sign situation */
      neg = !neg;
      M_NEG(b_i, b_f);
  } 
 
  a[0] = a_f & LOW_MASK;	/* prepare a operand */
  a[1] = (a_f & HIGH_MASK) >> (FRACTION_PREC/2);
  a[2] = a_i & LOW_MASK;
  a[3] = (a_i & HIGH_MASK) >> (FRACTION_PREC/2);
  
  b[0] = b_f & LOW_MASK;	/* prepare b operand */
  b[1] = (b_f & HIGH_MASK) >> (FRACTION_PREC/2);
  b[2] = b_i & LOW_MASK;
  b[3] = (b_i & HIGH_MASK) >> (FRACTION_PREC/2);

  c[0] = c[1] = c[2] = c[3] = c[4] = 0;

  for (i = 0; i < 4; i++)	/* we do assume 32 * 32 = 64 bit multiplication */
    for (j = 0; j < 4; j++) {
	  uint32_t result_low, result_high;
	  int low_index = (i+j)/2;      /* formal [0..3]  - index for low long word */
	  int mid_index = 1+low_index;  /* formal [1..4]! - index for high long word
					                   will generate unecessary add of 0 to c[4]
					                   but save 15 'if (result_high) expressions' */
	  int high_index = 1+mid_index; /* formal [2..5]! - index for high word overflow
					                   - only assigned on overflow (limits range to 2..3) */

	  result_low = (uint32_t)a[i] * (uint32_t)b[j];	/* partial product */
	  if ((i+j) & 1) {		/* splits across two result registers */
	    result_high   = result_low >> (FRACTION_PREC/2);
	    result_low  <<= FRACTION_PREC/2;
	    carry         = (unsigned)1<<(FRACTION_PREC/2);
	  } else {			    /* stays in a result register - except for overflows */
	    result_high = 0;
	    carry       = 1;
      }

	  if (((c[low_index] >> 1) + (result_low >> 1) + ((c[low_index] & result_low & carry) != 0)) &
	    (uint32_t)((unsigned)1<<(FRACTION_PREC - 1))) {
	    result_high++;	/* propagate overflows */
      }

	  c[low_index]   += result_low; /* add up partial products */
	  if (((c[mid_index] >> 1) + (result_high >> 1) + ((c[mid_index] & result_high & 1) != 0)) &
	      (uint32_t)((unsigned)1<<(FRACTION_PREC - 1))) {
	        c[high_index]++;		/* propagate overflows of high word sum */
      }

	  c[mid_index] += result_high;  /* will add a 0 to c[4] once but saves 15 if conditions */
    }

  if (c[3]) {		/* overflow */
      i = ((unsigned)1 << (FRACTION_PREC-1)) - 1;
      f = ~(unsigned)0;
  } else { /* take produkt - discarding extra precision */
      i = c[2];
      f = c[1];
  } 
 
  if (neg) {		    /* recover sign */
      M_NEG(i, f);
  }

  *o_i = i;
  *o_f = f;
}

