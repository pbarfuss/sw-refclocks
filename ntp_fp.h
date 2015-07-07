/*
 * ntp_fp.h - definitions for NTP fixed/floating-point arithmetic
 */

#ifndef NTP_FP_H
#define NTP_FP_H

#include <stdint.h>
#define ntp_isspace(c) \
    ((sizeof(c) == sizeof(char)) \
     ? ((((c) == ' ') || (((unsigned char)((c) - 9)) <= (13 - 9)))) \
     : ((((c) == ' ') || (((unsigned int)((c) - 9)) <= (13 - 9)))))
#define ntp_isdigit(c) \
    ((sizeof(c) == sizeof(char)) \
     ? (((unsigned char)((c) - '0')) < 10) \
     : (((unsigned int)((c) - '0')) < 10))
#define ntp_isxdigit(c) \
    (ntp_isdigit(c) \
     || ((sizeof(c) == sizeof(char)) \
         ? (((unsigned char)((((c)) | 0x20) - 'a')) < 6) \
         : (((unsigned int)((((c)) | 0x20) - 'a')) < 6)))

/*
 * Time of day conversion constant.  Ntp's time scale starts in 1900,
 * Windows NT in 1601, Unix in 1970.
 */
#define JAN_1601    0x232661280ULL /* 9435484800 1900 - 1601 in seconds */
#define JAN_1970    0x83AA7E80  /* 2208988800 1970 - 1900 in seconds */

/*
 * NTP uses two fixed point formats.  The first (l_fp) is the "long"
 * format and is 64 bits long with the decimal between bits 31 and 32.
 * This is used for time stamps in the NTP packet header (in network
 * byte order) and for internal computations of offsets (in local host
 * byte order). We use the same structure for both signed and unsigned
 * values, which is a big hack but saves rewriting all the operators
 * twice. Just to confuse this, we also sometimes just carry the
 * fractional part in calculations, in both signed and unsigned forms.
 * Anyway, an l_fp looks like:
 *
 *    0			  1		      2			  3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |			       Integral Part			     |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |			       Fractional Part			     |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 */
typedef struct {
	union {
		uint32_t Xl_ui;
		int32_t Xl_i;
	} Ul_i;
	union {
		uint32_t Xl_uf;
		int32_t Xl_f;
	} Ul_f;
} l_fp;

#define l_ui	Ul_i.Xl_ui		/* unsigned integral part */
#define	l_i	Ul_i.Xl_i		/* signed integral part */
#define	l_uf	Ul_f.Xl_uf		/* unsigned fractional part */
#define	l_f	Ul_f.Xl_f		/* signed fractional part */

/*
 * Fractional precision (of an l_fp) is actually the number of
 * bits in a long.
 */
#define	FRACTION_PREC	(32)


/*
 * The second fixed point format is 32 bits, with the decimal between
 * bits 15 and 16.  There is a signed version (s_fp) and an unsigned
 * version (u_fp).  This is used to represent synchronizing distance
 * and synchronizing dispersion in the NTP packet header (again, in
 * network byte order) and internally to hold both distance and
 * dispersion values (in local byte order).  In network byte order
 * it looks like:
 *
 *    0			  1		      2			  3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |		  Integer Part	     |	   Fraction Part	     |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 */
typedef int32_t s_fp;
typedef uint32_t u_fp;

/*
 * A unit second in fp format.	Actually 2**(half_the_bits_in_a_long)
 */
#define	FP_SECOND	(0x10000)

/*
 * Byte order conversions
 */
#define	HTONS_FP(x)	(htonl(x))
#define	HTONL_FP(h, n)	do { (n)->l_ui = htonl((h)->l_ui); \
			     (n)->l_uf = htonl((h)->l_uf); } while (0)
#define	NTOHS_FP(x)	(ntohl(x))
#define	NTOHL_FP(n, h)	do { (h)->l_ui = ntohl((n)->l_ui); \
			     (h)->l_uf = ntohl((n)->l_uf); } while (0)
#define	NTOHL_MFP(ni, nf, hi, hf) \
	do { (hi) = ntohl(ni); (hf) = ntohl(nf); } while (0)
#define	HTONL_MFP(hi, hf, ni, nf) \
	do { (ni) = htonl(hi); (nf) = htonl(hf); } while (0)

/* funny ones.	Converts ts fractions to net order ts */
#define	HTONL_UF(uf, nts) \
	do { (nts)->l_ui = 0; (nts)->l_uf = htonl(uf); } while (0)
#define	HTONL_F(f, nts) do { (nts)->l_uf = htonl(f); \
				if ((f) & 0x80000000) \
					(nts)->l_i = -1; \
				else \
					(nts)->l_i = 0; \
			} while (0)

/*
 * Conversions between the two fixed point types
 */
#define	MFPTOFP(x_i, x_f)	(((x_i) >= 0x00010000) ? 0x7fffffff : \
				(((x_i) <= -0x00010000) ? 0x80000000 : \
				(((x_i)<<16) | (((x_f)>>16)&0xffff))))
#define	LFPTOFP(v)		MFPTOFP((v)->l_i, (v)->l_f)

#define UFPTOLFP(x, v) ((v)->l_ui = (u_fp)(x)>>16, (v)->l_uf = (x)<<16)
#define FPTOLFP(x, v)  (UFPTOLFP((x), (v)), (x) < 0 ? (v)->l_ui -= 0x10000 : 0)

#define MAXLFP(v) ((v)->l_ui = 0x7fffffff, (v)->l_uf = 0xffffffff)
#define MINLFP(v) ((v)->l_ui = 0x80000000, (v)->l_uf = 0)

/*
 * Primitive operations on long fixed point values.  If these are
 * reminiscent of assembler op codes it's only because some may
 * be replaced by inline assembler for particular machines someday.
 * These are the (kind of inefficient) run-anywhere versions.
 */
#define	M_NEG(v_i, v_f)		/* v = -v */ \
	do { \
		if ((v_f) == 0) \
			(v_i) = -((s_fp)(v_i)); \
		else { \
			(v_f) = -((s_fp)(v_f)); \
			(v_i) = ~(v_i); \
		} \
	} while(0)

#define	M_NEGM(r_i, r_f, a_i, a_f)	/* r = -a */ \
	do { \
		if ((a_f) == 0) { \
			(r_f) = 0; \
			(r_i) = -(a_i); \
		} else { \
			(r_f) = -(a_f); \
			(r_i) = ~(a_i); \
		} \
	} while(0)

#define M_ADD(r_i, r_f, a_i, a_f)	/* r += a */ \
	do { \
		register uint32_t lo_tmp; \
		register uint32_t hi_tmp; \
		\
		lo_tmp = ((r_f) & 0xffff) + ((a_f) & 0xffff); \
		hi_tmp = (((r_f) >> 16) & 0xffff) + (((a_f) >> 16) & 0xffff); \
		if (lo_tmp & 0x10000) \
			hi_tmp++; \
		(r_f) = ((hi_tmp & 0xffff) << 16) | (lo_tmp & 0xffff); \
		\
		(r_i) += (a_i); \
		if (hi_tmp & 0x10000) \
			(r_i)++; \
	} while (0)

#define M_ADD3(r_ovr, r_i, r_f, a_ovr, a_i, a_f) /* r += a, three word */ \
	do { \
		register uint32_t lo_tmp; \
		register uint32_t hi_tmp; \
		\
		lo_tmp = ((r_f) & 0xffff) + ((a_f) & 0xffff); \
		hi_tmp = (((r_f) >> 16) & 0xffff) + (((a_f) >> 16) & 0xffff); \
		if (lo_tmp & 0x10000) \
			hi_tmp++; \
		(r_f) = ((hi_tmp & 0xffff) << 16) | (lo_tmp & 0xffff); \
		\
		lo_tmp = ((r_i) & 0xffff) + ((a_i) & 0xffff); \
		if (hi_tmp & 0x10000) \
			lo_tmp++; \
		hi_tmp = (((r_i) >> 16) & 0xffff) + (((a_i) >> 16) & 0xffff); \
		if (lo_tmp & 0x10000) \
			hi_tmp++; \
		(r_i) = ((hi_tmp & 0xffff) << 16) | (lo_tmp & 0xffff); \
		\
		(r_ovr) += (a_ovr); \
		if (hi_tmp & 0x10000) \
			(r_ovr)++; \
	} while (0)

#define M_SUB(r_i, r_f, a_i, a_f)	/* r -= a */ \
	do { \
		register uint32_t lo_tmp; \
		register uint32_t hi_tmp; \
		\
		if ((a_f) == 0) { \
			(r_i) -= (a_i); \
		} else { \
			lo_tmp = ((r_f) & 0xffff) + ((-((s_fp)(a_f))) & 0xffff); \
			hi_tmp = (((r_f) >> 16) & 0xffff) \
			    + (((-((s_fp)(a_f))) >> 16) & 0xffff); \
			if (lo_tmp & 0x10000) \
				hi_tmp++; \
			(r_f) = ((hi_tmp & 0xffff) << 16) | (lo_tmp & 0xffff); \
			\
			(r_i) += ~(a_i); \
			if (hi_tmp & 0x10000) \
				(r_i)++; \
		} \
	} while (0)

#define	M_RSHIFTU(v_i, v_f)		/* v >>= 1, v is unsigned */ \
	do { \
		(v_f) = (uint32_t)(v_f) >> 1; \
		if ((v_i) & 01) \
			(v_f) |= 0x80000000; \
		(v_i) = (uint32_t)(v_i) >> 1; \
	} while (0)

#define	M_RSHIFT(v_i, v_f)		/* v >>= 1, v is signed */ \
	do { \
		(v_f) = (uint32_t)(v_f) >> 1; \
		if ((v_i) & 01) \
			(v_f) |= 0x80000000; \
		if ((v_i) & 0x80000000) \
			(v_i) = ((v_i) >> 1) | 0x80000000; \
		else \
			(v_i) = (v_i) >> 1; \
	} while (0)

#define	M_LSHIFT(v_i, v_f)		/* v <<= 1 */ \
	do { \
		(v_i) <<= 1; \
		if ((v_f) & 0x80000000) \
			(v_i) |= 0x1; \
		(v_f) <<= 1; \
	} while (0)

#define	M_LSHIFT3(v_ovr, v_i, v_f)	/* v <<= 1, with overflow */ \
	do { \
		(v_ovr) <<= 1; \
		if ((v_i) & 0x80000000) \
			(v_ovr) |= 0x1; \
		(v_i) <<= 1; \
		if ((v_f) & 0x80000000) \
			(v_i) |= 0x1; \
		(v_f) <<= 1; \
	} while (0)

#define	M_ADDUF(r_i, r_f, uf)		/* r += uf, uf is uint32_t fraction */ \
	M_ADD((r_i), (r_f), 0, (uf))	/* let optimizer worry about it */

#define	M_SUBUF(r_i, r_f, uf)		/* r -= uf, uf is uint32_t fraction */ \
	M_SUB((r_i), (r_f), 0, (uf))	/* let optimizer worry about it */

#define	M_ADDF(r_i, r_f, f)		/* r += f, f is a int32_t fraction */ \
	do { \
		if ((f) > 0) \
			M_ADD((r_i), (r_f), 0, (f)); \
		else if ((f) < 0) \
			M_ADD((r_i), (r_f), (-1), (f));\
	} while(0)

#define	M_ISNEG(v_i, v_f)		/* v < 0 */ \
	(((v_i) & 0x80000000) != 0)

#define	M_ISHIS(a_i, a_f, b_i, b_f)	/* a >= b unsigned */ \
	(((uint32_t)(a_i)) > ((uint32_t)(b_i)) || \
	  ((a_i) == (b_i) && ((uint32_t)(a_f)) >= ((uint32_t)(b_f))))

#define	M_ISGEQ(a_i, a_f, b_i, b_f)	/* a >= b signed */ \
	(((int32_t)(a_i)) > ((int32_t)(b_i)) || \
	  ((a_i) == (b_i) && ((uint32_t)(a_f)) >= ((uint32_t)(b_f))))

#define	M_ISEQU(a_i, a_f, b_i, b_f)	/* a == b unsigned */ \
	((a_i) == (b_i) && (a_f) == (b_f))

/*
 * Operations on the long fp format
 */
#define	L_ADD(r, a)	M_ADD((r)->l_ui, (r)->l_uf, (a)->l_ui, (a)->l_uf)
#define	L_SUB(r, a)	M_SUB((r)->l_ui, (r)->l_uf, (a)->l_ui, (a)->l_uf)
#define	L_NEG(v)	M_NEG((v)->l_ui, (v)->l_uf)
#define L_ADDUF(r, uf)	M_ADDUF((r)->l_ui, (r)->l_uf, (uf))
#define L_SUBUF(r, uf)	M_SUBUF((r)->l_ui, (r)->l_uf, (uf))
#define	L_ADDF(r, f)	M_ADDF((r)->l_ui, (r)->l_uf, (f))
#define	L_RSHIFT(v)	M_RSHIFT((v)->l_i, (v)->l_uf)
#define	L_RSHIFTU(v)	M_RSHIFTU((v)->l_ui, (v)->l_uf)
#define	L_LSHIFT(v)	M_LSHIFT((v)->l_ui, (v)->l_uf)
#define	L_CLR(v)	((v)->l_ui = (v)->l_uf = 0)

#define	L_ISNEG(v)	(((v)->l_ui & 0x80000000) != 0)
#define L_ISZERO(v)	((v)->l_ui == 0 && (v)->l_uf == 0)
#define	L_ISHIS(a, b)	((a)->l_ui > (b)->l_ui || \
			  ((a)->l_ui == (b)->l_ui && (a)->l_uf >= (b)->l_uf))
#define	L_ISGEQ(a, b)	((a)->l_i > (b)->l_i || \
			  ((a)->l_i == (b)->l_i && (a)->l_uf >= (b)->l_uf))
#define	L_ISEQU(a, b)	M_ISEQU((a)->l_ui, (a)->l_uf, (b)->l_ui, (b)->l_uf)

/*
 * s_fp/double and u_fp/double conversions
 */
#define FRIC		65536.			/* 2^16 as a double */
#define DTOFP(r)	((s_fp)((r) * FRIC))
#define DTOUFP(r)	((u_fp)((r) * FRIC))
#define FPTOD(r)	((double)(r) / FRIC)

/*
 * l_fp/double conversions
 */
#define FRAC		4294967296.		/* 2^32 as a double */
#define M_DTOLFP(d, r_i, r_uf)			/* double to l_fp */ \
	do { \
		register double d_tmp; \
		\
		d_tmp = (d); \
		if (d_tmp < 0) { \
			d_tmp = -d_tmp; \
			(r_i) = (int32_t)(d_tmp); \
			(r_uf) = (uint32_t)(((d_tmp) - (double)(r_i)) * FRAC); \
			M_NEG((r_i), (r_uf)); \
		} else { \
			(r_i) = (int32_t)(d_tmp); \
			(r_uf) = (uint32_t)(((d_tmp) - (double)(r_i)) * FRAC); \
		} \
	} while (0)
#define M_LFPTOD(r_i, r_uf, d)			/* l_fp to double */ \
	do { \
		register l_fp l_tmp; \
		\
		l_tmp.l_i = (r_i); \
		l_tmp.l_f = (r_uf); \
		if (l_tmp.l_i < 0) { \
			M_NEG(l_tmp.l_i, l_tmp.l_uf); \
			(d) = -((double)l_tmp.l_i + ((double)l_tmp.l_uf) / FRAC); \
		} else { \
			(d) = (double)l_tmp.l_i + ((double)l_tmp.l_uf) / FRAC; \
		} \
	} while (0)
#define DTOLFP(d, v)	M_DTOLFP((d), (v)->l_ui, (v)->l_uf)
#define LFPTOD(v, d)	M_LFPTOD((v)->l_ui, (v)->l_uf, (d))

struct calendar {
	uint16_t year;		/* year (A.D.) */
	uint16_t yearday;	/* day of year, 1 = January 1 */
	uint8_t month;		/* month, 1 = January */
	uint8_t monthday;	/* day of month */
};

/*
 * Prototypes
 */
extern	char *dofptoa(u_fp, int, short, int);
extern	char *dolfptoa(uint32_t, uint32_t, int, short, int);

extern	int atolfp(const char *, l_fp *);
extern	char *fptoa(s_fp, short);
extern	char *fptoms(s_fp, short);
extern	int hextolfp(const char *, l_fp *);
extern	void gpstolfp(int, int, unsigned long, l_fp *);
extern	char *mfptoa(uint32_t, uint32_t, short);
extern	char *mfptoms(uint32_t, uint32_t, short);
extern	int	mstolfp(const char *, l_fp *);
extern	void mfp_mul(int32_t *, uint32_t *, int32_t, uint32_t, int32_t, uint32_t);
extern  void caljulian(time_t ntp_day, struct calendar *jt);
extern  uint32_t calyearstart(uint32_t rec_ui);
extern  int ymd2yd(uint32_t, uint32_t, uint32_t);
extern  size_t strlcpy(char *dst, const char *src, size_t siz);

/* emalloc.c */
extern void *emalloc(size_t size);
extern void *erealloc(void *ptr, size_t size);
extern void  efree(void *ptr);
extern char *estrdup(const char *str);

/* ntp_random.c */
extern unsigned int ntp_random(void);
extern void ntp_srandom(unsigned int);

extern	char *prettydate(l_fp *);
extern	char *gmprettydate(l_fp *);
extern	char *uglydate(l_fp *);

extern	void get_systime(l_fp *);
extern	int step_systime(double);
extern	int adj_systime(double);

#define	lfptoa(fpv, ndec)	mfptoa((fpv)->l_ui, (fpv)->l_uf, (ndec))
#define	lfptoms(fpv, ndec)	mfptoms((fpv)->l_ui, (fpv)->l_uf, (ndec))

#define stoa(addr)		socktoa(addr)
#define	ntoa(addr)		stoa(addr)
#define stohost(addr)   stoa(addr)

#define	ufptoa(fpv, ndec)	dofptoa((fpv), 0, (ndec), 0)
#define	ufptoms(fpv, ndec)	dofptoa((fpv), 0, (ndec), 1)
#define	ulfptoa(fpv, ndec)	dolfptoa((fpv)->l_ui, (fpv)->l_uf, 0, (ndec), 0)
#define	ulfptoms(fpv, ndec)	dolfptoa((fpv)->l_ui, (fpv)->l_uf, 0, (ndec), 1)
#define	umfptoa(fpi, fpf, ndec) dolfptoa((fpi), (fpf), 0, (ndec), 0)

#endif /* NTP_FP_H */
