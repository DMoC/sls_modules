/* -*- c++ -*-
 *
 * SOCLIB_LGPL_HEADER_BEGIN
 * 
 * This file is part of SoCLib, GNU LGPLv2.1.
 * 
 * SoCLib is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * SoCLib is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with SoCLib; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 * 
 * SOCLIB_LGPL_HEADER_END
 *
 * Copyright (c) TIMA
 *          Patrice Gerin < patrice.gerin@imag.fr >
 *
 * Maintainers: Patrice Gerin
 */#ifndef IDCT_H
#define IDCT_H

#include <stdint.h>

/*
 * Minimum and maximum values a `signed int' can hold.  
 */

#define	IDCT_INT_MIN	(- IDCT_INT_MAX - 1)
#define	IDCT_INT_MAX	2147483647

/*
 * Useful constants: 
 */

/*
 * ck = cos(k*pi/16) = s8-k = sin((8-k)*pi/16) times 1 << C_BITS and
 * rounded 
 */
#define c0_1  16384
#define c0_s2 23170
#define c1_1  16069
#define c1_s2 22725
#define c2_1  15137
#define c2_s2 21407
#define c3_1  13623
#define c3_s2 19266
#define c4_1  11585
#define c4_s2 16384
#define c5_1  9102
#define c5_s2 12873
#define c6_1  6270
#define c6_s2 8867
#define c7_1  3196
#define c7_s2 4520
#define c8_1  0
#define c8_s2 0
#define sqrt2 c0_s2

#define Y(i,j)          Y[(i << 3) + j]
#define X(i,j)          (output[(i << 3) + j])

/*
 * The number of bits of accuracy in all (signed) integer operations: May
 * lie between 1 and 32 (bounds inclusive). 
 */

#define ARITH_BITS      16

/*
 * The minimum signed integer value that fits in ARITH_BITS: 
 */

#define ARITH_MIN       (-1 << (ARITH_BITS-1))

/*
 * The maximum signed integer value that fits in ARITH_BITS: 
 */

#define ARITH_MAX       (~ARITH_MIN)

/*
 * The number of bits coefficients are scaled up before 2-D IDCT: 
 */

#define S_BITS           3

/*
 * The number of bits in the fractional part of a fixed point constant: 
 */

#define C_BITS          14

#define SCALE(x,n)      ((x) << (n))

/*
 * Butterfly: but(a,b,x,y) = rot(sqrt(2),4,a,b,x,y) 
 */

#define but(a,b,x,y)    { x = SUB(a,b); y = ADD(a,b); }

/*
 * This version is vital in passing overall mean error test. 
 */

static inline int32_t DESCALE (int32_t x, int32_t n) {
	return (x + (1 << (n - 1)) - (x < 0)) >> n;
}

/*
 * Maximum and minimum intermediate int values: 
 */

static inline int32_t ADD(int32_t x, int32_t y) {
	int32_t r = x + y;

	return r;			/* in effect: & 0x0000FFFF */
}

static inline int32_t SUB(int32_t x, int32_t y) {
	int32_t r = x - y;

	return r;			/* in effect: & 0x0000FFFF */
}

static inline int32_t CMUL(int32_t c, int32_t x) {
	int32_t r = c * x;
	
	/*
	 * Less accurate rounding here also works fine: 
	 */
	
	r = (r + (1 << (C_BITS - 1))) >> C_BITS;
	return r;
}

/*
 * Rotate (x,y) over angle k*pi/16 (counter-clockwise) and scale with f. 
 */

static inline void rot(int32_t f, int32_t k, int32_t x, int32_t y, int32_t *rx, int32_t *ry) {
	int32_t COS[2][8] = {
		{c0_1, c1_1, c2_1, c3_1, c4_1, c5_1, c6_1, c7_1},
		{c0_s2, c1_s2, c2_s2, c3_s2, c4_s2, c5_s2, c6_s2, c7_s2}
	};

	*rx = SUB(CMUL(COS[f][k], x), CMUL(COS[f][8 - k], y));
	*ry = ADD(CMUL(COS[f][8 - k], x), CMUL(COS[f][k], y));
}

/*
 * Inverse 1-D Discrete Cosine Transform. Result Y is scaled up by factor
 * sqrt(8). Original Loeffler algorithm. 
 */

static inline void idct_1d(int32_t *Y) {
	int32_t z1[8], z2[8], z3[8];

	/*
	 * Stage 1: 
	 */

	but(Y[0], Y[4], z1[1], z1[0]);
	rot(1, 6, Y[2], Y[6], &z1[2], &z1[3]);
	but(Y[1], Y[7], z1[4], z1[7]);
	z1[5] = CMUL(sqrt2, Y[3]);
	z1[6] = CMUL(sqrt2, Y[5]);

	/*
	 * Stage 2: 
	 */
	but(z1[0], z1[3], z2[3], z2[0]);
	but(z1[1], z1[2], z2[2], z2[1]);
	but(z1[4], z1[6], z2[6], z2[4]);
	but(z1[7], z1[5], z2[5], z2[7]);

	/*
	 * Stage 3: 
	 */
	z3[0] = z2[0];
	z3[1] = z2[1];
	z3[2] = z2[2];
	z3[3] = z2[3];
	rot(0, 3, z2[4], z2[7], &z3[4], &z3[7]);
	rot(0, 1, z2[5], z2[6], &z3[5], &z3[6]);

	/*
	 * Final stage 4: 
	 */
	but(z3[0], z3[7], Y[7], Y[0]);
	but(z3[1], z3[6], Y[6], Y[1]);
	but(z3[2], z3[5], Y[5], Y[2]);
	but(z3[3], z3[4], Y[4], Y[3]);
}

/*
 * Inverse 2-D Discrete Cosine Transform. 
 */

static inline void IDCT(int32_t * input, uint8_t * output) {
	int32_t Y[64];
	int32_t k, l;

	for (k = 0; k < 8; k++) {	
		for (l = 0; l < 8; l++)	Y(k, l) = SCALE(input[(k << 3) + l], S_BITS);
		idct_1d(&Y(k, 0));
	}

	for (l = 0; l < 8; l++) {
		int32_t Yc[8];

		for (k = 0; k < 8; k++) Yc[k] = Y(k, l);
		
		idct_1d(Yc);
		
		for (k = 0; k < 8; k++) {
			int32_t r = 128 + DESCALE(Yc[k], S_BITS + 3);
			X(k, l) = r;
		}
	}
}

#endif
