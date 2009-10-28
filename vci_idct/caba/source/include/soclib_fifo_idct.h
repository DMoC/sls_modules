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
 */

#ifndef _SOCLIB_FIFO_IDCT_H
#define _SOCLIB_FIFO_IDCT_H
	
#include <stdio.h>
#include <systemc.h>

#include "fifo/soclib_fifo_interfaces.h"

static inline int DESCALE(int x, int n) __attribute__ ((always_inline));
static inline int ADD(int x, int y) __attribute__ ((always_inline));
static inline int SUB(int x, int y) __attribute__ ((always_inline));
static inline int CMUL(int c, int x) __attribute__ ((always_inline));
static inline void rot(int f, int k, int x, int y, int *rx, int *ry) __attribute__ ((always_inline));
static inline void idct_1d(int *Y) __attribute__ ((always_inline));
void IDCT(unsigned int* input, unsigned char output[8][8]);

template<class T>
class cfifo
{
public:
	cfifo(unsigned int size)
	:	_buffer(NULL),
		_size(size),
		_nb_data(0),
		_in_idx(0),
		_out_idx(0)
	{
		_buffer = new T[_size];
	};
	~cfifo() {};

	bool read(T * data)
	{
		if(_nb_data == 0) return(false);
		*data = _buffer[_out_idx];
		//cout << __func__ << "buffer[" << _out_idx << "]" << (void *)(*data) << endl;
		_out_idx = (_out_idx + 1) % _size;
		_nb_data--;
		return(true);
	};

	bool write(T data)
	{
		if(_nb_data == _size) return(false);
		//cout << __func__ << "buffer[" << _in_idx << "]" << (void *)(data) << endl;
		_buffer[_in_idx] = data;
		_in_idx = (_in_idx + 1) % _size;
		_nb_data++;
		return(true);
	};

	unsigned int status() 
	{
		return _nb_data;
	};

	unsigned int size() 
	{
		return _size;
	};

private:
	T *           _buffer;
	unsigned int  _size;
	unsigned int  _nb_data;
	unsigned int  _in_idx;
	unsigned int  _out_idx;
};


struct SOCLIB_FIFO_IDCT : sc_module {

	//	IO PORTS
	sc_in<bool> 	         CLK;
	sc_in<bool> 	         RESET;
	sc_in<bool> 	         START;
	OUTPUT_FIFO< 4 >         OUTPUT;
   	INPUT_FIFO < 4 >         INPUT;

	unsigned int data_out;
	bool         data_out_valid;
	bool         data_in_valid;

	cfifo < unsigned int > local_fifo_in;
	cfifo < unsigned int > local_fifo_out;

///////////////////////////////////////////////////
//	constructor
///////////////////////////////////////////////////

SC_HAS_PROCESS(SOCLIB_FIFO_IDCT);

SOCLIB_FIFO_IDCT (sc_module_name insname)
	: local_fifo_in(1024),
	  local_fifo_out(1024)
{
	SC_METHOD (transition);
	sensitive_pos << CLK;
	dont_initialize();

	SC_METHOD (genMoore);
	sensitive_neg << CLK;
	dont_initialize();

	data_out_valid = false;
	data_in_valid = false;

}; // end constructor

///////////////////////////////////////////////////////
//      destructor()
//////////////////////////////////////////////////////

~SOCLIB_FIFO_IDCT()
{
}

///////////////////////////////////////////////////////
//	Transition method()
//////////////////////////////////////////////////////

unsigned int mcu_count;
void transition()
{
	unsigned int  received_data[64];
	unsigned char computed_data[8][8];
	unsigned int tmp;

	if(RESET == false) 
		return;

	// If data in present and fifo read is ok
	if (!data_in_valid && (INPUT.ROK == true) && (local_fifo_in.write((unsigned int) INPUT.DATA.read()) == true) )
		data_in_valid = true;
	else 
		data_in_valid = false;


	if(!data_out_valid && (OUTPUT.WOK == true) && (local_fifo_out.read(&data_out) == true) )
		data_out_valid = true; 
	else
		data_out_valid = false; 

	if( (local_fifo_in.status() >= 64) && ((local_fifo_out.size() - local_fifo_out.status()) >= 16) )
	{
		for(int i = 0 ; i < 64 ; i++) local_fifo_in.read(&received_data[i]);
		IDCT(received_data,computed_data);
		for (int i = 0; i < 64; i+=4) 
		{
			tmp  = (((unsigned char*)computed_data)[i+3] << 24) & 0xFF000000;
			tmp |= (((unsigned char*)computed_data)[i+2] << 16) & 0x00FF0000;
			tmp |= (((unsigned char*)computed_data)[i+1] << 8) & 0x0000FF00;
			tmp |=  ((unsigned char*)computed_data)[i+0];
			local_fifo_out.write(tmp);
			//local_fifo_out.write(mcu_count == 0 ? 0xffffffff : 0);
		}
			mcu_count = (mcu_count + 1) % 6;
	}

}; // end transition()


/////////////////////////////////////////////////////
//	genMoore() method
/////////////////////////////////////////////////////

void genMoore()
{
	if(data_in_valid)
		INPUT.READ = true;
	else
		INPUT.READ = false;

	if(data_out_valid)
		OUTPUT.WRITE = true;
	else 
		OUTPUT.WRITE = false;
	
	OUTPUT.DATA.write(data_out);

}; // end genMoore()

}; // end structure SOCLIB_FIFO_TG


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
#define Y(i,j)          Y[8*i+j]
#define X(i,j)          (output[i][j])

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
 * This version is vital in passing overall mean error test. 
 */
static inline int DESCALE(int x, int n)
{
    return (x + (1 << (n - 1)) - (x < 0)) >> n;
}

/*
 * Maximum and minimum intermediate int values: 
 */
int             mini = 0;
int             maxi = 0;
static inline int ADD(int x, int y)
{
    int             r = x + y;

    if (r > maxi)
	maxi = r;
    if (r < mini)
	mini = r;
    return r;			/* in effect: & 0x0000FFFF */
}

static inline int SUB(int x, int y)
{
    int             r = x - y;

    if (r > maxi)
	maxi = r;
    if (r < mini)
	mini = r;
    return r;			/* in effect: & 0x0000FFFF */
}

static inline int CMUL(int c, int x)
{
    int             r = c * x;
    /*
     * Less accurate rounding here also works fine: 
     */
    r = (r + (1 << (C_BITS - 1))) >> C_BITS;
    if (r > maxi)
	maxi = r;
    if (r < mini)
	mini = r;
    return r;
}

/*
 * Rotate (x,y) over angle k*pi/16 (counter-clockwise) and scale with f. 
 */
static inline void
rot(int f, int k, int x, int y, int *rx, int *ry)
{
    int             COS[2][8] = {
	{c0_1, c1_1, c2_1, c3_1, c4_1, c5_1, c6_1, c7_1},
	{c0_s2, c1_s2, c2_s2, c3_s2, c4_s2, c5_s2, c6_s2, c7_s2}
    };
#define Cos(k)  COS[f][k]
#define Sin(k)  Cos(8-k)
    *rx = SUB(CMUL(Cos(k), x), CMUL(Sin(k), y));
    *ry = ADD(CMUL(Sin(k), x), CMUL(Cos(k), y));
#undef Cos
#undef Sin
}

/*
 * Butterfly: but(a,b,x,y) = rot(sqrt(2),4,a,b,x,y) 
 */
#define but(a,b,x,y)    { x = SUB(a,b); y = ADD(a,b); }

/*
 * Inverse 1-D Discrete Cosine Transform. Result Y is scaled up by factor
 * sqrt(8). Original Loeffler algorithm. 
 */
static inline void idct_1d(int *Y)
{
    int             z1[8],
                    z2[8],
                    z3[8];

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
void IDCT(unsigned int* input, unsigned char output[8][8])
{
    int             Y[64];
    int             k,
                    l;

    mini = IDCT_INT_MAX;
    maxi = IDCT_INT_MIN;
    for (k = 0; k < 8; k++) {	/* Pass 1: process rows. */
	for (l = 0; l < 8; l++)	/* Prescale k-th row: */
	    Y(k, l) = SCALE(input[8*k+l], S_BITS);
	idct_1d(&Y(k, 0));	/* 1-D IDCT on k-th row: */
	/*
	 * Result Y is scaled up by factor sqrt(8)*2^S_BITS. 
	 */
    }
    for (l = 0; l < 8; l++) {	/* Pass 2: process columns. */
	int             Yc[8];

	for (k = 0; k < 8; k++)
	    Yc[k] = Y(k, l);
	idct_1d(Yc);		/* 1-D IDCT on l-th column: */
	for (k = 0; k < 8; k++) {	/* Result is once more scaled up
					 * by a factor sqrt(8). */
	    int             r = 128 + DESCALE(Yc[k], S_BITS + 3);
	    r = r > 0 ? (r < 255 ? r : 255) : 0;	/* Clip to 8 bits
							 * unsigned: */
	    X(k, l) = r;
	}
    }
#if 0
    if (mini < ARITH_MIN)
	printf("Underflow; Minimum intermediate value: %d\n");
    if (maxi > ARITH_MAX)
	printf("Overflow; Maximum intermediate value: %d\n");
#endif
}

#endif
