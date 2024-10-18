#ifndef LAV_COMPUTER
#define LAV_COMPUTER

#include "lav_constants.h"

#if LAV_NEON

#include <arm_neon.h>
#include <cpu-features.h>
#define ASSUME_ALIGNED_FLOAT_128(ptr) ((float *)__builtin_assume_aligned((ptr), 16))

#endif


class lavComputer {

    public: 

	    static int ID_t;

	    //add float vectors with NEON
	    static void add_float_vector(float* sum, float* summand, int count);

	    //add to int32_t vectors.
	    //the result could be put in scr1 instead of dst
	    static void add_int32_vector(int32_t* dst, int32_t* src1, int32_t* src2, int count);


	    //multiply a float vector by a scalar.
	    //the result could be put in scr1 instead of dst
	    static void mul_float_vector_by_scalar(float* dst, float* src1, float scalar, int count);

	    static void mul_float_vector(float* dst, float* src1, float* src2, int count);

	    //add to short vector -> no problem of coding limits
	    //the result should be put in in a dest different from src1 and scr2
	    static void add_short_vector(short* dst, short* src1, short* src2, int count);

	    //multiply a short vector by a float vector and put the result back into a short vector
	    //the result should be put in a dest different from src1
	    static void mul_short_vector_by_float_vector(short* dst, short* src1, float* src2, int count);

	    static void init();

};

#endif



