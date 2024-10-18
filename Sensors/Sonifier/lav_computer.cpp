#include "lav_computer.h"

int lavComputer::ID_t = 0;

#if LAV_NEON



//add float vectors with NEON
void lavComputer::add_float_vector(float* sum, float* summand, int count)
{
	asm volatile (
	   "1:                                          \n"
	   "vld1.32         {q0}, [%[sum]]              \n"
	   "vld1.32         {q1}, [%[summand]]!         \n"
	   "vadd.f32        q0, q0, q1                  \n"
	   "subs            %[count], %[count], #4      \n"
	   "vst1.32         {q0}, [%[sum]]!             \n"
	   "bgt             1b                          \n"
	   : [sum] "+r" (sum)
	   : [summand] "r" (summand), [count] "r" (count)
	   : "memory", "q0", "q1"
  );
}



/*void add_float_vector_with_neon3(float* dst, float* src1, float* src2, int count)
{
	asm volatile (
	   "1:                                                        \n"
	   "vld1.32         {q0}, [%[src1]]!                          \n"
	   "vld1.32         {q1}, [%[src2]]!                          \n"
	   "vadd.f32        q0, q0, q1                                \n"
	   "subs            %[count], %[count], #4                    \n"
	   "vst1.32         {q0}, [%[dst]]!                           \n"
	   "bgt             1b                                        \n"
	   : [dst] "+r" (dst)
	   : [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
	   : "memory", "q0", "q1"
  );
}*/

//Add float vector with NEON (src1 == dst)
/*void add_float_vector_with_neon3(float* dst, float* src1, float* src2, int count)
{
	asm volatile (
			"1:                                	\n"
			"vld1.32 {q0,q1}, [%[src1]]!		\n"
			"vld1.32 {q2,q3}, [%[src2]]!		\n"
			"vadd.f32 q0, q0, q2				\n"
			"vadd.f32 q1, q1, q3   				\n"
			"vld1.32 {q4,q5}, [%[src1]]!   		\n"
			"vld1.32 {q6,q7}, [%[src2]]!		\n"
			"vadd.f32 q4, q4, q6				\n"
			"vadd.f32 q5, q5, q7   				\n"
			"subs %[count], %[count], #16		\n"
			"vst1.32 {q0, q1}, [%[dst]]!  		\n"
			"vst1.32 {q4, q5}, [%[dst]]!		\n"
			"bgt             1b                 \n"
			: [dst] "+r" (dst)
			: [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
			: "memory", "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7"
	  );
}*/






//__attribute__((optimize("unroll-loops")))restrict
/*void add_float_vector_with_neon3(float * dst, float *src1, float *src2,  int size)
{
	for(int i=0;i<size;i+=4){
		float32x4_t inFloat41  = vld1q_f32(ASSUME_ALIGNED_FLOAT_128(src1));
		float32x4_t inFloat42  = vld1q_f32(ASSUME_ALIGNED_FLOAT_128(src2));
		float32x4_t outFloat64 = vaddq_f32 (inFloat41, inFloat42);
		vst1q_f32 (ASSUME_ALIGNED_FLOAT_128(dst), outFloat64);
		src1+=4;
		src2+=4;
		dst+=4;
	}
}*/

//add to int32_t vectors.
//the result could be put in scr1 instead of dst
void lavComputer::add_int32_vector(int32_t* dst, int32_t* src1, int32_t* src2, int count)
{

	asm volatile (
		   "6:                                                        \n"
		   "vld1.32         {q0}, [%[src1]]!                          \n"
		   "vld1.32         {q1}, [%[src2]]!                          \n"
		   "vadd.s32        q0, q0, q1                                \n"
		   "subs            %[count], %[count], #4                    \n"
		   "vst1.32         {q0}, [%[dst]]!                           \n"
		   "bgt             6b                                        \n"
		   : [dst] "+r" (dst)
		   : [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
		   : "memory", "q0", "q1"
	  );
}


//multiply a float vector by a scalar.
//the result could be put in scr1 instead of dst
void lavComputer::mul_float_vector_by_scalar(float* dst, float* src1, float scalar, int count)
{

	asm volatile (

			"vdup.32         q1, %[scalar]	 	                      	\n"
			"5:                                                        	\n"
			"vld1.32         {q0}, [%[src1]]!                         	\n"
			"vmul.f32        q0, q0, q1                               	\n"
			"subs            %[count], %[count], #4                    	\n"
			"vst1.32         {q0}, [%[dst]]!                          	\n"
			"bgt             5b                                        	\n"
			: [dst] "+r" (dst)
			: [src1] "r" (src1), [scalar] "r" (scalar), [count] "r" (count)
			: "memory", "q0", "q1"
	  );
}

void lavComputer::mul_float_vector(float* dst, float* src1, float* src2, int count)
{

	asm volatile (
		   "2:                                                        \n"
		   "vld1.32         {q0}, [%[src1]]!                          \n"
		   "vld1.32         {q1}, [%[src2]]!                          \n"
		   "vmul.f32        q0, q0, q1                                \n"
		   "subs            %[count], %[count], #4                    \n"
		   "vst1.32         {q0}, [%[dst]]!                           \n"
		   "bgt             2b                                        \n"
		   : [dst] "+r" (dst)
		   : [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
		   : "memory", "q0", "q1"
	  );
}

//add to short vector -> no problem of coding limits
//the result should be put in in a dest different from src1 and scr2
void lavComputer::add_short_vector(short* dst, short* src1, short* src2, int count)
{

	asm volatile (
		   "3:                                                        \n"
		   "vld1.16         {q0}, [%[src1]]!                          \n"
		   "vld1.16         {q1}, [%[src2]]!                          \n"
		   "vadd.i16        q0, q0, q1                                \n"
		   "subs            %[count], %[count], #8                    \n"
		   "vst1.16         {q0}, [%[dst]]!                           \n"
		   "bgt             3b                                        \n"
		   : [dst] "+r" (dst)
		   : [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
		   : "memory", "q0", "q1"
	  );
}

//multiply a short vector by a float vector and put the result back into a short vector
//the result should be put in a dest different from src1
void lavComputer::mul_short_vector_by_float_vector(short* dst, short* src1, float* src2, int count)
{
	asm volatile (
		"4:                                                        	\n"
		"vld1.16		{d0}, [%[src1]]!							\n"
		"vld1.32        {q1}, [%[src2]]!                    	    \n"
		"vmovl.s16		q0, d0										\n"
		"vcvt.f32.s32	q0, q0										\n"
		"vmul.f32       q0, q0, q1									\n"
		"vcvt.s32.f32	q0, q0										\n"
		"vmovn.s32		d0, q0										\n"
		"subs            %[count], %[count], #4                    	\n"
		"vst1.16         {d0}, [%[dst]]!                           	\n"
		"bgt             4b                                        	\n"
		: [dst] "+r" (dst)
		: [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
		: "memory", "d0", "q0", "q1"

	);
}



/*
void mul_short_neon3(short* dst, short* src1, short* src2, int count)
{
	asm volatile (
		   "5:                                                        \n"
		   "vld1.16         {q0}, [%[src1]]!                          \n"
		   "vld1.16         {q1}, [%[src2]]!                          \n"
		   "vmul.i16        q0, q0, q1                                \n"
		   "subs            %[count], %[count], #8                    \n"
		   "vst1.16         {q0}, [%[dst]]!                           \n"
		   "bgt             5b                                        \n"
		   : [dst] "+r" (dst)
		   : [src1] "r" (src1), [src2] "r" (src2), [count] "r" (count)
		   : "memory", "q0", "q1"
	  );
}*/




void lavComputer::init() {

	/*int flagCanUseNeon = false;

	if (android_getCpuFamily() == ANDROID_CPU_FAMILY_ARM &&
		(android_getCpuFeatures() & ANDROID_CPU_ARM_FEATURE_NEON) != 0)
	{
		flagCanUseNeon = true;
	}

	//flagCanUseNeon = false;

	if (flagCanUseNeon) {
		//add_float_vector = &add_float_vector_with_neon3;
		mul_float_vector_by_scalar = &mul_float_vector_by_scalar_with_neon3;
		mul_float_vector = &mul_float_vector_with_neon3;
		add_short_vector = &add_short_vector_with_neon3;
		mul_short_vector_by_float_vector = &mul_short_vector_by_float_vector_with_neon3;
	}
	else {
		//this induce some clics in the output sound with 10x10 active pixels. Don't know why.
		//add_float_vector = &add_float_vector_without_neon3;
		mul_float_vector_by_scalar = &mul_float_vector_by_scalar_without_neon3;
		mul_float_vector = &mul_float_vector_without_neon3;
		add_short_vector = &add_short_vector_without_neon3;
		mul_short_vector_by_float_vector = &mul_short_vector_by_float_vector_without_neon3;

	}*/
}


#else



void lavComputer::init() {
}

//Add float vector with C
void lavComputer::add_float_vector(float* sum, float* summand, int count)
{
	for (ID_t=0; ID_t<count; ++ID_t)
		sum[ID_t] += summand[ID_t];
}

/*void lavComputer::add_float_vector(float* dst, float* src1, float* src2, int count)
{
	for (ID_t =0; ID_t<count; ++ID_t)
		dst[ID_t] += src2[ID_t];
}*/

void lavComputer::mul_float_vector_by_scalar(float* dst, float* src1, float scalar, int count)
{
	for (ID_t =0; ID_t<count; ++ID_t)
		dst[ID_t] *= scalar;
}

void lavComputer::mul_float_vector(float* dst, float* src1, float* src2, int count)
{
	for (ID_t =0; ID_t<count; ++ID_t)
		dst[ID_t] *= src2[ID_t];
}

void lavComputer::add_short_vector(short* dst, short* src1, short* src2, int count)
{
	for (ID_t =0; ID_t<count; ++ID_t)
		dst[ID_t] = src1[ID_t]+src2[ID_t];
}

void lavComputer::mul_short_vector_by_float_vector(short* dst, short* src1, float* src2, int count)
{
	float val = 0;

	for (ID_t=0; ID_t<SIZE_AUDIO_CHUNK_IN_VALUE; ++ID_t) {
		val = (float) ( src1[ID_t]);
		val *= src2[ID_t];
		dst[ID_t] = (short) val;
	}

}


#endif
