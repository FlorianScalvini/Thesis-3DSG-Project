#include "lav_constants.h"

float lavConstants::__elapsed;
float* lavConstants::_sigmoidal_fade_in = 0;
float* lavConstants::_sigmoidal_fade_out = 0;

float lavConstants::__maxNbSonifiedPixel = 100; //500;
int lavConstants::__cptcheck  = 0;
int lavConstants::__nbTimeCheck  = 1;
float* lavConstants::_buff_time = (float*) calloc(__nbTimeCheck, sizeof(float));

timespec lavConstants::begin;
timespec lavConstants::end;

void lavConstants::__startTimeChecking() {
    clock_gettime(CLOCK_MONOTONIC_RAW, &begin);
}

int lavConstants::compare(void const *a, void const *b)
{
   int const *pa = (int const *)a;
   int const *pb = (int const *)b;
   return *pa - *pb;
}

void lavConstants::__stopTimeChecking(char* typeOfCheck) {

    clock_gettime(CLOCK_MONOTONIC_RAW, &end);

    __elapsed = (end.tv_nsec - begin.tv_nsec)/1000000.+(end.tv_sec  - begin.tv_sec)*1000.;

	_buff_time[__cptcheck] = __elapsed;

	__cptcheck += 1;

	if (__cptcheck == __nbTimeCheck) {

		//LOGI("%s: %d\n", typeOfCheck, _buff_time[0]);
		//LOGI("%s: %d\n", typeOfCheck, _buff_time[1]);
		//LOGI("%s: %d\n", typeOfCheck, _buff_time[2]);

		float mean = 0;

		for (__cptcheck = 0; __cptcheck<__nbTimeCheck; ++__cptcheck) {
			int value = _buff_time[__cptcheck];
			mean += (float) _buff_time[__cptcheck];
		}
		mean = mean/(float) __nbTimeCheck;
		//LOGI("%s: %f\n", typeOfCheck, mean);

		qsort(_buff_time, __nbTimeCheck, sizeof(_buff_time[0]), compare);
		float median = _buff_time[__nbTimeCheck/2];
		float firstQuartile = _buff_time[__nbTimeCheck/4];
		float thirdQuartile = _buff_time[3*__nbTimeCheck/4];
		float minVal = _buff_time[0];
		float maxVal = _buff_time[__nbTimeCheck-1];


        char str[1000];
        sprintf(str, "%s (ms): %f, %f, %f, %f, %f, %f\n", typeOfCheck, mean, median, firstQuartile, thirdQuartile, minVal, maxVal);
	    lavLog::LAVLOG(str);


		/*
		float mean = 0;
		float std = 0;
		float tmp = 0;
		int max = 0;
		int min = 10000000000;

		for (__cptcheck = 0; __cptcheck<__nbTimeCheck; ++__cptcheck) {
			int value = _buff_time[__cptcheck];
			if (value > max) {
				max = value;
			}
			if (value<min) {
				min = value;
			}
			mean += (float) _buff_time[__cptcheck];
		}
		mean = mean/(float) __nbTimeCheck;

		for (__cptcheck = 0; __cptcheck<__nbTimeCheck; ++__cptcheck) {
			tmp = _buff_time[__cptcheck]-mean;
			std+=tmp*tmp;
		}
		std = sqrt(std/(float) __nbTimeCheck);


		LOGI("%s: Nb time check = %d, mean elapsed us=%f, std = %f, min = %d, max = %d \n", typeOfCheck, __nbTimeCheck, mean, std, min, max);
		mean = 0;
		*/
		__cptcheck = 0;

	}
}


void lavConstants::init() {



	_sigmoidal_fade_in = (float*) calloc(SIZE_AUDIO_CHUNK_IN_VALUE, sizeof(float));
	_sigmoidal_fade_out = (float*) calloc(SIZE_AUDIO_CHUNK_IN_VALUE, sizeof(float));

	float time = 0;
	float lambda = 7;
	float f0 = 1/(1+exp(-lambda*(-0.5)));

	for (int i =0; i<SIZE_AUDIO_CHUNK_IN_VALUE; i+=2) {
		float value = (1./(1.+ exp(-lambda*((float) i/(float) SIZE_AUDIO_CHUNK_IN_VALUE-0.5))) - f0) / (1-2*f0);
		//float value = ((float)(i/2)) / ((float) (SIZE_AUDIO_CHUNK_IN_SAMPLE));//for linear

		_sigmoidal_fade_in[i] = _sigmoidal_fade_in[i+1]= value;
		_sigmoidal_fade_out[i] = _sigmoidal_fade_out[i+1]= 1.-_sigmoidal_fade_in[i];
		time += 1/44100.;
	}
}

