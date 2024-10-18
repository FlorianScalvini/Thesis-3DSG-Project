#ifndef LAV_CONSTANTS
#define LAV_CONSTANTS



#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstdio>

#include "lav_log.h"

#define DEPTH_FRAME_WIDTH 640
#define DEPTH_FRAME_HEIGHT 360

#define COLOR_FRAME_WIDTH 1280
#define COLOR_FRAME_HEIGHT 720


//to define
#define DEPTH_VIDEOPROCESSING
#define GRAYSCALE_SONIFICATION
#define MIN_GRAYSCALE_SONIFICATION 40
//to be defined

//AUDIO
#define SIZE_AUDIO_CHUNK_IN_SAMPLE 256//256
#define NB_AUDIO_CHUNK_IN_SOUND 8//8

//to keep unchanged

#define PI 3.14159265359

#define AUDIO_SAMPLING_RATE 44100

#define NB_BYTE_PER_SAMPLE 2
#define NB_OUTPUT_AUDIO_CHANNEL 2

#define SIZE_AUDIO_CHUNK_IN_BYTE SIZE_AUDIO_CHUNK_IN_SAMPLE*NB_BYTE_PER_SAMPLE*NB_OUTPUT_AUDIO_CHANNEL
#define SIZE_AUDIO_CHUNK_IN_VALUE SIZE_AUDIO_CHUNK_IN_SAMPLE*NB_OUTPUT_AUDIO_CHANNEL

#define SIZE_SOUND_IN_SAMPLE NB_AUDIO_CHUNK_IN_SOUND*SIZE_AUDIO_CHUNK_IN_SAMPLE
#define SIZE_SOUND_IN_VALUE NB_AUDIO_CHUNK_IN_SOUND*SIZE_AUDIO_CHUNK_IN_VALUE
#define SIZE_SOUND_IN_BYTE NB_AUDIO_CHUNK_IN_SOUND*SIZE_AUDIO_CHUNK_IN_BYTE


//VIDEO

#define FRAME_WIDTH_SONIFIED 160
#define FRAME_HEIGHT_SONIFIED 120


class lavConstants {

    public:

	    static float* _sigmoidal_fade_in;
	    static float* _sigmoidal_fade_out;

	    static float __maxNbSonifiedPixel;

    	static void init();

		static void __startTimeChecking();
		static void __stopTimeChecking(char* typeOfCheck);
    

	private:
        static struct timespec begin, end;
        static float __elapsed;
		static int __cptcheck;
		static int __nbTimeCheck;

		static float* _buff_time;





		static int compare (void const *a, void const *b);



};


#endif
