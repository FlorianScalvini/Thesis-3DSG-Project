#ifndef LAV_CONSTANTS
#define LAV_CONSTANTS



#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include "logger.h"
#include <vector>

#define UART_GPS "/dev/ttyACM0"
#define UART_IMU "/dev/ttyUSB1"
#define I2C_IMU "/dev/i2c-3"

//to define
//#define OBJECT_DETECTION

#define PATH_MARKER
#define GRAYSCALE_SONIFICATION
#define MIN_GRAYSCALE_SONIFICATION 35
//to be defined

//AUDIO
#define SIZE_AUDIO_CHUNK_IN_SAMPLE 256//256
#define NB_AUDIO_CHUNK_IN_SOUND 8//8
#define NB_PATH_ANGLE 36
#define ANGLE_LIMIT 90
//VIDEO

#define DEPTH_FRAME_WIDTH 640
#define DEPTH_FRAME_HEIGHT 360

#define COLOR_FRAME_WIDTH 1280
#define COLOR_FRAME_HEIGHT 720


#define FRAME_WIDTH_SONIFIED 160
#define FRAME_HEIGHT_SONIFIED 120
#define SIDEWALK_SEGMENTATION_ID 3
#define CROSSWALK_SEGMENTATION_ID 4
#define THRESH_DST_SONIFY_DYNAMIC 15000
#define THRESH_DST_SONIFY 3000
#define THRESH_DST_SONIFY_NEAR 750
#define PEDESTRIAN_TRAFFIC_LIGHT_ID 11

// Navigation thresh of proximity
#define THRESH_DST_TARGET 3

//to keep unchanged

#define INCR_ANGL 1

#define PI 3.14159265359

#define AUDIO_SAMPLING_RATE 44100

#define NB_BYTE_PER_SAMPLE 2
#define NB_OUTPUT_AUDIO_CHANNEL 2

#define SIZE_AUDIO_CHUNK_IN_BYTE SIZE_AUDIO_CHUNK_IN_SAMPLE*NB_BYTE_PER_SAMPLE*NB_OUTPUT_AUDIO_CHANNEL
#define SIZE_AUDIO_CHUNK_IN_VALUE SIZE_AUDIO_CHUNK_IN_SAMPLE*NB_OUTPUT_AUDIO_CHANNEL


#define SIZE_SOUND_IN_SAMPLE NB_AUDIO_CHUNK_IN_SOUND*SIZE_AUDIO_CHUNK_IN_SAMPLE
#define SIZE_SOUND_IN_VALUE NB_AUDIO_CHUNK_IN_SOUND*SIZE_AUDIO_CHUNK_IN_VALUE
#define SIZE_SOUND_IN_BYTE NB_AUDIO_CHUNK_IN_SOUND*SIZE_AUDIO_CHUNK_IN_BYTE
#define SAVE_PATH "./save/"

#define NUM_CLASS 27
#define NUM_TRACKED_ID 3
const int trackedID[NUM_TRACKED_ID] = {0, 1, 2};

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
