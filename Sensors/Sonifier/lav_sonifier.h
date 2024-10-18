//
// Created by Florian on 08/12/22.
//

#ifndef LAV_SONIFIER
#define LAV_SONIFIER



#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>

#include <iostream>
#include <string>


//#include <asoundlib.h>


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../Sound/sound_reader_hrtf.h"
#include "logger.h"
#include "lav_sound_database.h"
#include "lav_audio_mixer.h"
#include "lav_constants.h"
#include "lav_computer.h"

//#include <cpu-features.h>
//#include <arm_neon.h>



class lavSonifier {

	private:
        static SoundReaderHrtf* _pathSound; // Path sound;
		static short* _short_sound;
		static short* _short_silence;
		static int32_t* _test_int32;
		static int _sizeSimplificationChunk;
		static int _nbKeptPixInSimplificationChunk;
		static short* _short_audio_output;
		static float* _float_audio_output;

        //GRAYSCALE
		static float** _float_grayscale_result;
		static float** _float_grayscale_modulation;
		static int* _grayscale_pixelCounter;
        static int* _startValueModulation;
        static int* _nbValueModulation;
        static int* _pathOrientation;


public:
        static void computeCompressionFactor(int);
        static void sonify(cv::Mat*);
        static void silence();
        static void init(const char* path);
        static void setAngle(int*);

};

#endif
