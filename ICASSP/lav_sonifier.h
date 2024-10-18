

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

#include "lav_log.h"
#include "lav_sound_database.h"
#include "lav_audio_mixer.h"
#include "lav_constants.h"
#include "lav_computer.h"

//#include <cpu-features.h>
//#include <arm_neon.h>



class lavSonifier {

	private:

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

        //GRAYSCALE

    public:

        static void computeCompressionFactor(int nbActivePixel);
        static void sonify(cv::Mat* mAbsDiffFrame);
        static void init();

};

#endif
