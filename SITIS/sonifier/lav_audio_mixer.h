

#ifndef LAV_AUDIO_MIXER
#define LAV_AUDIO_MIXER



#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>

#include <iostream>
#include <string>





#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>


//#include <arm_neon.h>
//#include <asoundlib.h>
//#include <cpu-features.h>

#include "lav_log.h"
#include "lav_constants.h"
#include "lav_computer.h"
#include "../sound/sound_wav.h"



class lavAudioMixer {

	private:

		static int _IDVal;

        static bool _firstImageReceived;
        static int _cptRecordedAudioChunk;
        static int _nbChunkToRecord;
        static bool _recordOutputInWave;
        static char* _recordingFileName;
        static short* _arrayForWaveFile;

		static unsigned int _buffer_as_swap;
		static unsigned int _cptVideoFrame;

		static pthread_mutex_t _buffer_mutex;


		static char* _sonification_buffer_reading;
		static char* _audio_output_buffer;
		static char* _beginning_audio_buffer;


		//float* _sigmoidal_fade_out;

		static char* _pointer_curr_chunk;
		static char* _position_last_chunk_pointer;
		static short* _short_beginning_audio_buffer;
		static short* _short_last_chunk_audio_output_buffer;

		static short* _short_pointer_curr_chunk;
		static short* _short_mix_result;

    private:		

        static void recordCurrentAudioChunk(void* currentAudioChunck);

    public:

        static void record_output(char* fileName, int durationInMs);

        
	    static void push_buffer(void* pointer_data);
	    static void* pull_buffer();
	    static void init();

};


#endif
