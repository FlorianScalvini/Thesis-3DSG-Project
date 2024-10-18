
#ifndef LAV_AUDIO_STREAM
#define LAV_AUDIO_STREAM

#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#include <alsa/asoundlib.h>

#include "lav_log.h"
#include "lav_manager.h"

#include "lav_audio_mixer.h"
#include "lav_constants.h"


class lavAudioStream {

	/*short* testBuffer;
	void* vTestBuffer;*/


	//private (anonymous namespace)
	private:

		static int _close_audio;
		static snd_pcm_t *playback_handle;

		static void* play_audio_stream(void* arg);

    public:

	    static void release();
	    static void start_thread_audio_stream();
	    static void init();

};

#endif


