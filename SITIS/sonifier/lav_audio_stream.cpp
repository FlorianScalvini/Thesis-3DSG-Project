
#include "lav_audio_stream.h"
#include "lav_vocal.h"

int lavAudioStream::_close_audio = 0;
snd_pcm_t* lavAudioStream::playback_handle = 0;


void lavAudioStream::release()
{
	/* allow the stream to be closed gracefully */
	signal(0, SIG_IGN);
	_close_audio = 1;
}

void lavAudioStream::start_thread_audio_stream() {


    //requires to be sudo
    pthread_attr_t custom_sched_attr; 	

    
	int fifo_mid_prio, fifo_max_prio, fifo_min_prio; 	
	struct sched_param fifo_param; 

    pthread_attr_init(&custom_sched_attr); 	
    pthread_attr_setinheritsched(&custom_sched_attr, PTHREAD_EXPLICIT_SCHED); 	
    pthread_attr_setschedpolicy(&custom_sched_attr, SCHED_FIFO); 	
		
    fifo_max_prio = sched_get_priority_max(SCHED_FIFO); 	
    fifo_min_prio = sched_get_priority_min(SCHED_FIFO); 	
    fifo_mid_prio = (fifo_min_prio + fifo_max_prio)/2; 	
    fifo_param.sched_priority = fifo_max_prio;//fifo_mid_prio; 	

    pthread_attr_setschedparam(&custom_sched_attr, &fifo_param); 	
	
	pthread_t thread_play_stream_audio;
	pthread_create(&thread_play_stream_audio, NULL, play_audio_stream, (void*)NULL);
	//pthread_create(&thread_play_stream_audio, &custom_sched_attr, play_audio_stream, (void*)NULL);

    //play_audio_stream(NULL);

}

void lavAudioStream::init()
{
    

    int i;
	int err;

	snd_pcm_hw_params_t *hw_params;
    unsigned int sampling_rate = AUDIO_SAMPLING_RATE;
    char* pcmOutputName = (char *)"default";

    int nbPeriod = 2;
    int periodSize = 256;//256;
    snd_pcm_uframes_t bufferSize = 2048;//2048;//periodSize*nbPeriod;

    short* buf = (short*) calloc(256*2, sizeof(short));

    float time = 0;
    for (int i = 0; i<256; i++) {
        time += 1./44100.;
        float value = 20000.*sin(2*3.14159*440*time);
        buf[i*2] = (short) value;
        buf[i*2+1] = (short) value;
    }


	if ((err = snd_pcm_open (&playback_handle, pcmOutputName, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
		fprintf (stderr, "cannot open audio device %s (%s)\n", 
			 pcmOutputName,
			 snd_strerror (err));
		exit (1);
	}
	   
	if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
		fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
			 
	if ((err = snd_pcm_hw_params_any (playback_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
			 snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_access (playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		fprintf (stderr, "cannot set access type (%s)\n",
			 snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_format (playback_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
		fprintf (stderr, "cannot set sample format (%s)\n",
			 snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_rate_near (playback_handle, hw_params, &sampling_rate, 0)) < 0) {
		fprintf (stderr, "cannot set sample rate (%s)\n",
			 snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_channels (playback_handle, hw_params, 2)) < 0) {
		fprintf (stderr, "cannot set channel count (%s)\n",
			 snd_strerror (err));
		exit (1);
	}

    err = snd_pcm_hw_params_set_periods(playback_handle, hw_params, nbPeriod, 0);
	if (err  == 0) {
        fprintf(stderr, "Error setting nb periods.\n");
		exit (1);
    }

    err = snd_pcm_hw_params_set_period_size(playback_handle, hw_params,	periodSize, 0);
	if (err < 0) {
		fprintf(stderr, "Error setting period size.\n");
		exit (1);
    }

    
    // Set buffer size (in frames). The resulting latency is given by 
    // latency = periodsize * periods / (rate * bytes_per_frame)     
    //err = snd_pcm_hw_params_set_buffer_size(playback_handle, hw_params, periodSize * nbPeriod);    
    err = snd_pcm_hw_params_set_buffer_size_near(playback_handle, hw_params, &bufferSize);
    if (err < 0) {
        fprintf(stderr, "Error setting buffersize.\n");
        exit (1);
    }
    lavLog::LAVLOG((char *)"bufferSize", bufferSize);

    //lavLog::LAVLOG("Set audio parameters", bufferSize);
	if ((err = snd_pcm_hw_params (playback_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot set parameters (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
    //lavLog::LAVLOG("Audio parameters successfully set");

    /* Set number of periods. Periods used to be called fragments. */ 
    /*if (snd_pcm_hw_params_set_periods(playback_handle, hw_params, 2, 0) < 0) {
        fprintf(stderr, "Error setting periods.\n");
        exit (1);
    }*/

    


    /* Set buffer size (in frames). The resulting latency is given by */
    /* latency = periodsize * periods / (rate * bytes_per_frame)     */
    /*if (snd_pcm_hw_params_set_buffer_size(playback_handle, hw_params, (periodsize * nbPeriods)>>2) < 0) {
        fprintf(stderr, "Error setting buffersize.\n");
        exit (1);
    }*/

	snd_pcm_hw_params_free (hw_params);

	if ((err = snd_pcm_prepare (playback_handle)) < 0) {
		fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
    //lavLog::LAVLOG("exit lav_audio_stream_init\n");

}
void* lavAudioStream::play_audio_stream(void* arg) {
    int result = 0;
    int cptLoop = 0;
    while (1) {//period_size) {
        //result = snd_pcm_writei(playback_handle, buf, 256);
        if(!lavVocal::isReading())
        {
            result = snd_pcm_writei(playback_handle, lavAudioMixer::pull_buffer(), 256);
        }
        else
        {
            result = snd_pcm_writei(playback_handle, lavVocal::pull_buffer(), 256);
        }
        //usleep(4000);
        
        if (result<0) {
            lavLog::LAVLOG((char *)"ERROR in lavAudioStream::play_audio_stream", result);
            if (result == -77) {
                lavLog::LAVLOG((char *)"PCM is not in the right state (SND_PCM_STATE_PREPARED or SND_PCM_STATE_RUNNING) ");
            }
            else if (result == -32) {
                lavLog::LAVLOG((char *)"an underrun occurred");
            }
            else if (result == -86) {
                lavLog::LAVLOG((char *)"a suspend event occurred (stream is suspended and waiting for an application recovery)");
            }
        }
        cptLoop++;
	/*
        if (cptLoop>1000) {
	    lavLog::LAVLOG("Bug volontaire dans lav_audio_stream.cpp methode playAudioStream");
            int test=5;
            lavLog::LAVLOG("cpt", test/0);
        }
	*/
    }
    snd_pcm_close (playback_handle);
}


