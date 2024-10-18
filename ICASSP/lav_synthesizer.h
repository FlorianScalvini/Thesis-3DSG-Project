#ifndef LAV_SYNTHESIZER
#define LAV_SYNTHESIZER

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

#include "lav_log.h"
#include "lav_constants.h"
#include "lav_wav.h"


class lavSynthesizer {

	//private (anonymous namespace)
	private:
		static float _base_amplitude;

    public:

	    static float getWeightedAmplitude(float frequency);


	    static float getAmplitudeRight(unsigned int ID_x);

	    static float getITDLeft(unsigned int ID_x);


	    static float getFrequency(int idRow, int idCol);


	    static void exportToWav(float*** sound_db, char* databasePath);

	    static void initFromSynthesizing(char* databasePath);

};


#endif
