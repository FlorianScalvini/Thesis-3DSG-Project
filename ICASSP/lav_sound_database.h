
#ifndef LAV_SOUND_DATABASE
#define LAV_SOUND_DATABASE


#include <time.h>
#include <stdlib.h>
#include <math.h>

#include "lav_log.h"
#include "lav_constants.h"
#include "lav_wav.h"
#include "lav_synthesizer.h"
#include "lav_computer.h"


class lavSoundDatabase {

    private:

		static float*** _sound_db;

		//float _audioVolume =100.;
		static float _audioVolume;//for normal sound

		static char* _defaultDatabasePath;
		static char* _databasePath;

    public:

	    static void setDatabasePath(char* databasePath);
	    static void setDefaultDatabasePath(char* databasePath);

	    static void multiplyByAudioVolume();

	    static void initFromWav();

	    static void init();

	    static float* getSound(unsigned int ID_x, unsigned int ID_y);

};

#endif
