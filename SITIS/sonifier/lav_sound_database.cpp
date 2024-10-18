
#include "lav_sound_database.h"

float*** lavSoundDatabase::_sound_db = 0;

float lavSoundDatabase::_audioVolume = 0;

char* lavSoundDatabase::_defaultDatabasePath = 0;
char* lavSoundDatabase::_databasePath = 0;


void lavSoundDatabase::setDatabasePath(char* databasePath) {
	_databasePath = databasePath;
    //lavLog::LAVLOG(_databasePath);
}

void lavSoundDatabase::setDefaultDatabasePath(const char* defaultDatabasePath) {
	_defaultDatabasePath = (char*)defaultDatabasePath;
    //lavLog::LAVLOG(_databasePath);
}

void lavSoundDatabase::multiplyByAudioVolume() {

	for (unsigned int ID_x =0; ID_x<FRAME_WIDTH_SONIFIED; ++ID_x) {
		float** curr_sound_db_x = _sound_db[ID_x];

		for (unsigned int ID_y =0; ID_y<FRAME_HEIGHT_SONIFIED; ++ID_y) {
			float* curr_sound_db_x_y = curr_sound_db_x[ID_y];
			lavComputer::mul_float_vector_by_scalar(curr_sound_db_x_y, curr_sound_db_x_y, _audioVolume, SIZE_SOUND_IN_VALUE);

		}
	}
}


void lavSoundDatabase::initFromWav() {

    lavLog::LAVLOG("initFromWav\n");

    if (_databasePath == 0) {
        _databasePath = _defaultDatabasePath;
    }
	
	if (lavWav::loadWav(_sound_db, _databasePath) == 1) {

        char str[1000];
        sprintf(str, "!!!!!!! Warning: Trying to load default database%s\n", _defaultDatabasePath);
		lavLog::LAVLOG(str);

		if (lavWav::loadWav(_sound_db, _defaultDatabasePath) == 1) {

            char str1[1000];
            sprintf(str1, "!!!!!!! Warning: Default database file is not present. Trying to synthesized it in %s\n", _defaultDatabasePath);
		    lavLog::LAVLOG(str1);

			
			lavSynthesizer::initFromSynthesizing(_defaultDatabasePath);
			if (lavWav::loadWav(_sound_db, _defaultDatabasePath) == 1) {
                char str2[1000];
                sprintf(str2, "!!!!!!! Warning: Problem creating the default database %s. Program aborted \n", _defaultDatabasePath);
			    lavLog::LAVLOG(str2);
			}
		}
	}
}

void lavSoundDatabase::init() {

    _audioVolume =200.;
    if (_defaultDatabasePath ==0) {
    	_defaultDatabasePath = "/sdcard/Download/lav_default_sound_db.wav";
    }


	lavLog::LAVLOG("init sound database\n");
	_sound_db = (float***) calloc(FRAME_WIDTH_SONIFIED, sizeof(float**));

	//initFromSynthesizing();
	initFromWav();

	multiplyByAudioVolume();
        lavLog::LAVLOG("Init from wave finished\n");
}


float* lavSoundDatabase::getSound(unsigned int ID_x, unsigned int ID_y)
{
	return _sound_db[ID_x][ID_y];
}


