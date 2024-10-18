#ifndef LAV
#define LAV

#include "logger.h"
#include "lav_audio_stream.h"
#include "lav_constants.h"
#include "lav_sonifier.h"
#include "lav_video_processor.h"
#include "lav_sound_database.h"
#include "lav_imu_acquisition.h"
#include "lav_wav.h"
#include "lav_computer.h"
#include "lav_manager.h"
#include "lav_vocal.h"
#include "lav_camera_acquisition.h"

class lav {
    public:
	    static void start();
	    static void stop();
	    static void setDefaultDatabasePath(char* databasePath);
	    static void setDatabasePath(char* databasePath);
	    static void startOrStopSound();
};


#endif
