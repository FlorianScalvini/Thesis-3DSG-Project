#include "lav.h"

void lav::start() {

	lavLog::LOG("start lav\n");

    lav::setDefaultDatabasePath("..//files//lav_default_sound_db.wav");

	lavComputer::init();
	lavConstants::init();
	lavSoundDatabase::init();
	lavAudioStream::init();
	lavSonifier::init();
	lavAudioMixer::init();
    lavCameraAcquisition::init();
	lavVideoProcessor::init("../files/yolov5/yolov5s.engine");
    lavLog::LOG("lav init finished\n");

    usleep(7000000);

    lavLog::LOG("lavCameraAcquisition::start_thread_video_stream\n");
    lavCameraAcquisition::start_thread_acquisition();

    lavLog::LOG("lavVideoProcessor::start_thread_video_stream\n");
    lavVideoProcessor::start_thread_video_stream();

    lavLog::LOG("lavAudioStream::start_thread_audio_stream\n");
    lavAudioStream::start_thread_audio_stream();    

    
}

void lav::stop() {
    lavLog::LOG("lav::stop() !!!!!!!!!!\n");
	lavAudioStream::release();
	lavVideoProcessor::release();
}

void lav::setDatabasePath(char* databasePath) {
    char str[200];
    sprintf(str, "setDatabasePath %s\n", databasePath);
	lavLog::LOG(str);
	lavSoundDatabase::setDatabasePath(databasePath);
}

void lav::setDefaultDatabasePath(char* databasePath) {
    char str[200];
    sprintf(str, "setDefaultDatabasePath %s\n", databasePath);
	lavLog::LOG(str);
	lavSoundDatabase::setDefaultDatabasePath(databasePath);
}

void lav::startOrStopSound() {
	lavLog::LOG("startOrStopSound \n");
	lavVideoProcessor::startOrStopSound();
}

