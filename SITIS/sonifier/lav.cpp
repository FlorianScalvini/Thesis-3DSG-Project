#include "lav.h"

void lav::start() {
    lav::setDefaultDatabasePath("../files/lav_default_sound_db.wav", "../res/hrtf_beep_10.wav");
	lavLog::LAVLOG("start lav\n");
    lavVocal::init();
	lavVideoProcessor::init();
	lavComputer::init();
	lavConstants::init();
	lavSoundDatabase::init();
	lavAudioStream::init();
	lavSonifier::init();
	lavAudioMixer::init();
    lavCameraAcquisition::init();
    lavManager::init("../path/graph.txt");

    //lavAudioMixer::record_output("./res/hrtf_cut020_180H_90V/audio/trajectory/09_09_far.wav", 5000);
    lavLog::LAVLOG("lav init finished\n");
    //lavVideoProcessor::start_video_stream(NULL);

    usleep(3000000);

    lavLog::LAVLOG("lavCameraAcquisition::start_thread_video_stream\n");
    lavCameraAcquisition::start_thread_acquisition();

    lavLog::LAVLOG("lavVideoProcessor::start_thread_video_stream\n");
    lavVideoProcessor::start_thread_video_stream();

    lavLog::LAVLOG("lavAudioStream::start_thread_audio_stream\n");
    lavAudioStream::start_thread_audio_stream();

    lavLog::LAVLOG("lavAudioStream::start_thread_audio_stream\n");
    lavManager::start_thread_path_manager();

}

void lav::stop() {
    lavLog::LAVLOG("lav::stop() !!!!!!!!!!\n");
    lavManager::release();
	lavAudioStream::release();
	lavVideoProcessor::release();
}

void lav::setDatabasePath(char* databasePath) {
    char str[200];
    sprintf(str, "setDatabasePath %s\n", databasePath);
	lavLog::LAVLOG(str);
	lavSoundDatabase::setDatabasePath(databasePath);
}

void lav::setDefaultDatabasePath(const char* databasePath, const char* pathSoundPath) {
    char str[200];
    sprintf(str, "setDefaultDatabasePath %s\n", databasePath);
	lavLog::LAVLOG(str);
	lavSoundDatabase::setDefaultDatabasePath(databasePath);
    lavSonifier::setDatabasePath(pathSoundPath);
    sprintf(str, "setSoundDirectionPath %s\n", pathSoundPath);
    lavLog::LAVLOG(str);
}


void lav::startOrStopSound() {
	lavLog::LAVLOG("startOrStopSound \n");
	lavVideoProcessor::startOrStopSound();
}

