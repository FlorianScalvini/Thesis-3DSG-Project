#include "lav.h"

void lav::start() {
    std::locale::global(std::locale::classic());
    lav::setDefaultDatabasePath("../files/DatabaseHRTF/lav_default_sound_db.wav");
    SonifierLogger::logger("start lav\n");
    lavAudioMixer::init();
    lavAudioStream::init();
    lavVocal::init("../files/");
    lavSonifier::init("../files/DatabaseHRTF/hrtf_beep_0_-5.wav");
    lavComputer::init();
    lavConstants::init();


    lavSoundDatabase::init();
    lavCameraAcquisition::init();
    lavImuAcquisition::init("./calibration.txt", UART_IMU);
    lavVideoProcessor::init("../files/Model/yolov8m.engine", "../files/Model/ddrnet.engine");

    lavManager::init("../files/map.osm");
    SonifierLogger::logger("lav init finished\n");

    usleep(1000000);

    SonifierLogger::logger("lavAudioStream::start_thread_audio_stream\n");
    lavAudioStream::start_thread_audio_stream();

    SonifierLogger::logger("lavImuAcquisition::start_thread_video_stream\n");
    lavImuAcquisition::start_thread_acquisition();

    SonifierLogger::logger("lavCameraAcquisition::start_thread_video_stream\n");
    lavCameraAcquisition::start_thread_acquisition();

    usleep(2000000);

    SonifierLogger::logger("lavVideoProcessor::start_thread_video_stream\n");
    lavVideoProcessor::start_thread_video_stream();

    SonifierLogger::logger("lavManager::start_thread_path_stream\n");
    lavManager::start_thread_manager_stream();
}

void lav::stop() {
    SonifierLogger::logger("lav::stop() !!!!!!!!!!\n");
    lavManager::release();
	lavAudioStream::release();
	lavVideoProcessor::release();
}

void lav::setDatabasePath(char* databasePath) {
    char str[200];
    sprintf(str, "setDatabasePath %s\n", databasePath);
    SonifierLogger::logger(str);
	lavSoundDatabase::setDatabasePath(databasePath);
}

void lav::setDefaultDatabasePath(char* databasePath) {
    char str[200];
    sprintf(str, "setDefaultDatabasePath %s\n", databasePath);
    SonifierLogger::logger(str);
	lavSoundDatabase::setDefaultDatabasePath(databasePath);
}

void lav::startOrStopSound() {
    SonifierLogger::logger("Silence ... \n");
	lavVideoProcessor::changeStatus(lavVideoProcessor::SILENCE);
}

