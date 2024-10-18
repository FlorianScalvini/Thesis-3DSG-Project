//
// Created by ubuntu on 21/06/22.
//

#include "lav_vocal.h"


SoundReader lavVocal::ptrSound;
std::mutex lavVocal::_sound_mutex;
short* lavVocal::emptyBuffer;
std::map<std::string, std::string> lavVocal::_soundLst;

void lavVocal::init(const std::string& path)
{
    // List sound file into a map of file name and path
    if(getFilesInDirectory(path + "/Soundfiles/", _soundLst))
    {
        perror("Error while listing sound files");
    }
    if(getFilesInDirectory(path + "/Soundfiles/number/", _soundLst))
    {
        perror("Error while listing sound files");
    }
    emptyBuffer = new short[SIZE_AUDIO_CHUNK_IN_VALUE];
    ptrSound = SoundReader();
    _sound_mutex.unlock();
};

void lavVocal::readSound(std::string name)
{
    auto it = _soundLst.find(name);
    if (it != _soundLst.end()) {
        _sound_mutex.lock();
        ptrSound.init(it->second.c_str(), SIZE_AUDIO_CHUNK_IN_SAMPLE);
        ptrSound.start();
        _sound_mutex.unlock();
    } else {
        fprintf(stderr, "The key %s is not associated with a file in the database.", name.c_str());
    }
}

bool lavVocal::isReading() {
    return ptrSound.isReading();
}

void* lavVocal::pull_buffer()
{
    void * return_pointer = emptyBuffer;
    _sound_mutex.lock();
    return_pointer = ptrSound.pull_buffer();
    _sound_mutex.unlock();
    return return_pointer;
};

