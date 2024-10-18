//
// Created by ubuntu on 21/06/22.
//

#ifndef STAG_LAV_VOCAL_H
#define STAG_LAV_VOCAL_H

#include "../sound/sound_reader.h"
#include "sound/sound_reader_hrtf.h"
#include "lav_constants.h"
#include <vector>
#include <map>



class lavVocal {
public:
    static void init();
    static void* pull_buffer();
    static void start(unsigned int indice);
    static bool isReading();
    static short* emptyBuffer;

    static pthread_mutex_t _sound_mutex;
    static std::vector<SoundReader> sounds;
    static SoundReader* ptrSound;


};


#endif //STAG_LAV_VOCAL_H
