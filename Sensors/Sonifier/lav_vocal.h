//
// Created by ubuntu on 21/06/22.
//

#ifndef STAG_LAV_VOCAL_H
#define STAG_LAV_VOCAL_H

#include "Sound/sound_reader.h"
#include "Sound/sound_reader_hrtf.h"
#include "Common/utils.h"
#include "lav_constants.h"
#include <vector>
#include <map>



class lavVocal {
public:
    static void init(const std::string&);
    static void* pull_buffer();
    static void readSound(std::string);
    static bool isReading();

private:
    static short* emptyBuffer;
    static std::map<std::string, std::string>  _soundLst;
    static std::mutex _sound_mutex;
    static SoundReader ptrSound;
};


#endif //STAG_LAV_VOCAL_H
