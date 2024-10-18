//
// Created by Florian on 27/06/22.
//

#ifndef STAG_SOUND_READER_HRTF_H
#define STAG_SOUND_READER_HRTF_H

#include "sound_reader.h"
#include <vector>
class SoundReaderHrtf {
public:
    SoundReaderHrtf(const char* file, int nbAngle, int angleLimit, int size_sound_in_value);
    short* getSpatializedSound(int idx);

private:
    int elevationStart;
    int elevationInc;
    bool isInit;
    int sampleNb;
    int sizeAudioChunk;
    int increAngle;
    int sizeSample; // Taille d'un echantillion de la base de donnée
    short *bufferSound;
    short *lastChunk;
    short *emptyBuffer;
    short* currentPointer;
    int angleLimit;
};


#endif //STAG_SOUND_READER_HRTF_H
