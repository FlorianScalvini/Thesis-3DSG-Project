//
// Created by ubuntu on 27/06/22.
//

/*
 *
 * Read an WAV file composed with multiple monophonic sounds convoluted by an HRTF at several horizontal position
 *
 */

#ifndef STAG_SOUND_READER_HRTF_H
#define STAG_SOUND_READER_HRTF_H

#include "sound_reader.h"

class SoundReaderHrtf {
public:
    SoundReaderHrtf(const char* file, int size_sound_in_value, int increAngle);
    short* getSpatializedSound(int idx);

private:
    bool isInit;
    int sampleNb;
    unsigned int angle;
    int sizeSample; // Taille d'un echantillion de la base de donnée
    short *bufferSound;
    short *emptyBuffer;
    int _increAngle;
};


#endif //STAG_SOUND_READER_HRTF_H
