//
// Created by ubuntu on 21/06/22.
//

#ifndef STAG_SOUND_READER_H
#define STAG_SOUND_READER_H

#include "sound_wav.h"


class SoundReader {
public:
    explicit SoundReader();
    virtual void* pull_buffer();
    void start();
    virtual int init(char* file, int sizechunks);
    virtual bool isReading();
    int sizeAudioChunkSample;
    int sizeData;
    short *bufferSound;
    short *lastChunkPtr;
    short *currentChunkPtr;
    int numberChunks;
    bool isInit;
private:
};


#endif //STAG_SOUND_READER_H
