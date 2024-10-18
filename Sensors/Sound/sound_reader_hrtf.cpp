//
// Created by Florian on 27/06/22.
//

#include "sound_reader_hrtf.h"
#include <math.h>


SoundReaderHrtf::SoundReaderHrtf(const char  * file, int nbAngle, int angleLimit, int size_sound_in_value) {

    SoundWav soundWav( file);
    soundWav.readHeader();
    unsigned int sizeData = soundWav.getSizeData();
    isInit = false;
    // The file %s is a monophonic sound
    if(soundWav.getNbChannel() != 2)
    {
        printf("Is not a stereophonic sound\n");
        return;
    }

    if(soundWav.getSizeData() % size_sound_in_value != 0)
    {
        printf("The file size is not divisible by %i\n", size_sound_in_value);
        return;
    }
    sizeAudioChunk = size_sound_in_value;
    sampleNb = nbAngle;
    sizeSample = (int)(sizeData / nbAngle);
    if(sizeData % sizeAudioChunk != 0 )
    {
        printf("The file size %i is not divisible by %i\n",sizeData, sizeAudioChunk);
        return;
    }
    if(sizeData % sampleNb != 0 )
    {
        printf("The file size %i is not divisible by the number of indicate angle%i\n",sizeData, sampleNb);
        return;
    }
    bufferSound = new short [sizeData];
    soundWav.readData(bufferSound, sizeData);
    currentPointer = emptyBuffer;
    lastChunk = bufferSound + sizeData - sizeAudioChunk;
    emptyBuffer = new short [sizeAudioChunk];
    memset(emptyBuffer, 0, sizeAudioChunk*sizeof(short));
    increAngle = abs(angleLimit / (nbAngle - 1));
    this->angleLimit = angleLimit;
    isInit = true;
}


//Return the sound corresponding to a resired angle
short* SoundReaderHrtf::getSpatializedSound(int angle){
    int idx = round((angle + angleLimit / 2) / increAngle);
    short * pointer_return = emptyBuffer;
    //std::cout<<"idx : " <<idx<<std::endl;
    if(isInit && idx >= 0 && idx < sampleNb)
    {
        int offset = round((float) idx / (float) increAngle) * sizeSample;
        pointer_return = bufferSound + offset;
    }
    return pointer_return;
}