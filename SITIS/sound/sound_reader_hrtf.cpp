//
// Created by ubuntu on 27/06/22.
//

#include "sound_reader_hrtf.h"
#include <math.h>

SoundReaderHrtf::SoundReaderHrtf(const char* file, int size_sound_in_value, int increAngle) {

    SoundWav soundWav((char *) file);
    soundWav.readHeader();
    int sizeData = soundWav.getSizeData();
    isInit = false;
    // The file %s is a monophonic sound

    if(increAngle < 0)
    {
        printf("Negative value of the angular step\n");
        return;
    }

    _increAngle = increAngle;

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



    sizeSample = size_sound_in_value;
    sampleNb = (int)(sizeData / sizeSample);

    bufferSound = new short [sizeData];
    soundWav.readData(bufferSound, sizeData);

    emptyBuffer = new short [sizeSample];
    memset(emptyBuffer, 0, sizeSample*sizeof(short));
    isInit = true;

}

short* SoundReaderHrtf::getSpatializedSound(int idx)
{
    short * pointer_return;
    if(!isInit && idx < 0 || idx > sampleNb*_increAngle)
    {
        return emptyBuffer;
    }
    else
    {
        int offset = round((float) idx / (float) _increAngle) * sizeSample;
        //std::cout<<idx<<" "<<round((float) idx / (float) INCRE_ANGLE)<<std::endl;
        pointer_return = bufferSound + offset;
    }
    return pointer_return;
}


