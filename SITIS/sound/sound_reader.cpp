//
// Created by ubuntu on 21/06/22.
//

#include "sound_reader.h"



SoundReader::SoundReader() {
    currentChunkPtr = nullptr;
    lastChunkPtr = nullptr;
    numberChunks = 0;
    sizeAudioChunkSample = 0;
    sizeData = 0;
    isInit = false;
    bufferSound = nullptr;
}


int SoundReader::init(char* file, int sizechunks) {
    if(isInit)
    {
        delete [] bufferSound;
    }
    sizeAudioChunkSample = 2*sizechunks ;
    SoundWav soundWav((char *) file);
    soundWav.readHeader();
    sizeData = soundWav.getSizeData();
    if(soundWav.getNbChannel() != 1)
    {
        printf("Error, the file %s is not a monophonic sound");
        return EXIT_FAILURE;
    }
    numberChunks = ceil((sizeData) / sizechunks);
    bufferSound = new short [numberChunks *  sizeAudioChunkSample];
    memset(bufferSound, 0, numberChunks*sizeAudioChunkSample*sizeof(short));
    auto* tempArray = new short [sizeData];
    soundWav.readData(tempArray, sizeData);
    for(int i = 0; i < sizeData; i++)
    {
        bufferSound[2*i] = tempArray[i];
        bufferSound[2*i + 1] = tempArray[i];
    }

    currentChunkPtr = nullptr;
    lastChunkPtr = bufferSound + (numberChunks-1)*sizeAudioChunkSample;
    isInit = true;
    return EXIT_SUCCESS;
}

bool SoundReader::isReading()
{
    if(currentChunkPtr == nullptr || !isInit)
        return false;
    else
        return true;
}

void SoundReader::start()
{
    currentChunkPtr = bufferSound;
}

 void* SoundReader::pull_buffer()
{
    void* pointer_result = currentChunkPtr;
    if(currentChunkPtr == lastChunkPtr)
    {
        currentChunkPtr = nullptr;
        std::cout<<"END SOUND !"<<std::endl;
    }
    else
        currentChunkPtr += sizeAudioChunkSample;
    return pointer_result;
}
