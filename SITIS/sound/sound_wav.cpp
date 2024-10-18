//
// Created by ubuntu on 20/06/22.
//

#include "sound_wav.h"

SoundWav::SoundWav(char *filepath) : filename(filepath)
{
    sizeData = -1;
    sizeHeader = -1;
    this->header.num_channels = 0;
}

SoundWav::~SoundWav() = default;


int SoundWav::getSizeData(){
    return this->sizeData;
}

int SoundWav::getSizeHeader(){
    return this->sizeHeader;
}

int SoundWav::getNbChannel() {
    return this->header.num_channels;
}

int SoundWav::getNbbytes()
{
    return this->header.bits_per_sample / 8;
}

int SoundWav::readHeader()
{
    FILE *_wav_file = fopen(filename, "rb");
    if (!_wav_file) {
        char str[100];
        sprintf(str, "Unable to load file %s\n", filename);
        std::cout<<str<<std::endl;
        return EXIT_FAILURE;
    }

    fseek(_wav_file, 0, SEEK_SET);
    unsigned char buffer[HEADER_SIZE]; // Headers size
    unsigned char* ptr = buffer;
    int i = 0;
    int index = 0;
    while(i < HEADER_SIZE)
    {
        fread(buffer+i, sizeof(unsigned char), 1, _wav_file);
        index++;
        if((i == 12 && buffer[12] != 0x66) || (i == 13 && buffer[13] != 0x6D) || (i == 14 && buffer[14] != 0x74) ||( i == 15 && buffer[15] != 0x20))
        {
            i = 12;
            continue;
        }
        if((i == 36 && buffer[36] != 0x64) || (i == 37 && buffer[37] != 0x61) || (i == 38 && buffer[38] != 0x74) || (i == 39 && buffer[39] != 0x61))
        {
            i = 36;
            continue;
        }
        if(buffer[i] == EOF)
        {
            std::cout<<"Error : EOF value"<<std::endl;
            break;
        }
        ptr++;
        i++;
    }
    fclose(_wav_file);
    sizeHeader = index;
    memcpy(&header ,buffer, HEADER_SIZE);
    sizeData =  8 * header.data_sz  / (header.bits_per_sample);
    return EXIT_SUCCESS;
}


int SoundWav::writeStandardWavFile(const char* filename, char* format, char* data, unsigned int nbByte) {

    //int nbSample = 10000;
    //short* sound = (short*) calloc(nbSample*2, sizeof(short));
    int nbBytePerSample = 0;
    int nbBitPerValue = 0;
    int uFormat = 0;
    int nbSample = 0;

    if (strcmp(format, "FLOAT")==0) {
        nbSample = nbByte/(2*sizeof(float));
        nbBytePerSample = sizeof(float)*2;
        nbBitPerValue = sizeof(float)*8;
        uFormat = FORMAT_FLOAT;

    }
    else {
        nbSample = nbByte/(2*sizeof(short));
        nbBytePerSample = sizeof(short)*2;
        nbBitPerValue = sizeof(short)*8;
        uFormat = FORMAT_PCM;
    }



    FILE* wavfile = nullptr;
    Wav_header wavheader;

    wavheader.riff_id = ID_RIFF;
    wavheader.riff_sz = 0;

    wavheader.wave_id = ID_WAVE;

    wavheader.fmt_id = ID_INFO;
    wavheader.fmt_sz = 12;
    wavheader.audio_format = uFormat;
    wavheader.num_channels = 2;
    wavheader.sample_rate = 44100;
    wavheader.bits_per_sample = nbBitPerValue;
    wavheader.byte_rate = (wavheader.bits_per_sample / nbBytePerSample) * wavheader.num_channels * wavheader.sample_rate;
    wavheader.block_align = wavheader.num_channels * (wavheader.bits_per_sample / nbBytePerSample);
    wavheader.data_id = ID_DATA;

    wavfile = fopen(filename, "wb");
    if (!wavfile) {
        char str[100];
        sprintf(str, "Unable to create file %s\n", filename);
        printf(str);
        return 1;
    }

    fseek(wavfile, sizeof(struct Wav_header), SEEK_SET);


    //if (fwrite((void*)sound, 1, nbByte, wav_file) != nbByte) {
    if (fwrite(data, 1, nbByte, wavfile) != nbByte) {
        printf("Error writing recorded data to Wav file\n");
        return 0;
    }

    int copied_nb_frames = nbByte/nbBytePerSample;

    wavheader.data_sz = copied_nb_frames * wavheader.block_align;
    wavheader.riff_sz = wavheader.data_sz + sizeof(wavheader) - 8;
    fseek(wavfile, 0, SEEK_SET);
    fwrite(&wavheader, sizeof(struct Wav_header), 1, wavfile);
    fclose(wavfile);
    return 1;

}

Wav_header* SoundWav::getHeader()
{
    return &header;
}


void SoundWav::readData(short* data, int nbItem) {
    FILE * wav_file = fopen(filename, "rb");
    fseek(wav_file, sizeHeader, SEEK_SET);
    if(header.bits_per_sample == 16)
    {
        fread(data, sizeof(short), nbItem, wav_file);
    }
    else if(header.bits_per_sample == 8)
    {
        char dataChar[nbItem / 2];
        fread(dataChar, sizeof(char), sizeof(char) * nbItem /2 , wav_file);
        for (int i = 0 ; i < nbItem /2 ; ++i)
        {
            data[i] = dataChar[i];
        }

    }
    fclose(wav_file);
}




int SoundWav::createWavFile(char *databasePath)
{

}

void SoundWav::closeWavFile() {
}

int SoundWav::addByteToWave(char* data, unsigned int nbByte) {

}

