//
// Created by Florian on 20/06/22.
//

#ifndef STAG_SOUND_WAV_H
#define STAG_SOUND_WAV_H

#include <iostream>
#include <cmath>
#include <unistd.h>
#include <pthread.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <cstring>

#define HEADER_SIZE 44
#define FORMAT_PCM 1
#define FORMAT_FLOAT 0x0003
#define ID_RIFF 0x46464952
#define ID_INFO 0x4f464e49
#define ID_INAM 0x4d414e49
#define ID_WAVE 0x45564157
#define ID_FMT  0x20746d66
#define ID_DATA 0x61746164

struct Wav_header {

    uint32_t riff_id; // RIFF Header Magic header
    uint32_t riff_sz; // RIFF Chunk Size

    uint32_t wave_id; // WAVE Header

    uint32_t fmt_id; // FMT header
    uint32_t fmt_sz; // Size of the fmt chunk
    uint16_t audio_format; // Audio format 1=PCM,6=mulaw,7=alaw,     257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM
    uint16_t num_channels; // Number of channels 1=Mono 2=Sterio
    uint32_t sample_rate; // Sampling Frequency in Hz
    uint32_t byte_rate; // bytes per second
    uint16_t block_align;  // 2=16-bit mono, 4=16-bit stereo
    uint16_t bits_per_sample; // Number of bits per sample
    uint32_t data_id; // "data"  string
    uint32_t data_sz; // Sampled data length
};


class SoundWav{
public:
    SoundWav(const char * filename);
    SoundWav(){};
    ~SoundWav();
    static int writeStandardWavFile(const char* filename, char* format, char* data, unsigned int nbByte);
    void closeWavFile();
    int addByteToWave(char* data, unsigned int nbByte);
    int createWavFile(char* databasePath);
    int readHeader();
    void readData(short* data, unsigned int nb_item);
    unsigned int getSizeData() const;
    unsigned int getSizeHeader() const;
    unsigned int getNbChannel() const;
    unsigned int getNbbytes() const;
    Wav_header* getHeader();

private:
    Wav_header header{};
    const char * filename;
    int sizeHeader;

    unsigned int sizeData;
};


#endif