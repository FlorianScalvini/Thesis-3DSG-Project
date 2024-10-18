
#include "lav_wav.h"

FILE* lavWav::_wav_file = 0;
int lavWav::_copied_nb_frames = 0;
struct lavWav::Wav_header lavWav::_wav_header;

void lavWav::closeWavFile() {
	_wav_header.data_sz = _copied_nb_frames * _wav_header.block_align;
	_wav_header.riff_sz = _wav_header.data_sz + sizeof(_wav_header) - 8;
	fseek(_wav_file, 0, SEEK_SET);
	fwrite(&_wav_header, sizeof(struct Wav_header), 1, _wav_file);
	fclose(_wav_file);
}

int lavWav::addByteToWave(char* data, unsigned int nbByte) {
	if (fwrite(data, 1, nbByte, _wav_file) != nbByte) {
		lavLog::LOG("Error capturing sample\n");
		return 0;
	}

	_copied_nb_frames += nbByte/8;

	return 1;
}

int lavWav::writeStandardWavFile(char* filepath, char* format, char* data, unsigned int nbByte) {

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

  

    FILE* wav_file = 0;
    struct Wav_header wav_header;

	wav_header.riff_id = ID_RIFF;
	wav_header.riff_sz = 0;

	wav_header.wave_id = ID_WAVE;

	wav_header.info_id = ID_INFO;
	wav_header.info_sz = 12;
	wav_header.name_id = ID_INAM;

	wav_header.name_sz = 0;
	//strcpy(wav_header.name_data, "XX");

    //wav_header.name_sz = 95;
	//strcpy(wav_header.name_data, "nb_pos_x=160 nb_pos_y=120 nb_sample_per_sound=2048 nb_byte_per_sound=16384 first_pos=BottomLeft");

	wav_header.fmt_id = ID_FMT;
	wav_header.fmt_sz = 16;

	wav_header.audio_format = uFormat;
	wav_header.num_channels = 2;
	wav_header.sample_rate = 44100;
	wav_header.bits_per_sample = nbBitPerValue;
	wav_header.byte_rate = (wav_header.bits_per_sample / nbBytePerSample) * wav_header.num_channels * wav_header.sample_rate;
	wav_header.block_align = wav_header.num_channels * (wav_header.bits_per_sample / nbBytePerSample);


	wav_header.data_id = ID_DATA;

	wav_file = fopen(filepath, "wb");
	if (!wav_file) {
        char str[100];
        sprintf(str, "Unable to create file %s\n", filepath);
		lavLog::LOG(str);
		return 1;
	}

	fseek(wav_file, sizeof(struct Wav_header), SEEK_SET);


	//if (fwrite((void*)sound, 1, nbByte, wav_file) != nbByte) {
	if (fwrite(data, 1, nbByte, wav_file) != nbByte) {
		lavLog::LOG("Error writing recorded data to Wav file\n");
		return 0;
	}

    int copied_nb_frames = nbByte/nbBytePerSample;

    wav_header.data_sz = copied_nb_frames * wav_header.block_align;
	wav_header.riff_sz = wav_header.data_sz + sizeof(wav_header) - 8;
	fseek(wav_file, 0, SEEK_SET);
	fwrite(&wav_header, sizeof(struct Wav_header), 1, wav_file);
	fclose(wav_file);

	return 1;

}

int lavWav::createWavFile(char* databasePath) {

    lavLog::LOG("lavWav::createWavFile\n");

    _copied_nb_frames = 0;

	_wav_header.riff_id = ID_RIFF;
	_wav_header.riff_sz = 0;

	_wav_header.wave_id = ID_WAVE;

	_wav_header.info_id = ID_INFO;
	_wav_header.info_sz = 12;
	_wav_header.name_id = ID_INAM;


	//_wav_header.name_sz = 95;
	//strcpy(_wav_header.name_data, "nb_pos_x=160 nb_pos_y=120 nb_sample_per_sound=2048 nb_byte_per_sound=16384 first_pos=BottomLeft");

    lavLog::LOG("strcpy\n");
	_wav_header.name_sz = 0;
	//strcpy(_wav_header.name_data, "");
    lavLog::LOG("strcpy OK\n");


	_wav_header.fmt_id = ID_FMT;
	_wav_header.fmt_sz = 16;

	_wav_header.audio_format = FORMAT_FLOAT;//FORMAT_PCM;
	_wav_header.num_channels = 2;
	_wav_header.sample_rate = 44100;
	_wav_header.bits_per_sample = 32;//16;
	_wav_header.byte_rate = (_wav_header.bits_per_sample / 8) * _wav_header.num_channels * _wav_header.sample_rate;
	_wav_header.block_align = _wav_header.num_channels * (_wav_header.bits_per_sample / 8);


	_wav_header.data_id = ID_DATA;

	_wav_file = fopen(databasePath, "wb");
	if (!_wav_file) {
        char str[100];
        sprintf(str, "Unable to create file %s\n", databasePath);
		lavLog::LOG(str);
		return 1;
	}

	fseek(_wav_file, sizeof(struct Wav_header), SEEK_SET);
}

/*int create_data_wav() {

	createWavFile();

	int sizeInShort = 20;
	int sizeInByte = sizeInShort*2;

	short* buffer = (short*) calloc(sizeInShort, sizeof(short));

	float time = 0;

	for (int ID_sample=0; ID_sample<sizeInShort; ID_sample +=2) {
		buffer[ID_sample] = buffer[ID_sample+1]= 10000*sin(2*PI*440*time);
		time += 1/(float)44100;
	}

	addByteToWave((char*) buffer, sizeInByte);
	addByteToWave((char*) buffer, sizeInByte);

	closeWavFile();

}*/

int lavWav::loadWav(float*** sound_db, char* filename) {

    char str[200];
    sprintf(str, "Trying to load database %s\n", filename);
	lavLog::LOG(str);

	_wav_file = fopen(filename, "rb");
	if (!_wav_file) {
        char str2[200];
        sprintf(str2, "!!!!!!! Warning: Unable to load file %s\n", filename);
	    lavLog::LOG(str2);
		return 1;
	}

    char str3[200];
    sprintf(str3, "File found %s\n", filename);
    lavLog::LOG(str3);

	fseek(_wav_file, sizeof(struct Wav_header), SEEK_SET);

	for (int ID_x =0; ID_x<FRAME_WIDTH_SONIFIED; ++ID_x) {

		sound_db[ID_x] = (float**) calloc(FRAME_HEIGHT_SONIFIED, sizeof(float*));
		float** curr_sound_db_x = sound_db[ID_x];

		for (int ID_y =0; ID_y<FRAME_HEIGHT_SONIFIED; ++ID_y) {
			curr_sound_db_x[ID_y] = (float*) calloc(SIZE_SOUND_IN_VALUE, sizeof(float));
			float* curr_sound_db_x_y = curr_sound_db_x[ID_y];
			fread(curr_sound_db_x_y, sizeof(float), SIZE_SOUND_IN_VALUE, _wav_file);
			//to use with RK3188 and not for RK3288 ???
			//fseek(_wav_file, SIZE_SOUND_IN_VALUE, SEEK_CUR);

		}
	}

	return 0;

}


