#ifndef LAV_WAV
#define LAV_WAV

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>

#include "lav_log.h"
#include "lav_constants.h"

class lavWav {



	private:

		#define ID_RIFF 0x46464952
		#define ID_INFO 0x4f464e49
		#define ID_INAM 0x4d414e49
		#define ID_WAVE 0x45564157
		#define ID_FMT  0x20746d66
		#define ID_DATA 0x61746164

		#define FORMAT_PCM 1
		#define FORMAT_FLOAT 0x0003

		struct Wav_header {
			uint32_t riff_id;
			uint32_t riff_sz;

				uint32_t wave_id;

					uint32_t info_id;
					uint32_t info_sz;

					uint32_t name_id;
					uint32_t name_sz;
					//char name_data[95];
					char name_data[0];//but 95 char...strange

					uint32_t fmt_id;
					uint32_t fmt_sz;
						uint16_t audio_format;
						uint16_t num_channels;
						uint32_t sample_rate;
						uint32_t byte_rate;
						uint16_t block_align;
						uint16_t bits_per_sample;
					uint32_t data_id;
					uint32_t data_sz;
		};

		/*struct info_header {
			uint32_t info_id;
			uint32_t info_sz;
			uint32_t nb_pos_x;
			uint32_t nb_pos_y;
			uint32_t first_pos_id;
			uint32_t nb_sample_per_sound;
		};*/

		static FILE *_wav_file;
		static int _copied_nb_frames;
		static struct Wav_header _wav_header;


    public:

	    static void closeWavFile();

	    static int addByteToWave(char* data, unsigned int nbByte);

	    static int createWavFile(char* databasePath);

	    static int loadWav(float*** sound_db, char* filename);

        static int create_data_wav();


        static int writeStandardWavFile(char* filepath, char* format, char* data, unsigned int nbByte);

};

#endif
