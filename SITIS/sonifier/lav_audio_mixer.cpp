
#include "lav_audio_mixer.h"


bool lavAudioMixer::_firstImageReceived = false;
int lavAudioMixer::_cptRecordedAudioChunk = 0;
bool lavAudioMixer::_recordOutputInWave = false;
int lavAudioMixer::_nbChunkToRecord=0;
short* lavAudioMixer::_arrayForWaveFile = 0;
char* lavAudioMixer::_recordingFileName = 0;


int lavAudioMixer::_IDVal = 0;
unsigned int lavAudioMixer::_cptVideoFrame = 0;
unsigned int lavAudioMixer::_buffer_as_swap = 0;
pthread_mutex_t lavAudioMixer::_buffer_mutex;
char* lavAudioMixer::_sonification_buffer_reading = 0;
char* lavAudioMixer::_audio_output_buffer = 0;
char* lavAudioMixer::_beginning_audio_buffer = 0;
char* lavAudioMixer::_pointer_curr_chunk = 0;
char* lavAudioMixer::_position_last_chunk_pointer = 0;
short* lavAudioMixer::_short_beginning_audio_buffer = 0;
short* lavAudioMixer::_short_last_chunk_audio_output_buffer = 0;
short* lavAudioMixer::_short_pointer_curr_chunk = 0;
short* lavAudioMixer::_short_mix_result = 0;


void lavAudioMixer::record_output(char* fileName, int durationInMs)
{

    
    _recordOutputInWave = true;
    _recordingFileName = fileName;
    _nbChunkToRecord = (int)((float) durationInMs*(float) AUDIO_SAMPLING_RATE/(1000.*(float) SIZE_AUDIO_CHUNK_IN_SAMPLE)+0.5);

    //lavLog::LAVLOG("record_output::_nbChunkToRecord", _nbChunkToRecord);
    _arrayForWaveFile = (short*) calloc(SIZE_AUDIO_CHUNK_IN_BYTE*_nbChunkToRecord, sizeof(char));
}

void lavAudioMixer::recordCurrentAudioChunk(void* currentAudioChunck)
{


    //lavLog::LAVLOG("recordCurrentAudioChunk", _cptRecordedAudioChunk);

    memcpy(&_arrayForWaveFile[SIZE_AUDIO_CHUNK_IN_VALUE*_cptRecordedAudioChunk], currentAudioChunck, SIZE_AUDIO_CHUNK_IN_BYTE);
    _cptRecordedAudioChunk +=1;

    if (_cptRecordedAudioChunk>_nbChunkToRecord) {
        _recordOutputInWave = false;
        SoundWav::writeStandardWavFile(_recordingFileName, (char *)"SHORT", (char*) _arrayForWaveFile, SIZE_AUDIO_CHUNK_IN_BYTE*_nbChunkToRecord);
        //lavLog::LAVLOG("RECORD !!!!", _recordingFileName);
    }
}

void lavAudioMixer::push_buffer(void* pointer_data)
{
	pthread_mutex_lock(&_buffer_mutex);
	_buffer_as_swap = 1;
	memcpy(_sonification_buffer_reading, pointer_data, SIZE_SOUND_IN_BYTE);
	//lavConstants::__startTimeChecking();

        ++_cptVideoFrame;
	if (_cptVideoFrame%30==0) {
	    lavConstants::__stopTimeChecking((char *)"lavAudioMixer::push_buffer");
	    lavConstants::__startTimeChecking();
	}
    //lavLog::LAVLOG(">> push_buffer", _cptVideoFrame);
   
	pthread_mutex_unlock(&_buffer_mutex);
}

void* lavAudioMixer::pull_buffer()
{

	void* pointer_result = NULL;
	//lavConstants::__startTimeChecking();

	pthread_mutex_lock(&_buffer_mutex);
	if (_buffer_as_swap) {

        _firstImageReceived = true;

		//lavConstants::__stopTimeChecking("mixer");
		_buffer_as_swap = 0;

		if (_pointer_curr_chunk == _position_last_chunk_pointer) {

			lavComputer::add_short_vector(_short_beginning_audio_buffer, (short*) _sonification_buffer_reading, _short_last_chunk_audio_output_buffer, SIZE_AUDIO_CHUNK_IN_VALUE);

			memcpy(_audio_output_buffer, _sonification_buffer_reading, SIZE_SOUND_IN_BYTE);

			pthread_mutex_unlock(&_buffer_mutex);

			_pointer_curr_chunk = _beginning_audio_buffer;
			pointer_result = _pointer_curr_chunk;
			_pointer_curr_chunk = _audio_output_buffer + SIZE_AUDIO_CHUNK_IN_BYTE;
		}
		else {
			lavComputer::mul_short_vector_by_float_vector(_short_mix_result, (short*) _pointer_curr_chunk, lavConstants::_sigmoidal_fade_out, SIZE_AUDIO_CHUNK_IN_VALUE);
			lavComputer::add_short_vector(_short_beginning_audio_buffer, (short*) _sonification_buffer_reading, _short_mix_result, SIZE_AUDIO_CHUNK_IN_VALUE);

			memcpy(_audio_output_buffer, _sonification_buffer_reading, SIZE_SOUND_IN_BYTE);

			pthread_mutex_unlock(&_buffer_mutex);
			_pointer_curr_chunk = _beginning_audio_buffer;
			pointer_result = _pointer_curr_chunk;
			_pointer_curr_chunk = _audio_output_buffer + SIZE_AUDIO_CHUNK_IN_BYTE;
		}

		//lavConstants::__stopTimeChecking("cycle");
	}

	else {
		if (_pointer_curr_chunk == _position_last_chunk_pointer) {

			lavComputer::add_short_vector(_short_beginning_audio_buffer,(short*) _audio_output_buffer, _short_last_chunk_audio_output_buffer, SIZE_AUDIO_CHUNK_IN_VALUE);

			pthread_mutex_unlock(&_buffer_mutex);

			_pointer_curr_chunk = _beginning_audio_buffer;
			pointer_result = _pointer_curr_chunk;
			_pointer_curr_chunk = _audio_output_buffer + SIZE_AUDIO_CHUNK_IN_BYTE;
		}
		else {
			pthread_mutex_unlock(&_buffer_mutex);
			pointer_result = _pointer_curr_chunk;
			_pointer_curr_chunk += SIZE_AUDIO_CHUNK_IN_BYTE;

		}
	}

    if ((_recordOutputInWave) and (_firstImageReceived)) {
        recordCurrentAudioChunk(pointer_result);
    }

	//lavConstants::__stopTimeChecking();

    //lavLog::LAVLOG("pull_buffer", _cptVideoFrame);
	return pointer_result;
}


void lavAudioMixer::init() {

    _IDVal=0;

	_buffer_as_swap = 0;

	_sonification_buffer_reading = (char*) calloc(SIZE_SOUND_IN_BYTE, sizeof(char));
	_audio_output_buffer = (char*) calloc(SIZE_SOUND_IN_BYTE, sizeof(char));
	_beginning_audio_buffer = (char*) calloc(SIZE_AUDIO_CHUNK_IN_BYTE, sizeof(char));

	_pointer_curr_chunk = _audio_output_buffer;
	_position_last_chunk_pointer = _audio_output_buffer +(NB_AUDIO_CHUNK_IN_SOUND-1)*SIZE_AUDIO_CHUNK_IN_BYTE;

	_short_beginning_audio_buffer = (short*) _beginning_audio_buffer;
	_short_last_chunk_audio_output_buffer = (short*) (_position_last_chunk_pointer);
	_short_mix_result = (short*) calloc(SIZE_AUDIO_CHUNK_IN_VALUE, sizeof(short));

	//_pointer_curr_chunk_converted_inFloat = (float*) calloc(SIZE_AUDIO_CHUNK_IN_VALUE, sizeof(float));

}

