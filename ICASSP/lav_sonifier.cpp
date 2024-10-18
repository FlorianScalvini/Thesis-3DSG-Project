#include "lav_sonifier.h"


short* lavSonifier::_short_sound = 0;
short* lavSonifier::_short_silence = 0;

int32_t* lavSonifier::_test_int32 = 0;

int lavSonifier::_sizeSimplificationChunk = 0;
int lavSonifier::_nbKeptPixInSimplificationChunk = 0;

short* lavSonifier::_short_audio_output = 0;
float* lavSonifier::_float_audio_output = 0;

#ifdef GRAYSCALE_SONIFICATION
float** lavSonifier::_float_grayscale_result = 0;
float** lavSonifier::_float_grayscale_modulation = 0;
int* lavSonifier::_grayscale_pixelCounter = 0;
int* lavSonifier::_startValueModulation = 0;
int* lavSonifier::_nbValueModulation = 0;
#endif


void lavSonifier::computeCompressionFactor(int nbActivePixel) {

	float nbTotalKeptActivePixel = lavConstants::__maxNbSonifiedPixel*(1-lavConstants::__maxNbSonifiedPixel/((float)nbActivePixel+lavConstants::__maxNbSonifiedPixel));

	//float nbKeptHotPixel = 500*(1-500/((float)nbHotPix+500));
	float ratio = nbTotalKeptActivePixel/ (float)nbActivePixel;

	_sizeSimplificationChunk = 64;

	if (ratio>1./20) {
		_nbKeptPixInSimplificationChunk = (int) ((ratio*(float) _sizeSimplificationChunk)+0.5);
	}
	else {
		_sizeSimplificationChunk =  (int) (1./ratio);
		_nbKeptPixInSimplificationChunk = 1;
	}
}

void lavSonifier::sonify(cv::Mat* mAbsDiffFrame) {

	//unrolled 01-01 01:27:44.440: I/lav_native(4458): sonify: 3567, 3080, 4877, 3018, 11683
	//normal   01-01 01:37:29.950: I/lav_native(4576): sonify: 3564, 3206, 5126, 1761, 12144



	cv::Mat inputMat = (*mAbsDiffFrame);
	int nbActivePixel = countNonZero(inputMat);
    int grayscale = 0;

	memset(_float_audio_output, 0, SIZE_SOUND_IN_VALUE*sizeof(float));
	memset(_short_audio_output, 0, SIZE_SOUND_IN_VALUE*sizeof(short));

    memset(_grayscale_pixelCounter, 0, 256*sizeof(int));
    for (int idGrayscale =0; idGrayscale<256; ++idGrayscale) {
	    memset(_float_grayscale_result[idGrayscale], 0, SIZE_SOUND_IN_VALUE*sizeof(float));
    }

	if (nbActivePixel>0) {

		computeCompressionFactor(nbActivePixel);

		int cptPixInCompression = 0;
		int nbSonifiedPix = 0;
		float* sound = NULL;

		unsigned int ID_y,ID_x, ID_t;
		uchar* p;

		for( ID_y = 0; ID_y < FRAME_HEIGHT_SONIFIED; ++ID_y)
		{
			p = inputMat.ptr<uchar>(ID_y);
			for ( ID_x = 0; ID_x < FRAME_WIDTH_SONIFIED; ++ID_x)
			{
                grayscale = p[ID_x];
				if (grayscale >0)  {
					if ((cptPixInCompression < _nbKeptPixInSimplificationChunk)) {
						sound = lavSoundDatabase::getSound(ID_x, ID_y);

                        int idStartValue = _startValueModulation[grayscale];
                        int nbValue = _nbValueModulation[grayscale];

                        lavComputer::add_float_vector(&_float_grayscale_result[grayscale][idStartValue], &sound[idStartValue], nbValue);


                        _grayscale_pixelCounter[grayscale]+=1;

						nbSonifiedPix +=1;
					}
					cptPixInCompression += 1;

					if (cptPixInCompression == _sizeSimplificationChunk) {
						cptPixInCompression = 0;
					}
 				}
			}
		}

        int startValueModulation =0;
        int nbValueModulation =0;

        for (int idGrayscale =0; idGrayscale<256; ++idGrayscale) {
            //lavLog::LOG("grayscale", idGrayscale);
            //lavLog::LOG("counter", _grayscale_pixelCounter[idGrayscale]);

            startValueModulation = _startValueModulation[idGrayscale];
            nbValueModulation = _nbValueModulation[idGrayscale];


            if (_grayscale_pixelCounter[idGrayscale]>1) {
                lavComputer::mul_float_vector(&_float_grayscale_result[idGrayscale][startValueModulation], &_float_grayscale_result[idGrayscale][startValueModulation], &_float_grayscale_modulation[idGrayscale][startValueModulation], nbValueModulation);
                lavComputer::add_float_vector(&_float_audio_output[startValueModulation], &_float_grayscale_result[idGrayscale][startValueModulation], nbValueModulation);
            }
        }

		exit_loop:
		for (ID_t =0; ID_t<SIZE_SOUND_IN_VALUE; ++ID_t) {
			_short_audio_output[ID_t] = (short) _float_audio_output[ID_t];
		}
	}
	lavAudioMixer::push_buffer(_short_audio_output);
}


void lavSonifier::init() {

    _sizeSimplificationChunk = 0;
	_nbKeptPixInSimplificationChunk =0;

	_short_sound = (short*) calloc(SIZE_SOUND_IN_BYTE, sizeof(char));
	_short_silence = (short*) calloc(SIZE_SOUND_IN_BYTE, sizeof(char));

	_test_int32 = (int32_t*) calloc(SIZE_SOUND_IN_VALUE, sizeof(int32_t));

	float time = 0.;
	for (int ID_t =0; ID_t<SIZE_SOUND_IN_VALUE; ID_t+=2) {
		_short_sound[ID_t] = _short_sound[ID_t+1] = 10000*sin(2.*PI*440*time);
		time += 1/44100.;
	}

	_short_audio_output = (short*) calloc(SIZE_SOUND_IN_VALUE, sizeof(short));
	_float_audio_output = (float*) calloc(SIZE_SOUND_IN_VALUE, sizeof(float));

    #ifdef GRAYSCALE_SONIFICATION
    _float_grayscale_result = (float**) calloc(256, sizeof(float*));
    _float_grayscale_modulation  = (float**) calloc(256, sizeof(float*));
    _grayscale_pixelCounter = (int*) calloc(256, sizeof(int));
    _startValueModulation = (int*) calloc(256, sizeof(int));
    _nbValueModulation = (int*) calloc(256, sizeof(int));

	for (int idGrayscale =0; idGrayscale<256; ++idGrayscale) {
		_float_grayscale_result[idGrayscale] = (float*) calloc(SIZE_SOUND_IN_VALUE, sizeof(float));
        _float_grayscale_modulation[idGrayscale] = (float*) calloc(SIZE_SOUND_IN_VALUE, sizeof(float));
	}


    float maxGrayScale = 255.;
    float minGrayScale = (float) MIN_GRAYSCALE_SONIFICATION;
    float maxAmplitudeGrayScale = maxGrayScale-minGrayScale;

    //float minPeriod = 0.002;
    float minPeriod = 0.03;
    float maxPeriod = 1.;//cannot be more than 1 !
    float minAmplitude = 0.5;
    float maxAmplitude = 5.;
    float alphaPeriod = 5.;
    float alphaMaxValue = 1.5;

	for (int idGrayscale =0; idGrayscale<256; ++idGrayscale) {

        float normGrayScale = ((float)idGrayscale-minGrayScale)/maxAmplitudeGrayScale;

        if (normGrayScale>-0.0000001) {

            float period = minPeriod+maxPeriod*pow(1-normGrayScale, alphaPeriod);
            float maxValue = minAmplitude+(maxAmplitude-minAmplitude)*pow(normGrayScale, alphaMaxValue);

            int midIDSample=SIZE_SOUND_IN_SAMPLE/2;
            float midTime=((float) midIDSample)/44100.;

            float phase = PI-(2.*PI*midTime/period);

            
            int nbSampleInOnePeriod = (int) (period*44100.);
            int startIDSampleModulation = 0;
            int stopIDSampleModulation = SIZE_SOUND_IN_SAMPLE;
            if ((midIDSample-nbSampleInOnePeriod/2)>0) {
                startIDSampleModulation = midIDSample-nbSampleInOnePeriod/2;
                stopIDSampleModulation = midIDSample+nbSampleInOnePeriod/2;

                startIDSampleModulation = (startIDSampleModulation/8)*8;
                stopIDSampleModulation = (stopIDSampleModulation/8+1)*8;

                if ((startIDSampleModulation<0) || (startIDSampleModulation>SIZE_SOUND_IN_SAMPLE)) {
                    startIDSampleModulation = 0;
                    stopIDSampleModulation = SIZE_SOUND_IN_SAMPLE;
                }
            }

            _startValueModulation[idGrayscale] = startIDSampleModulation*2;
            _nbValueModulation[idGrayscale] = (stopIDSampleModulation-startIDSampleModulation)*2;

            float time =0;
            float gapTime = 1./44100.;
            int ID_value = 0;
            float value = 0;

	        for (int ID_sample =0; ID_sample<SIZE_SOUND_IN_SAMPLE; ++ID_sample) {
                ID_value = ID_sample*2;
                value = maxValue*(1-cos(2*PI*time/period+phase))/2;
		        _float_grayscale_modulation[idGrayscale][ID_value] = value;
                _float_grayscale_modulation[idGrayscale][ID_value+1] = value;
                
                time += gapTime;		    
	        }
        }

    }
    #endif

}


