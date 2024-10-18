
#include "lav_synthesizer.h"

float lavSynthesizer::_base_amplitude = 0;

float lavSynthesizer::getWeightedAmplitude(float frequency) {

	float num_Ra_f = pow(12200., 2)*pow(frequency,4);
	float den_Ra_f_1 = pow(frequency, 2) + pow(20.6, 2);
	float den_Ra_f_2 = sqrt( ( pow(frequency, 2) + pow(107.7, 2) ) *  (pow(frequency, 2) + pow(737.9, 2) ) );
	float den_Ra_f_3 = pow(frequency, 2) + pow(12200, 2);

	float Ra_f = num_Ra_f/(den_Ra_f_1*den_Ra_f_2*den_Ra_f_3);

	float A_f = 2.+20*log10(Ra_f);

	//float weightForAmplitude = pow(10, A_f/40);//adaptation for bad headphones
	float weightForAmplitude = pow(10, A_f/20);//correct with good headphones

	return _base_amplitude*1./weightForAmplitude;

}


float lavSynthesizer::getAmplitudeRight(unsigned int ID_x) {
	float XVal = (float) ID_x/((float) FRAME_WIDTH_SONIFIED-1);
	//float balance = (float)1/((float)1.+(float)exp((float)-2.6*(XVal-(float)0.5)));
	float balance = (float)1/((float)1.+(float)exp((float)-6*(XVal-(float)0.5)));
	return balance;
}

float lavSynthesizer::getITDLeft(unsigned int ID_x) {

	float XVal = (float) ID_x/(float) FRAME_WIDTH_SONIFIED;

	float posXFromCenter = XVal-(float)0.5;
	float delay = (float)0.0875/(float)343.*((float)sin(posXFromCenter*PI)+posXFromCenter*PI);

	return -delay/2;

}


float lavSynthesizer::getFrequency(int idRow, int idCol) {

	    idRow = FRAME_HEIGHT_SONIFIED-1-idRow;

	    float nbPixLine = (float)FRAME_WIDTH_SONIFIED;
	    float nbPixCol = (float)FRAME_HEIGHT_SONIFIED;
	    //float minFreq = 250;
	    //float maxFreq = 2500;

	    float zmin = (float)2.50280542986;
        float zmax = (float)14.4980269058;

	    float xVal_tmp = (idRow*nbPixLine+idCol)/(float) (nbPixCol*nbPixLine);
	    float xVal = (xVal_tmp+idRow/(float)nbPixCol)/(float)2.;
	    float valueBark = zmin+(zmax-zmin)*xVal;
	    //float valueHzFromBark = bark2Hz(valueBark);
	    float valueHzFromBark = (float)1960.*(valueBark+(float)0.53)/((float)26.28-valueBark);
	    return valueHzFromBark;
}


void lavSynthesizer::exportToWav(float*** sound_db, char* databasePath) {

    lavLog::LAVLOG("exportToWav\n");
    std::unique_ptr <SoundWav> wavfile(new SoundWav());
    wavfile->createWavFile(databasePath);
	for (unsigned int ID_x =0; ID_x<FRAME_WIDTH_SONIFIED; ++ID_x) {
		float** curr_sound_db_x = sound_db[ID_x];

		for (unsigned int ID_y =0; ID_y<FRAME_HEIGHT_SONIFIED; ++ID_y) {
			float* curr_sound_db_x_y = curr_sound_db_x[ID_y];

            wavfile->addByteToWave((char*)curr_sound_db_x_y, SIZE_SOUND_IN_VALUE *4 );
		}
	}
    wavfile->closeWavFile();
}

void lavSynthesizer::initFromSynthesizing(char* databasePath) {

	lavLog::LAVLOG("initFromSynthesizing\n");

    _base_amplitude = 0.4;

	float*** sound_db  = (float***) calloc(FRAME_WIDTH_SONIFIED, sizeof(float**));

	srand(time(NULL));

	for (unsigned int ID_x =0; ID_x<FRAME_WIDTH_SONIFIED; ++ID_x) {

        //lavLog::LAVLOG("ID_x", ID_x);

		sound_db[ID_x] = (float**) calloc(FRAME_HEIGHT_SONIFIED, sizeof(float*));
		float** curr_sound_db_x = sound_db[ID_x];

		float rand_phase = 2*PI*rand();

		for (unsigned int ID_y =0; ID_y<FRAME_HEIGHT_SONIFIED; ++ID_y) {

            //lavLog::LAVLOG("ID_y", ID_y);

			curr_sound_db_x[ID_y] = (float*) calloc(SIZE_SOUND_IN_VALUE, sizeof(float));
			float* curr_sound_db_x_y = curr_sound_db_x[ID_y];

			float coefILDRight = getAmplitudeRight(ID_x);
			float coefILDLeft = 1-coefILDRight;

			float ITDLeft = getITDLeft(ID_x);
			float ITDRight = -ITDLeft;

			float frequency = getFrequency(ID_y, ID_x);
			float weightedAmplitude = getWeightedAmplitude(frequency);
			//float weightedAmplitude = 0.4;
			float amplitudeLeft = weightedAmplitude*coefILDLeft;
			float amplitudeRight = weightedAmplitude*coefILDRight;

			//to leave if rand phase
			rand_phase = 2*PI*rand();

			//for test purposes
			/*
			frequency = 440;
			amplitudeLeft = 0.01;//10;//0.3;
			amplitudeRight = 0.01;//10;//0.3;
			ITDLeft = 0;
			ITDRight = 0;
			rand_phase = 3.14159/2;
			*/
			//end for test purposes

			float time = 0;

			for (int ID_t =0; ID_t<SIZE_SOUND_IN_VALUE; ID_t+=2) {
				curr_sound_db_x_y[ID_t] = amplitudeLeft *sin(2.*PI*frequency*(time+ITDLeft)+ rand_phase);
				curr_sound_db_x_y[ID_t+1]= amplitudeRight *sin(2.*PI*frequency*(time+ITDRight) + rand_phase);
				time += 1/44100.;
			}

			for (int ID_t =0; ID_t<SIZE_AUDIO_CHUNK_IN_VALUE; ID_t+=2) {
				curr_sound_db_x_y[ID_t] = curr_sound_db_x_y[ID_t]*lavConstants::_sigmoidal_fade_in[ID_t];
				curr_sound_db_x_y[ID_t+1] = curr_sound_db_x_y[ID_t+1]*lavConstants::_sigmoidal_fade_in[ID_t+1];
			}

			for (int ID_t =SIZE_SOUND_IN_VALUE-SIZE_AUDIO_CHUNK_IN_VALUE; ID_t<SIZE_SOUND_IN_VALUE; ID_t+=2) {
				curr_sound_db_x_y[ID_t] = curr_sound_db_x_y[ID_t]*lavConstants::_sigmoidal_fade_out[ID_t-(SIZE_SOUND_IN_VALUE-SIZE_AUDIO_CHUNK_IN_VALUE)];
				curr_sound_db_x_y[ID_t+1] = curr_sound_db_x_y[ID_t+1]*lavConstants::_sigmoidal_fade_out[ID_t+1-(SIZE_SOUND_IN_VALUE-SIZE_AUDIO_CHUNK_IN_VALUE)];
			}

		}
	}

	exportToWav(sound_db, databasePath);

	for (unsigned int ID_x =0; ID_x<FRAME_WIDTH_SONIFIED; ++ID_x) {
		for (unsigned int ID_y =0; ID_y<FRAME_HEIGHT_SONIFIED; ++ID_y) {
			delete[] sound_db[ID_x][ID_y];
		}
		delete[] sound_db[ID_x];
	}
	delete[] sound_db;

}

