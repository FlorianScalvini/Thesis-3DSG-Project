//
// Created by Florian on 08/12/22.
//
#ifndef LAV_WAV
#define LAV_WAV

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>

#include "logger.h"
#include "lav_constants.h"
#include "../Sound/sound_wav.h"

namespace lavWav
{
    int loadWav(float*** sound_db, char* filename); // Load a wav file corresponding to a HRTF database
}

#endif
