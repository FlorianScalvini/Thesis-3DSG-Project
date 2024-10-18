#ifndef LAV_WAV
#define LAV_WAV

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>

#include "lav_log.h"
#include "lav_constants.h"
#include "../sound/sound_wav.h"

namespace lavWav
{
    int loadWav(float*** sound_db, char* filename);
}

#endif
