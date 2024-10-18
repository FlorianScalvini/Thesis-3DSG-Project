#include <iostream>

using namespace std;

#include "Sonifier/lav.h"
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// Clears the window and draws the tetrahedron.  The tetrahedron is  easily
// specified with a triangle strip, though the specification really isn't very
// easy to read.

//pulseaudio -k && sudo alsa force-reload && sleep 2 && pulseaudio -k && sudo alsa force-reload

int main(int argc, char** argv) {
    srand(time(nullptr));
    lav::start();
    getchar();
    lav::stop();
    return 0;
}
