#ifndef LOGGER
#define LOGGER

//#include <android/log.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>

class SonifierLogger {

    public:

        static void logger(char* msg);
        static void logger(const char* msg);
        static void logger(char* nameOfValue, int value);
        static void logger(char* nameOfValue, char* value);

        static void displayImage(char* label, cv::Mat image);

};

#endif
