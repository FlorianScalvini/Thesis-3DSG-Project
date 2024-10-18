#ifndef LAV_LOG
#define LAV_LOG

//#include <android/log.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>

class lavLog {

    public:

        static void LOG(char* msg);
        static void LOG(const char* msg);
        static void LOG(char* nameOfValue, int value);
        static void LOG(char* nameOfValue, char* value);

        static void displayImage(char* label, cv::Mat image);

};

#endif
