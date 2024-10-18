#include "logger.h"

void SonifierLogger::logger(char* msg) {
    printf(msg);
    printf("\n");

}

void SonifierLogger::logger(const char* msg) {
    printf(msg);
    printf("\n");
}


void SonifierLogger::logger(char* nameOfValue, int value) {
    printf("%s: %i\n", nameOfValue, value);
}

void SonifierLogger::logger(char* nameOfValue, char* value) {
    printf("%s: %s\n", nameOfValue, value);
}


void SonifierLogger::displayImage(char* label, cv::Mat image) {
    #if DESKTOP
    cv::imshow(label,image);
    cv::waitKey(10);
    #endif
}

