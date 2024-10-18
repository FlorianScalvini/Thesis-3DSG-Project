#include "lav_log.h"

void lavLog::LOG(char* msg) {

    //LOGI(msg);
    printf(msg);
    printf("\n");

}

void lavLog::LOG(const char* msg) {

    //LOGI(msg);
    printf(msg);
    printf("\n");

}


void lavLog::LOG(char* nameOfValue, int value) {
    printf("%s: %i\n", nameOfValue, value);
}

void lavLog::LOG(char* nameOfValue, char* value) {
    printf("%s: %s\n", nameOfValue, value);
}


void lavLog::displayImage(char* label, cv::Mat image) {
    #if DESKTOP
    cv::imshow(label,image);
    cv::waitKey(10);
    #endif
}

