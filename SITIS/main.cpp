#include "sonifier/lav.h"
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main() {
/*
    cv::Mat _inputMatColor = cv::imread("/home/ubuntu/CLionProjects/SoundPathFinding/5 distorted quads.png", cv::IMREAD_COLOR);
    cv::Mat img;
    Stag stagDetector = Stag(15, 7, false);

    cv::cvtColor(_inputMatColor, img, cv::COLOR_BGR2GRAY);
    stagDetector.detectMarkers(img);
    Drawer::drawMarkers(&img, stagDetector.markers);
    stagDetector.logResults("/home/ubuntu/CLionProjects/SoundPathFinding/");
    std::cout<<"";

*/
    lav::start();
    getchar();
    lav::stop();

}

