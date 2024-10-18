#include <iostream>
#include "RealsenseCamera.h"
#include "Segmentation.h"

int main() {
    // Initialisation de la caméra
    RealsenseCamera camera;
    camera.start(cv::Size(640,480));

    // Initialisation du modele de segmentation
    Segmentation _segNetwork("//home//ubuntu//CLionProjects//SimpleSegmentation//ressources//ddrnet.engine");
    cv::Mat img, segImg;


    while(true)
    {
        img = camera.getColorImage(); // Get image
        _segNetwork.doInference(img); // Run inférence
        segImg = _segNetwork.getOutput(); // Get résult
        cv::resize(segImg, segImg, cv::Size(img.cols, img.rows), 0,0,cv::INTER_NEAREST);
        cv::cvtColor(segImg * 50, segImg, cv::COLOR_GRAY2BGR);
        cv::addWeighted(img, 0.7, segImg, 0.3, 0, segImg);
        cv::imshow("Image", segImg);
        cv::waitKey(1);
    }
}
