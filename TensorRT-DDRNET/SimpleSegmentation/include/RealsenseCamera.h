//
// Created by ubuntu on 11/04/24.
//

#ifndef SIMPLEYOLO_REALSENSECAMERA_H
#define SIMPLEYOLO_REALSENSECAMERA_H

#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <unistd.h>


class RealsenseCamera {
public:
    RealsenseCamera();
    ~RealsenseCamera();
    bool start(cv::Size frameSize);
    cv::Mat getColorImage();

private:
    static cv::Mat _frameToMat(const rs2::frame& f);
    rs2::pipeline pipeRealSense;
    rs2::config cfg;
    rs2::frameset frames;
    bool _isInit;
};

#endif