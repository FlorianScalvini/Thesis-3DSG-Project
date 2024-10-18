//
// Created by florian on 02/07/2021.
//
#ifndef LAV_REALSENSE_CAMERA_H
#define LAV_REALSENSE_CAMERA_H

/*
 *
 * Customized classe inherented from VideoCapture for an Intel Realsense Stereophonic Camera
 *
 */

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <unistd.h>
#include "video_capture.h"

#define DEC_FILTER 1
#define SPAT_FILTER 1
#define TEMP_FILTER 1

#define FOV_X 69
#define FOV_Y -1

class RealsenseCamera: public VideoCapture {

public:
    RealsenseCamera();
    static RealsenseCamera* __singleton;
    static RealsenseCamera* getSingleton();
    void start() override;
    void getNextFrame(cv::Mat&, cv::Mat&) override;
    void release() override;
    void configureCamera(cv::Size frameSize) override;
    void pixelToWorld(float *pixel, float *point, float depth) override;

private:

    static cv::Mat frame_to_mat(const rs2::frame& f); // Convert Frame format from Intel Realsense SDK to OpenCV Mat

    rs2::pipeline pipeRealSense;
    rs2::config cfg;
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    const std::string disparity_filter_name = "Disparity";
    rs2::disparity_transform depth_to_disparity = rs2::disparity_transform(true);
    rs2::disparity_transform disparity_to_depth = rs2::disparity_transform(false);
    rs2::align* align_to_color; // Align Depth map to Color
    std::vector<rs2::filter> filters;
    rs2_intrinsics intrinsics; // Intrinsics paramaters of the camera
    rs2::frameset frames;

    cv::Mat colorTransfert;
    cv::Mat depthTransfert;

};

#endif //LAV_REALSENSE_CAMERA_H