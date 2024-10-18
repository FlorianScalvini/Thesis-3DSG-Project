//
// Created by ubuntu on 11/04/24.
//


#include "RealsenseCamera.h"

RealsenseCamera::RealsenseCamera() : _isInit(false){}

RealsenseCamera::~RealsenseCamera() {
    pipeRealSense.stop();
    _isInit = false;
}

bool RealsenseCamera::start(cv::Size frameSize) {
    printf("Initialize RealsenseCamera ...\n");
    rs2::context context;
    auto test = context.query_all_sensors();
    rs2::device_list devices = context.query_devices();
    if (devices.size() == 0)
    {
        fprintf(stderr, "No device connected, please connect a RealSense device\n");
        _isInit = false;
        return false;
    }
    rs2::device device = devices.front(); // Select first device if multiple RealSense connected
    cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    cfg.enable_stream(RS2_STREAM_COLOR, frameSize.width, frameSize.height, RS2_FORMAT_RGB8,30);
    printf("Camera configured\n");
    rs2::pipeline_profile profile = pipeRealSense.start(cfg);
    for(int i = 0; i < 30; i++)
    {
        pipeRealSense.wait_for_frames(); //Wait for all configured streams to produce a frame
    }
    _isInit = true;
    return true;
}


cv::Mat RealsenseCamera::getColorImage() {
    if(!_isInit)
        return {cv::Size(0,0), CV_8UC3};
    if(pipeRealSense.try_wait_for_frames(&frames, RS2_DEFAULT_TIMEOUT))
    {
        rs2::frame color = frames.get_color_frame();
        return _frameToMat(color);
    }
    else
        return {cv::Size(0,0), CV_8UC3};
}

cv::Mat RealsenseCamera::_frameToMat(const rs2::frame& f)
{

    auto vf = f.as<rs2::video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return {cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP};
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        cv::Mat r_rgb = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat r_bgr;
        cv::cvtColor(r_rgb, r_bgr, cv::COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return {cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP};
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return {cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP};
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return {cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP};
    }

    throw std::runtime_error("Frame format is not supported yet!");
}
