//
// Created by florian on 02/07/2021.
//

#include "realsense_camera.h"



RealsenseCamera* RealsenseCamera::__singleton = nullptr;

RealsenseCamera* RealsenseCamera::getSingleton()
{
    if (__singleton ==nullptr) {
        __singleton = new RealsenseCamera();
    }
    return __singleton;
}

RealsenseCamera::RealsenseCamera()
{
    align_to_color = new rs2::align(RS2_STREAM_COLOR);
    xFOV = FOV_X;
    yFOV = FOV_Y;
}

void RealsenseCamera::start()
{
    rs2::pipeline_profile profile = pipeRealSense.start(cfg);
    const rs2::stream_profile color_profile = profile.get_stream(RS2_STREAM_COLOR);
    intrinsics = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
    printf("Wait first camera frames ...");
    for(int i = 0; i < 30; i++)
    {
        pipeRealSense.wait_for_frames(); //Wait for all configured streams to produce a frame
    }
    printf("Camera starting ...");
}



void RealsenseCamera::release()
{
    pipeRealSense.stop();
}

void RealsenseCamera::configureCamera(cv::Size frameSize)
{
    printf("Initialize RealsenseCamera ...\n");
    rs2::context context;
    auto test = context.query_all_sensors();
    rs2::device_list devices = context.query_devices();
    if (devices.size() == 0)
    {
        printf("No device connected, please connect a RealSense device\n");
        usleep(500000);
        this->configureCamera(frameSize);
    }
    rs2::device device = devices.front(); // Select first device if multiple RealSense connected
    cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    filters.push_back(spat_filter);
    filters.push_back(temp_filter);
    cfg.enable_stream(RS2_STREAM_DEPTH, frameSize.width, frameSize.height, RS2_FORMAT_Z16,30); // Set Stream Output + Resolution + Format + Framerate
    cfg.enable_stream(RS2_STREAM_COLOR, frameSize.width, frameSize.height, RS2_FORMAT_RGB8,30);
    align_to_color = new rs2::align(RS2_STREAM_COLOR);
    printf("Camera configured\n");
}

void RealsenseCamera::pixelToWorld(float *pixel, float *point, float depth) {
    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
}

void RealsenseCamera::getNextFrame(cv::Mat& imgDepth, cv::Mat& imgColor){
    if(pipeRealSense.try_wait_for_frames(&frames, RS2_DEFAULT_TIMEOUT))
    {
        //if(align_to_color != nullptr)
        //   frames = align_to_color->process(frames);


        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::frame color = frames.get_color_frame();
        for(rs2::filter filter : filters)
        {
            depth = depth.apply_filter(filter);
        }
        if(align_to_color != nullptr)
           frames = align_to_color->process(frames);
        imgDepth = frame_to_mat(depth); // RS2_FORMAT_Z16 -> Mat C1
        imgColor = frame_to_mat(color);
    }
}

cv::Mat RealsenseCamera::frame_to_mat(const rs2::frame& f)
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

