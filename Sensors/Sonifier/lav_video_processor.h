//
// Created by Florian on 08/12/22.
//
#ifndef LAV_VIDEO_PROCESSOR
#define LAV_VIDEO_PROCESSOR

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>

#include "lav_manager.h"
#include "lav_constants.h"
#include "lav_imu_acquisition.h"
#include "lav_camera_acquisition.h"
#include "logger.h"
#include "lav_sonifier.h"
#include "lav_audio_stream.h"
#include "Tracking/Sort/Sort.h"
#include "Tracking/ByteTrack/ByteTrack.h"
#include "Tracking/TrackingAlgorithm.h"
#include "CNN_Network/Detection/Detecteur.h"
#include "CNN_Network/Detection/DetecteurTensorRT.h"
#include "CNN_Network/Segmentation/Segmentation.h"
#include "CNN_Network/Segmentation/SegmentationRT.h"
#include "CNN_Network/Detection/yolov8/Yolov8TRT.h"


class lavVideoProcessor {
    public:
        enum VideoState
        {
            SILENCE = 0,
            NAV = 1,
            CROSSWALK_SIGNAL_PATH = 2,
            TRAFFIC_LIGHT_SEARCH = 3,
            SAVE = 4
        };
        static void init(const char*, const char*);
        static void start();
        static void release();
        static void changeStatus(VideoState state); // Change the status of the video processing process
        static void* start_video_stream(void* args);
        static void start_thread_video_stream();
        static int getPathCorrection(int angle); // Correction of the path with
        static void setSavePath(std::string);
        static void _computeSegmentation(); // Segmentation by CNN
    private:
        static cv::Mat _computeNearPixel(); // Detect near elements
        static bool newValueIsAvailable();
        static void _captureFramesAndIMU();
        static void _computeObstacle(); // Compute obstacles : Near & distant
        static void _computeTracking(); // Object tracking
        static void _computeCrosswalk(); // Segmentation : Not yet implemented
        static void _computeTrafficLight(); // Detection of pedestrian traffic light
        static void _saveImg();
        static void _computeObjectDetection(); // Object detection
        static bool _isPredominantlyGreen(const cv::Mat& img); // Check if the color predominant on the image is green
        static void _processVideo();
        static bool _hazardMouvement(const Eigen::Vector3d&, const Eigen::Vector3d&);

        static Detecteur* _detectObstacle;
        static cv::Mat _inputMat;
        static cv::Mat _inputMatColor;
        static std::mutex _mutexImg;
        static std::mutex _mutexSeg;
        static std::vector<ObjectBoundingBox>  _staticObstacle;
        static std::vector<ObjectBoundingBox> _dynamicObstacle;
        static bool _newVideoProcessingIsAvailable;
        static bool _closeVideo;
        static cv::Point3f _positionMotion;
        static Sort* _track;
        static Segmentation* _segSidewalk;
        static int status;
        static std::string _savePath;
        static cv::Mat _segImg;
        static int pedestrianLightStatus;
        static Eigen::Quaterniond _previousOrientation;
        static Eigen::Quaterniond _orientation;
};

#endif