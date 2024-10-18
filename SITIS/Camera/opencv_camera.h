#ifndef LAV_OPENCV_CAMERA
#define LAV_OPENCV_CAMERA

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
//#include <opencv2/ocl/ocl.hpp>
#include "constant.h"
#include "video_capture.h"

class OpencvCamera: public VideoCapture
{

    private:

    cv::Mat _inputMat;
    cv::Mat _tmpMat;
    cv::VideoCapture* _capture;
    int _width;
    int _height;

    public:

    OpencvCamera(int IDCamera = 0);
    void start() override;
    cv::Mat getNextFrame();
    void release();
    void configureCamera(cv::Size frameSize) override;
};

#endif
