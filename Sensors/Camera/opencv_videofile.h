#ifndef LAV_OPENCV_VIDEOFILE
#define LAV_OPENCV_VIDEOFILE


#include <unistd.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifdef DESKTOP
#include <opencv2/videoio.hpp>
#endif
#include "../../lav_constants.h"
#include "video_capture.h"

class OpencvVideoFile: public VideoCapture
{

    private:

        struct timespec _requestStart, _requestEnd;

		cv::Mat _inputMat;
		cv::Mat _tmpMat;
        cv::Mat _blackMat;

        int _maxUsSleepingPeriod;
        int _usTime;

        bool _isFirstFrame;

        cv::VideoCapture* _capture;

    public:

        OpencvVideoFile(char* filename, int fps);

        cv::Mat getNextFrame();
        void release();

};

#endif
