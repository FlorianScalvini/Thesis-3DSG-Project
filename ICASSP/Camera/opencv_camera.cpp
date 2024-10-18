#include "opencv_camera.h"



OpencvCamera::OpencvCamera(int IDCamera)
{

    _capture = new cv::VideoCapture(IDCamera); 
    _capture->set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH_SONIFIED);
	_capture->set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT_SONIFIED);
    
}

cv::Mat OpencvCamera::getNextFrame()
{

    #if LAV_ANDROID
        if (_capture->grab()) {
            _capture->retrieve(_tmpMat, 1);// 1 for CV_CAP_ANDROID_GREY_FRAME
            if ((_tmpMat.size().width != FRAME_WIDTH_SONIFIED) || (_tmpMat.size().height != FRAME_HEIGHT_SONIFIED)) {
                cv::resize(_tmpMat, _tmpMat, cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), 0, 0, 1);
                return _inputMat
            }
            else {
                return _tmpMat;
            }
        }

    #else
        (*_capture) >> _tmpMat;
        if ((_tmpMat.size().width != FRAME_WIDTH_SONIFIED) || (_tmpMat.size().height != FRAME_HEIGHT_SONIFIED)) {
        cv::resize(_tmpMat, _tmpMat, cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), 0, 0, 1);
    }
        if (_tmpMat.channels() !=1) {
            cvtColor(_tmpMat, _tmpMat, cv::COLOR_RGB2GRAY);
            _tmpMat.convertTo(_inputMat, CV_8UC1);
            return _inputMat;
        }
    #endif

    
}

void OpencvCamera::release()
{
    _capture->release();
}
