#include "opencv_videofile.h"



OpencvVideoFile::OpencvVideoFile(char* filename, int fps)
{

    _maxUsSleepingPeriod = 1000000/fps;



    _blackMat = cv::Mat(cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), CV_8UC1);
    _capture = new cv::VideoCapture(filename);     

    if (!_capture->isOpened()) {
        printf((char *)"\n!!!!! PROBLEM: Cannot open video file :", filename);
    }

    _isFirstFrame = true;
}

cv::Mat OpencvVideoFile::getNextFrame()
{
    //lavLog::LAVLOG("OpencvVideoFile::getNextFrame");
    int nbChannelHasBeenChanged =0;
   
    (*_capture) >> _tmpMat;

    //lavLog::LAVLOG("_tmpMat width", _tmpMat.size().width);
    //lavLog::LAVLOG("_tmpMat height", _tmpMat.size().height);

    if (_tmpMat.size().width !=0) {

        if ((_tmpMat.size().width != FRAME_WIDTH_SONIFIED) || (_tmpMat.size().height != FRAME_HEIGHT_SONIFIED)) {
            //lavLog::LAVLOG("OpencvVideoFile::getNextFrame");
            cv::resize(_tmpMat, _tmpMat, cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), 0, 0, 1);
            //lavLog::LAVLOG("OpencvVideoFile::getNextFrame2");
        }
        if (_tmpMat.channels() !=1) {
            cvtColor(_tmpMat, _tmpMat, cv::COLOR_RGB2GRAY);
            _tmpMat.convertTo(_inputMat, CV_8UC1);
            nbChannelHasBeenChanged = 1;

        }

        if (!_isFirstFrame) {

            clock_gettime(CLOCK_REALTIME, &_requestEnd);
            _usTime = (_requestEnd.tv_sec - _requestStart.tv_sec)*1000000  + (int) ((float) (_requestEnd.tv_nsec - _requestStart.tv_nsec)/1000. +0.5);                        
            //lavLog::LAVLOG("OpencvVideoFile::getNextFrame sleep", _maxUsSleepingPeriod-_usTime);
            int sleepingPeriod = _maxUsSleepingPeriod-_usTime-120;
            if (sleepingPeriod>0) {
                usleep(sleepingPeriod);
            }

            //clock_gettime(CLOCK_REALTIME, &_requestEnd);
            //_usTime = (_requestEnd.tv_sec - _requestStart.tv_sec)*1000000  + (int) ((float) (_requestEnd.tv_nsec - _requestStart.tv_nsec)/1000. +0.5);

            //lavLog::LAVLOG("OpencvVideoFile::getNextFrame 2", _usTime);
            
        }
        else {
            _isFirstFrame = false;
        }

        clock_gettime(CLOCK_REALTIME, &_requestStart);

        if (nbChannelHasBeenChanged) {

            return _inputMat;
        }
        else {           
            return _tmpMat;
        }
    }

    else {
        usleep(_maxUsSleepingPeriod);
        return _blackMat;
    }
    


}

void OpencvVideoFile::release()
{
    _capture->release();
}



