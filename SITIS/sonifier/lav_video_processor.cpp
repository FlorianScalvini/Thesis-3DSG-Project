#include "lav_video_processor.h"
#include <utility>
#include "lav_camera_acquisition.h"

cv::Size sizeColor(COLOR_FRAME_WIDTH, COLOR_FRAME_WIDTH);//SHORT_RANGE

cv::Mat lavVideoProcessor::_inputMat;
cv::Mat lavVideoProcessor::_inputMatColor;
std::mutex lavVideoProcessor::_mutex;
DataVideoProcessing lavVideoProcessor::transData;

cv::Size size(320, 240);//SHORT_RANGE


bool lavVideoProcessor::_newValue;
bool lavVideoProcessor::_silence = false;
int lavVideoProcessor::_close_video = 0;


#ifdef PATH_MARKER
Stag lavVideoProcessor::stagDetector = Stag(15, 7, false);
#endif

void lavVideoProcessor::init() {
    _newValue = false;
    _silence = true;
}

void lavVideoProcessor::release() {

	_close_video = 1;

}

void lavVideoProcessor::start()
{
    _mutex.lock();
    _newValue = false;
    _silence = false;
    _mutex.unlock();
}

void lavVideoProcessor::stop()
{
    _mutex.lock();
    _newValue = false;
    _silence = true;
    _mutex.unlock();
}

void lavVideoProcessor::startOrStopSound() {
    _mutex.lock();
    _newValue = false;
	if (!_silence) {
		_silence = true;
	}
	else {
		_silence = false;
	}
    _mutex.unlock();
}

void lavVideoProcessor::_captureFrames()
{
    lavCameraAcquisition::getNextFrame(_inputMat, _inputMatColor);
}

void lavVideoProcessor::processFrame() {
    cv::Mat img = cv::Mat(sizeColor, CV_8UC1);

    /*
     *  Obstacle detection
     */

    cv::Mat _threshMat = cv::Mat(cv::Size(_inputMat.cols ,_inputMat.cols), CV_16UC1);
    cv::Mat _threshMatH = cv::Mat(cv::Size(_inputMat.cols ,_inputMat.cols), CV_16UC1);;
    cv::Mat _threshMatL= cv::Mat(cv::Size(_inputMat.cols ,_inputMat.cols), CV_16UC1);
    cv::threshold(_inputMat, _threshMatH, 100, 1, cv::THRESH_BINARY);

    cv::threshold(_inputMat, _threshMatL, 1300, 1, cv::THRESH_BINARY_INV);
    _threshMat = (_threshMatH & _threshMatL) * 255;
    _threshMat.convertTo(_threshMat, CV_8UC1, 1);

    /*
     * Marker detection
     */

    DataVideoProcessing data;
    cv::cvtColor(_inputMatColor, img, cv::COLOR_BGR2GRAY);
    stagDetector.detectMarkers(img);
    for(auto const& mrk: stagDetector.markers) // For each marker detected
    {
        int angle = (int)lavCameraAcquisition::pixelToAng((int)(mrk.center.x), FOV_X, COLOR_FRAME_WIDTH)  + (int)(FOV_X / 2);
        unsigned short distance = _inputMat.at<unsigned short>(mrk.center.y, mrk.center.x);
        data.data_path.push_back({(unsigned int)mrk.center.x, (unsigned int)mrk.center.y, distance, angle ,mrk.id});
    }
    cv::resize(_threshMat, data.obstacleMat, cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED));
    push_data(data);
}


void lavVideoProcessor::acquireAndProcessFrame() {
    _captureFrames();
    processFrame();
}

DataVideoProcessing lavVideoProcessor::pull_data()
{
    DataVideoProcessing dataOut;
    while(!_newValue)
        usleep(250);
    _mutex.lock();
    if(!_silence)
        dataOut = transData;
    _newValue = false;
    _mutex.unlock();
    return dataOut;
}

void lavVideoProcessor::push_data(DataVideoProcessing data) {
    _mutex.lock();
    transData = std::move(data);
    _newValue = true;
    _mutex.unlock();
}


void* lavVideoProcessor::start_video_stream(void* args) {
    //int i = 0;
    //auto t_start = std::chrono::high_resolution_clock::now();;
    while (!_close_video) {
        if(!_silence)
            acquireAndProcessFrame();
        else
            usleep(1000);
    }
    lavLog::LAVLOG("video_close\n");
    return nullptr;
}


void lavVideoProcessor::start_thread_video_stream() {
    //usleep(10000);
    pthread_t thread_video_processing;
    pthread_create(&thread_video_processing, nullptr, start_video_stream, (void*)nullptr);
}






