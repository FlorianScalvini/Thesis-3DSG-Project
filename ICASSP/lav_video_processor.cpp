
#include "lav_video_processor.h"


cv::Size size(320, 240);//SHORT_RANGE
cv::Mat lavVideoProcessor::_inputMat;
cv::Mat lavVideoProcessor::_inputMatColor;
cv::Mat lavVideoProcessor::_outputMat;
Detecteur* lavVideoProcessor::_detectPerson;
bool lavVideoProcessor::_silence = 0;
int lavVideoProcessor::_close_video = 0;
std::mutex lavVideoProcessor::_mutexImg;


void lavVideoProcessor::init(const char* detModelPath) {
    // Load Detection model with TensorRT framework
    _detectPerson = new DetecteurTensorRT(detModelPath);
    _outputMat = cv::Mat(cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), CV_8UC1);
}

void lavVideoProcessor::release() {
	_close_video = 1;
}

void lavVideoProcessor::startOrStopSound() {
	if (!_silence) {
		_silence = true;
	}
	else {
		_silence = false;
	}
}


void lavVideoProcessor::_captureFrames()
{
    _mutexImg.lock();
    lavCameraAcquisition::getNextFrame(_inputMat, _inputMatColor);
    _mutexImg.unlock();
}

void lavVideoProcessor::_processFrame() {
    // Perform inference on the color image
    _detectPerson->doInference(_inputMatColor);
    // Get results
    auto resultDetections = _detectPerson->getOutput();
    // For each object detected
    if (resultDetections.count(0) != 0)
    {
        for(int i = 0; i < resultDetections[0].size(); i++)
        {
            int x = (int)resultDetections[0][i].bbox.x();
            int y = (int)resultDetections[0][i].bbox.y();
            unsigned short _depth = _inputMat.at<unsigned short>((int)y*DEPTH_FRAME_WIDTH/COLOR_FRAME_WIDTH, (int)x*DEPTH_FRAME_WIDTH/COLOR_FRAME_WIDTH);
            // Ignore far objects
            if(_depth > 5000 || _depth < 30)
                _depth = 0;
            else if(_depth < 1000)
                _depth = 255;
            else
                _depth =  255*(1 - (_depth - 1000) / 4000);
            int x_out = (int)x*FRAME_WIDTH_SONIFIED/COLOR_FRAME_WIDTH;
            int y_out = (int)y*FRAME_HEIGHT_SONIFIED/COLOR_FRAME_WIDTH;
            _outputMat.at<unsigned char>(y_out, x_out) = _depth;
        }
    }
}

void lavVideoProcessor::acquireAndProcessFrame() {
    _outputMat.setTo(0);
    lavVideoProcessor::_captureFrames(); //  Capture frames
    lavVideoProcessor::_processFrame(); // Perform detection
    lavSonifier::sonify(&_outputMat); // Sonify
}


void* lavVideoProcessor::start_video_stream(void* args) {
	while (! _close_video) {
		acquireAndProcessFrame();
		//usleep(5000);
	}
	lavLog::LOG("video_close\n");
	//_pCam->closeAcquisition();
	return nullptr;
}


void lavVideoProcessor::start_thread_video_stream() {
	pthread_t thread_video_processing;
	pthread_create(&thread_video_processing, NULL, start_video_stream, (void*)NULL);
}