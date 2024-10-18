
#ifndef LAV_VIDEO_PROCESSOR
#define LAV_VIDEO_PROCESSOR

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lav_log.h"
#include "lav_sonifier.h"
#include "lav_audio_stream.h"
#include "lav_camera_acquisition.h"
#include "Detection/Detecteur.h"
#include "Detection/DetecteurTensorRT.h"
//for standard opencv

class lavVideoProcessor {

    private:
		static cv::Mat _inputMat;
        static cv::Mat _inputMatColor;
		static cv::Mat _outputMat;
		static cv::Mat _outputMatForDisplay;
        static cv::Mat _colorFrameForVideoRecording;
        static std::mutex _mutexImg;
        static Detecteur* _detectPerson;
    	static bool _silence;
        static int _close_video;

    private:
	public:
	    static void init(const char*);
	    static void release();
	    static void startOrStopSound();
        static void _captureFrames();
	    static void _processFrame();
	    static void acquireAndProcessFrame();
	    static void* start_video_stream(void* args);
	    static void start_thread_video_stream();

};


#endif
