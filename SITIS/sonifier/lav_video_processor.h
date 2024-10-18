
#ifndef LAV_VIDEO_PROCESSOR
#define LAV_VIDEO_PROCESSOR

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "marker/Drawer.h"
#include "marker/Stag.h"
#include "lav_manager.h"

//#include <android_native_app_glue.h>

#include "lav_log.h"
#include "lav_sonifier.h"
#include "lav_audio_stream.h"
#include <list>
#include "lav_manager.h"


struct VisualTarget{
    unsigned int x_pixel;
    unsigned int y_pixel;
    unsigned short distance;
    int angle;
    int label_i;
};

struct DataVideoProcessing
{
    std::vector<VisualTarget> data_path;
    cv::Mat obstacleMat = cv::Mat(cv::Size(FRAME_WIDTH_SONIFIED, FRAME_HEIGHT_SONIFIED), CV_8UC1);
};




class lavVideoProcessor {
public:
    private:
        static cv::Mat _inputMat;
        static cv::Mat _inputMatColor;
        static DataVideoProcessing transData;
        static Stag stagDetector;
    	static bool _silence;
        static int _close_video;
        static bool _newValue;
        static std::mutex _mutex;

	public:
	    static void init();
	    static void release();
	    static void startOrStopSound();
        static void _captureFrames();
	    static void processFrame();
	    static void acquireAndProcessFrame();
        static void push_data(DataVideoProcessing data);
        static DataVideoProcessing pull_data();
	    static void* start_video_stream(void* args);
        static void start_thread_video_stream();
        static void start();
        static void stop();
};

#endif