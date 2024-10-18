#ifndef CAMERA
#define CAMERA

#include <climits>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>

/*#include "/home/maxime/system/OpenCV-android-sdk/sdk/native/jni/include/opencv2/core.hpp"
#include </home/maxime/system/OpenCV-android-sdk/sdk/native/jni/include/opencv2/imgproc.hpp>
#include </home/maxime/system/OpenCV-android-sdk/sdk/native/jni/include/opencv2/highgui.hpp>
*/

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#define CLEAR(x) memset (&(x), 0, sizeof (x))

class Camera {


	uint8_t *buffer;

	cv::Mat cvGrayMat;
	cv::Mat cvYUYVMat;

	//int n_buffers = 4;
	struct v4l2_buffer buf;

	int fd;

	int _widthImage;//FRAME_WIDTH_SONIFIED;
	int _heightImage;//FRAME_HEIGHT_SONIFIED;	

	public:

		Camera(int IDCam, int width, int height);
		cv::Mat getNextFrame();
		void closeAcquisition();

    private:

		int print_caps(int fd);

		int init(int fd);

		void getBuffer();

		void convertToOpenCv();

        static int xioctl(int fd, int request, void *arg);

};

#endif
