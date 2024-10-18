#include "camera.h"
#include "../../lav_constants.h"

int Camera::xioctl(int fd, int request, void *arg)
{
    return ioctl (fd, request, arg);
}

Camera::Camera(int IDCam, int width, int height)
{
    _widthImage = width;//FRAME_WIDTH_SONIFIED;
    _heightImage = height;//FRAME_HEIGHT_SONIFIED;


	buf = {0};

	cvGrayMat = cv::Mat(_heightImage, _widthImage, CV_8UC1);
	cvYUYVMat = cv::Mat(_heightImage, _widthImage, CV_8UC2);

	char strB[12];
	snprintf(strB, sizeof(strB), "/dev/video%d", IDCam);
    char str[80];
    sprintf(str, "open camera %s\n", strB);
	printf(str);

	//fd = open(str, O_RDWR | O_NONBLOCK);
	fd = open(str, O_RDWR);
	//fd = open(str, O_RDONLY);


	if (fd == -1)
	{
        char str[80];
        sprintf(str, "Opening video device: %s\n", strerror(errno));
		printf(str);

	}

	printf("init");
	if(print_caps(fd)) {
		printf("pb init");
	}

	printf("mmap");
	if(init(fd)) {
		printf("pb init_mmap");
	}

}


int Camera::print_caps(int fd)
{
	struct v4l2_capability caps = {};
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
	{
            char str[80];
            sprintf(str, "Querying Capabilities: %s", strerror(errno));
			printf(str);
			return 1;
	}

    char strLong[800];
    sprintf(strLong, "Driver Caps:\n"
			"  Driver: \"%s\"\n"
			"  Card: \"%s\"\n"
			"  Bus: \"%s\"\n"
			"  Version: %d.%d\n"
			"  Capabilities: %08x\n",
			caps.driver,
			caps.card,
			caps.bus_info,
			(caps.version>>16)&&0xff,
			(caps.version>>24)&&0xff,
			caps.capabilities);

	printf(strLong);


	struct v4l2_cropcap cropcap;//MAX = {0};
	CLEAR (cropcap);//MAX
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
	{
            char str[100];
            sprintf(str, "Querying Cropping Capabilities: %s", strerror(errno));
			printf(str);
			return 1;
	}

    char strLong2[800];
    sprintf(strLong2, "Camera Cropping:\n"
			"  Bounds: %dx%d+%d+%d\n"
			"  Default: %dx%d+%d+%d\n"
			"  Aspect: %d/%d\n",
			cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
			cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
			cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

	printf(strLong2);


	struct v4l2_fmtdesc fmtdesc = {0};
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	char fourcc[5] = {0};
	char c, e;
	printf("  FMT : CE Desc\n--------------------\n");
	while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
	{
		strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
		c = fmtdesc.flags & 1? 'C' : ' ';
		e = fmtdesc.flags & 2? 'E' : ' ';

        char strBis[800];
        sprintf(strBis, "  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        printf(strBis);

		fmtdesc.index++;
	}

	struct v4l2_format fmt;//MAX = {0};
	CLEAR(fmt);//MAX
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = _widthImage;
	fmt.fmt.pix.height = _heightImage;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420;
	//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;//MAX
	//fmt.fmt.pix.field = V4L2_FIELD_TOP;//MAX


	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
	{
		printf("pb Setting Pixel Format");
		return 1;
	}



	strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);

    char strLong3[1000];
    sprintf(strLong3, "Selected Camera Mode:\n"
			"  Width: %d\n"
			"  Height: %d\n"
			"  PixFmt: %s\n"
			"  Field: %d\n",
			fmt.fmt.pix.width,
			fmt.fmt.pix.height,
			fourcc,
			fmt.fmt.pix.field);

	printf(strLong3);
	

	struct v4l2_queryctrl qctrl;

	qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
	while (0 == ioctl (fd, VIDIOC_QUERYCTRL, &qctrl)) {

        char strTmp[100];
        sprintf(strTmp, "query ctrl %s %d", qctrl.name, qctrl.default_value);
        printf(strTmp);
		qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}


	return 0;
}

int Camera::init(int fd)
{

	/*v4l2_control c;
	CLEAR(c);
	c.id = V4L2_CID_EXPOSURE_AUTO;
	c.value = V4L2_EXPOSURE_MANUAL;
	if(xioctl(fd, VIDIOC_S_CTRL, &c) == -1)
		LOGI("exposure ctrl: %s", strerror(errno));
	*/

	struct v4l2_requestbuffers req;
	CLEAR(req);
	req.count = 1;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
	{
        char strTmp[100];
        sprintf(strTmp, "Requesting Buffer: %s", strerror(errno));
        printf(strTmp);
		return 1;
	}

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;

	if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
	{
        char strTmp[100];
        sprintf(strTmp, "Querying Buffer: %s", strerror(errno));
        printf(strTmp);
		return 1;
	}

	//buf.length = buf.length/2; //MAX


	buffer = (uint8_t*) mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

    char str5[100];
    sprintf(str5, "Length: %d\nAddress: %p\n", buf.length, buffer);
    printf(str5);

    char str6[100];
    sprintf(str6, "Image Length: %d\n", buf.bytesused);
    printf(str6);


	if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
	{
        char strTmp[100];
        sprintf(strTmp, "Error opening Capture: %s", strerror(errno));
        printf(strTmp);

		//return 1;
	}

	return 0;
}

void Camera::getBuffer() {//mmap

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;

	lavConstants::__startTimeChecking();

	if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
	{
        char strTmp[100];
        sprintf(strTmp, "Error query Buffer: %s", strerror(errno));
        printf(strTmp);
	}

	if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
	{
        char strTmp[100];
        sprintf(strTmp, "Error retrieving Frame: %s", strerror(errno));
        printf(strTmp);
	}

	lavConstants::__stopTimeChecking((char *)"camera acquire");

}

void Camera::convertToOpenCv() {

	cvYUYVMat.data = (uchar*) buffer;
	//cv::cvtColor(cvYUYVMat, cvGrayMat, cv::COLOR_YUV2GRAY_YUYV);

	//cv::cvtColor(cvYUYVMat, cvGrayMat, cv::COLOR_YUV420sp2GRAY);
}

cv::Mat Camera::getNextFrame()
{

	getBuffer();


	convertToOpenCv();



	return cvGrayMat;

}

void Camera::closeAcquisition() {

    printf("Close camera acquisition");

	if(-1 == xioctl(fd, VIDIOC_STREAMOFF, &buf.type))
	{
        char strTmp[100];
        sprintf(strTmp, "Error closing Capture: %s", strerror(errno));
        printf(strTmp);

		//return 1;
	}

	close(fd);
}

