#ifndef LAV_VIDEO_CAPTURE
#define LAV_VIDEO_CAPTURE

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

class VideoCapture {

public:

    virtual void start() = 0;
    virtual void release() = 0;
    virtual void getNextFrame(cv::Mat&, cv::Mat&) = 0;
    virtual void pixelToWorld(float *pixel, float *point, float depth) = 0;
    virtual void configureCamera(cv::Size frameSize) = 0;
    static int pixelToAng(int pixel, int fov, int nbPixel)
    {
        double rst =  ((pixel - nbPixel/2.0) / (nbPixel/2.0)) * (fov/2.0);
        return (int)round(rst);
    }

    static int AngToPixel(int angle, int fov, int widthImg)
    {
        // Calculate the pixel position based on the angle
        int pixel = static_cast<int>(angle * widthImg / fov);
        return pixel;
    }

    int getXFOV(){
        return xFOV;
    }
    int getYFOV(){
        return yFOV;
    }

protected:
    int xFOV;
    int yFOV;
};

#endif
