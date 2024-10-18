//
// From https://github.com/bbenligiray/stag
// 

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ImageSmooth.h"
#include "ImageSmoothCV.h"


///---------------------------------------------------------------------------------------------------------------------------------------
/// Given an image of size widthxheight in srcImg, smooths the image using a gaussian filter (cvSmooth) and copies the smoothed image to smoothImg
/// If sigma=0.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 5, 5); This is the default.
/// If sigma>0.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 0, 0, sigma);
///
void SmoothImage(unsigned char *srcImg, unsigned char *smoothImg, int width, int height, double sigma){
    cv::Mat *iplImg1, *iplImg2;

    if (sigma <= 0){
        memcpy(smoothImg, srcImg, width*height);
        return;
    } //end-if
    iplImg1 = new cv::Mat(cv::Size(width, height), CV_8UC1, srcImg);
    iplImg2 = new cv::Mat(cv::Size(width, height), CV_8UC1, smoothImg);
    if (sigma == 1.0)
        cv::GaussianBlur(*iplImg1, *iplImg2, cv::Size(5,5), 0, 0);
    else if (sigma == 1.5)
        cv::GaussianBlur(*iplImg1, *iplImg2, cv::Size(7,7), 0, 0);  // seems to be better?
    else
        cv::GaussianBlur(*iplImg1, *iplImg2, cv::Size(7, 7), sigma);

} //end-SmoothImage

///---------------------------------------------------------------------------------------------------------------------------------------
/// Given an image of size widthxheight in srcImg, smooths the image using a gaussian filter (cvSmooth) and copies the smoothed image to smoothImg
/// If sigma=0.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 5, 5); This is the default.
/// If sigma>0.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 0, 0, sigma);
///
void SmoothImage(cv::Mat *iplImg1, unsigned char *smoothImg, double sigma){
    cv::Mat *iplImg2;

    if (sigma <= 0){
        std::memcpy(smoothImg, iplImg1->data, iplImg1->total());
        return;
    } //end-if

    int width = iplImg1->cols;
    int height = iplImg1->rows;

    iplImg2 = new cv::Mat(cv::Size(width, height), CV_8UC1, 1);
    memcpy(&iplImg2, smoothImg, iplImg2->total());
    if (sigma == 1.0)
        cv::GaussianBlur(*iplImg1, *iplImg2, cv::Size(5,5), 0, 0);
    else
        cv::GaussianBlur(*iplImg1, *iplImg2, cv::Size(7, 7), sigma);
} //end-SmoothImage

///------------------------------------------------------------------------------------------
/// Perform Gauss filter on "src" and store the result in "dst"
///
static void GaussFilter(unsigned char *src, unsigned char *dst, int width, int height){

  memcpy(dst, src, width*height);
  for (int i=2; i<height-2; i++){
    for (int j=2; j<width-2; j++){
      dst[i*width+j] =  
        (2*src[(i-2)*width+j-2] + 4*src[(i-2)*width+j-1] + 5*src[(i-2)*width+j] + 4*src[(i-2)*width+j+1] + 2*src[(i-2)*width+j+2] +
         4*src[(i-1)*width+j-2] + 9*src[(i-1)*width+j-1] + 12*src[(i-1)*width+j] + 9*src[(i-1)*width+j+1] + 4*src[(i-1)*width+j+2] +
         5*src[i*width+j-2] + 12*src[i*width+j-1] + 15*src[i*width+j] + 12*src[i*width+j+1] + 5*src[i*width+j+2] +
         4*src[(i+1)*width+j-2] + 9*src[(i+1)*width+j-1] + 12*src[(i+1)*width+j] + 9*src[(i+1)*width+j+1] + 4*src[(i+1)*width+j+2] +
         2*src[(i+2)*width+j-2] + 4*src[(i+2)*width+j-1] + 5*src[(i+2)*width+j] + 4*src[(i+2)*width+j+1] + 2*src[(i+2)*width+j+2] + 80) / 159;
    } //end-for
  } //end-for
} //end-GaussFilter


