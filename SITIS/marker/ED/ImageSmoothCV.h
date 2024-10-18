//
// From https://github.com/bbenligiray/stag
// 

#ifndef IMAGE_SMOOTH_CV_H
#define IMAGE_SMOOTH_CV_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/// Given an image of size widthxheight in srcImg, smooths the image using a gaussian filter (cvSmooth) and copies the smoothed image to smoothImg
/// If sigma=1.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 5, 5); This is the default.
/// If sigma>1.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 0, 0, sigma);
///
void SmoothImage(cv::Mat *srcImg, unsigned char *smoothImg, double sigma=1.0);

#endif