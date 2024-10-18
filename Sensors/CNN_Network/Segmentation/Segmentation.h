//
// Created by Florian on 23/01/23.
//

#ifndef OUTDOORNAV_SEGMENTATION_H
#define OUTDOORNAV_SEGMENTATION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <map>


class Segmentation {
public:

    virtual void doInference(cv::Mat
    img) = 0; // Perform the inference
    virtual void getOutput(cv::Mat &img) = 0; // Return the segmented image
    virtual void destroy() = 0;

    // Size of the input image
    cv::Size getInputSize()
    {
        return cv::Size{_widthInput, _heightInput};
    }

    // Size of the segmented image
    cv::Size getOutputSize()
    {
        return cv::Size{_widthOutput, _heightOutput};
    }

protected:
    int _widthInput = 0;
    int _heightInput = 0;
    int _widthOutput = 0;
    int _heightOutput = 0;
};



#endif //OUTDOORNAV_SEGMENTATION_H
