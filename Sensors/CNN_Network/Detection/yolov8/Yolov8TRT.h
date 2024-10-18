//
// From https://github.com/wang-xinyu/tensorrtx
//

#ifndef OUTDOORNAV_YOLOV8TRT_H
#define OUTDOORNAV_YOLOV8TRT_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "../DetecteurTensorRT.h"
#include "../Detecteur.h"



class Yolov8TRT : public DetecteurTensorRT{
    public :
        Yolov8TRT(const std::string& pathNetwork) : DetecteurTensorRT(pathNetwork) {};
        std::map<unsigned int, std::vector<ObjectBoundingBox>> getOutput() override;
    private:
        void yoloV8ToRect(float bbox[4]) const;
};


#endif //OUTDOORNAV_YOLOV8TRT_H
