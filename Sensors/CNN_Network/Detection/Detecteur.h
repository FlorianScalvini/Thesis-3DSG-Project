//
// Created by florian on 30/08/2021.
//

#ifndef SONIFIER_DETECTEUR_H
#define SONIFIER_DETECTEUR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include "Rect.h"

#define NMS_THRESH 0.3
#define CONF_THRESH 0.3



struct ObjectBoundingBox
{
    Rect bbox;
    float conf;  // bbox_conf * cls_conf
    float class_id;
    unsigned short depth;

    ObjectBoundingBox()
    {
        conf = 0;
        class_id = 0;
        depth = 0;
        bbox = Rect(-1,-1,-1,-1);
    }

    explicit ObjectBoundingBox(float* data)
    {
        this->bbox = Rect(data[0], data[1], data[2], data[3]);
        this->conf = data[4];
        this->class_id = data[5];
        this->depth = 0;
    }

    ObjectBoundingBox(float* bbox, float conf, float class_id) :
    conf(conf), class_id(class_id)
    {
        this->bbox = Rect(bbox[0], bbox[1], bbox[2], bbox[3]);
        depth = 0;
    }
};


class Detecteur {
    public:
        virtual void doInference(cv::Mat& img) = 0;
        virtual std::map<unsigned int, std::vector<ObjectBoundingBox>> getOutput() = 0;
        virtual void destroy() = 0;
        void yoloToRect(float bbox[4]) const;

    protected:
        int _widthNetwork;
        int _heightNetwork;
        cv::Size _imgSize;
        cv::Mat letterBox(const cv::Mat &input) const;
        void convertMatToArray(const cv::Mat& img, float* data) const;
        static bool cmp(const ObjectBoundingBox& a, const ObjectBoundingBox& b);
        static void nms(std::map<unsigned int, std::vector<ObjectBoundingBox>>& output);
};


#endif //SONIFIER_DETECTEUR_H
