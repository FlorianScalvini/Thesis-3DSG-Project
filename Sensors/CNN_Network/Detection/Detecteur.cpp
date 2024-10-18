//
// Created by Florian on 30/08/2021.
//

#include "Detecteur.h"

void Detecteur::convertMatToArray(const cv::Mat& img, float *data) const
{
    int i = 0;
    for(int row = 0; row < _heightNetwork; ++row)
    {
        uchar* uc_pixel = img.data + row * img.step;
        for(int col = 0; col < _widthNetwork; ++col)
        {
            data[i] = (float)(uc_pixel[2] / 255.0);
            data[_widthNetwork * _heightNetwork + i] = (float)(uc_pixel[1] / 255.0);
            data[_widthNetwork * _heightNetwork * 2 + i] = (float)(uc_pixel[0] / 255.0);
            uc_pixel += 3;
            ++i;
        }
    }
}

cv::Mat Detecteur::letterBox(const cv::Mat &input) const
{
    cv::Mat output;
    double h1 = _widthNetwork * (input.rows/(double)input.cols);
    double w2 = _heightNetwork * (input.cols/(double)input.rows);
    if( h1 <= _heightNetwork) {
        cv::resize( input, output, cv::Size(_widthNetwork, h1));
    } else {
        cv::resize( input, output, cv::Size(w2, _heightNetwork));
    }

    int top = (_heightNetwork-output.rows) / 2;
    int down = (_heightNetwork-output.rows+1) / 2;
    int left = (_widthNetwork - output.cols) / 2;
    int right = (_widthNetwork - output.cols+1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(128,128,128) );

    return output;
}



bool Detecteur::cmp(const ObjectBoundingBox& a, const ObjectBoundingBox& b) {
    return a.conf > b.conf;
}


void Detecteur::nms(std::map<unsigned int, std::vector<ObjectBoundingBox>>& output) {
    for (auto & it : output) {
        //std::cout << it->second[0].class_id << " --- " << std::endl;
        std::sort(it.second.begin(), it.second.end(), cmp);
        for (size_t m = 0; m < it.second.size(); ++m) {

            for (size_t n = m + 1; n < it.second.size(); ++n) {
                if (Rect::calcIoU(it.second[m].bbox, it.second[n].bbox) > NMS_THRESH) {
                    it.second.erase(it.second.begin() + n);
                    --n;
                }
            }
        }
    }
}

void Detecteur::yoloToRect(float bbox[4]) const {
    float r_w = _widthNetwork / static_cast<float>(_imgSize.width);
    float r_h = _heightNetwork / static_cast<float>(_imgSize.height);
    float ratio = (r_h > r_w) ? r_w : r_h;
    float padding_x = (r_h > r_w) ? 0 : (_widthNetwork - r_h * _imgSize.width) / 2;
    float padding_y = (r_h > r_w) ? (_heightNetwork - r_w * _imgSize.height) / 2 : 0;

    float l = bbox[0] - bbox[2] / 2.f - padding_x;
    float r = bbox[0] + bbox[2] / 2.f - padding_x;
    float t = bbox[1] - bbox[3] / 2.f - padding_y;
    float b = bbox[1] + bbox[3] / 2.f - padding_y;

    bbox[0] = std::max((float)0, (float) l / ratio);
    bbox[1] = std::max((float)0, (float) t / ratio);
    bbox[2] = std::min((float)_imgSize.width - 1, (float) (r - l) / ratio);
    bbox[3] = std::min((float)_imgSize.height - 1, (float) (b - t) / ratio);
}