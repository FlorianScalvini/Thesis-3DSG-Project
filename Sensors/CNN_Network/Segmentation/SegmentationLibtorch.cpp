//
// Created by Florian on 01/09/2023.
//
#include "SegmentationLibtorch.h"

SegmentationLibtorch::SegmentationLibtorch(const std::string& pathNetwork)
{
    std::cout<<"Initialisation Network"<<std::endl;
    _module = torch::jit::load(pathNetwork);
    std::cout<<"Initialisation Network Done"<<std::endl;
    _widthInput = WIDTH_NETWORK;
    _heightInput = HEIGHT_NETWORK;
    if (torch::cuda::is_available()) {
        _device_type = torch::kCUDA;
    } else {
        _device_type = torch::kCPU;
    }
    _module.to(_device_type);
    _module.eval();
    cv::Mat firstInference(_widthInput, _heightInput, CV_8UC3, cv::Scalar(0, 0, 0));
    this->doInference(firstInference);
}


void SegmentationLibtorch::doInference(cv::Mat& img)
{
    cv::Mat image;
    cv::resize(img, image, cv::Size(_widthInput, _heightInput));
    torch::Tensor imgTensor = torch::from_blob(image.data, {image.rows, image.cols,3}, torch::kByte).to(_device_type);
    imgTensor = imgTensor.permute({2,0,1});
    imgTensor = imgTensor.toType(torch::kFloat);
    imgTensor = imgTensor.div(255);
    imgTensor = imgTensor.unsqueeze(0);
    // inference
    _preds = _module.forward({imgTensor}).toTuple()->elements()[0].toTensor().select(0, 0);;
}



void SegmentationLibtorch::getOutput(cv::Mat & img)
{

    cv::Mat img_seg = cv::Mat(cv::Size(_widthOutput , _heightOutput), CV_8U);
    if(_pOutput != nullptr)
    {
        for (int i = 0; i < _widthOutput * _heightOutput; ++i)
        {
            img_seg.at<unsigned char>(i) = (unsigned char)_pOutput[i];
        }
        img_seg.copyTo(img);
    }
    else
        fprintf(stderr, "The output buffer is not allocated");
}
void SegmentationLibtorch::destroy() {}

