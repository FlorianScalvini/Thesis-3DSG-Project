//
// Created by florian on 01/09/2021.
//
#include "DetecteurLibtorch.h"

DetecteurLibtorch::DetecteurLibtorch(const std::string& pathNetwork)
{
    _imgSize = cv::Size(0,0);
    std::cout<<"Initialisation Network"<<std::endl;
    _module = torch::jit::load(pathNetwork);
    std::cout<<"Initialisation Network Done"<<std::endl;
    _widthNetwork = WIDTH_NETWORK;
    _heightNetwork = HEIGHT_NETWORK;
    if (torch::cuda::is_available()) {
        _device_type = torch::kCUDA;
    } else {
        _device_type = torch::kCPU;
    }
    _module.to(_device_type);
    _module.eval();
    cv::Mat firstInference(_widthNetwork, _heightNetwork, CV_8UC3, cv::Scalar(0, 0, 0));
    this->doInference(firstInference);
}


void DetecteurLibtorch::doInference(cv::Mat& img)
{
    cv::Mat image = letterBox(img);
    torch::Tensor imgTensor = torch::from_blob(image.data, {image.rows, image.cols,3},torch::kByte).to(_device_type);
    imgTensor = imgTensor.permute({2,0,1});
    imgTensor = imgTensor.toType(torch::kFloat);
    imgTensor = imgTensor.div(255);
    imgTensor = imgTensor.unsqueeze(0);
    // inference
    _preds = _module.forward({imgTensor}).toTuple()->elements()[0].toTensor().select(0, 0);;
}



std::map<unsigned int, std::vector<ObjectBoundingBox>> DetecteurLibtorch::getOutput()
{

    int det_size = 6; // 4 Bbox  + Conf + Class id
    std::map<unsigned int, std::vector<ObjectBoundingBox>> outputs;
    torch::Tensor scores = _preds.select(1, 4) * std::get<0>( torch::max(_preds.slice(1, 5, _preds.sizes()[1]), 1));
    _preds = torch::index_select(_preds, 0, torch::nonzero(scores > CONF_THRESH).select(1, 0));
    _preds = _preds.to(torch::kCPU);
    std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(_preds.slice(1, 5, _preds.sizes()[1]), 1);
    torch::Tensor  det2 = torch::cat({_preds.slice(1, 0, 4), std::get<0>(max_tuple).unsqueeze(1), std::get<1>(max_tuple).unsqueeze(1)}, 1);
    auto* tensor_raw_data_ptr = det2.data_ptr<float>();
    for(int i = 0 ; i < det2.sizes()[0]; i++)
    {
        if (tensor_raw_data_ptr[1 + det_size * i + 4] <= CONF_THRESH) continue;
        yoloToRect(&tensor_raw_data_ptr[1 + det_size * i]);
        ObjectBoundingBox det = ObjectBoundingBox(&tensor_raw_data_ptr[1 + det_size * i]);
        if (outputs.count((int)det.class_id) == 0)
            outputs.emplace((unsigned int) det.class_id, std::vector<ObjectBoundingBox>());
        outputs[(int)det.class_id].push_back(det);
    }
    nms(outputs);
    return outputs;
}
void DetecteurLibtorch::destroy() {}

