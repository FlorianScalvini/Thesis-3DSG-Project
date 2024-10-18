//
// Created by florian on 01/09/2021.
//

#ifndef SONIFIER_DETECTEURLIBTORCH_H
#define SONIFIER_DETECTEURLIBTORCH_H

#include <vector>
#include "Detecteur.h"
#include <torch/script.h>
#include <torch/torch.h>
#include <ATen/ATen.h>
#include <c10/cuda/CUDAStream.h>
#include <ATen/cuda/CUDAEvent.h>
#include <iostream>
#include <string>
#include <cuda_runtime.h>
#include <cuda.h>

#define WIDTH_NETWORK 640
#define HEIGHT_NETWORK 640

class DetecteurLibtorch : public Detecteur {
public :
    DetecteurLibtorch(const std::string& pathNetwork);
    void doInference(cv::Mat& img);
    std::map<unsigned int, std::vector<ObjectBoundingBox>> getOutput();
    void destroy();

    private:
        torch::Tensor _preds;
        torch::jit::script::Module _module;
        torch::DeviceType _device_type;
};


#endif //SONIFIER_DETECTEURLIBTORCH_H
