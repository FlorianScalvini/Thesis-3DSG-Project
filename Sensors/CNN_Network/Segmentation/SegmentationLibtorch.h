//
// Created by Florian on 01/09/2023.
//

#ifndef SONIFIER_DETECTEURLIBTORCH_H
#define SONIFIER_DETECTEURLIBTORCH_H

#include <vector>
#include "Segmentation.h"
#include <torch/script.h>
#include <torch/torch.h>
#include <ATen/ATen.h>
#include <c10/cuda/CUDAStream.h>
#include <ATen/cuda/CUDAEvent.h>
#include <iostream>
#include <string>
#include <cuda_runtime.h>
#include <cuda.h>

#define WIDTH_NETWORK 1024
#define HEIGHT_NETWORK 512

class SegmentationLibtorch : public Segmentation {
public :
    explicit SegmentationLibtorch(const std::string& pathNetwork);
    void doInference(cv::Mat &) override;
    void getOutput(cv::Mat &) override;
    void destroy() override;

    private:
        torch::Tensor _preds;
        torch::jit::script::Module _module;
        torch::DeviceType _device_type;
};


#endif //SONIFIER_DETECTEURLIBTORCH_H
