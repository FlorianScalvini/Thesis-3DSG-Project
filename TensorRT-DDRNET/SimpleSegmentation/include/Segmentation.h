//
// Created by ubuntu on 23/01/23.
//

#ifndef SIMPLE_SEGMENTATION_H
#define SIMPLE_SEGMENTATION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <NvInfer.h>
#include <cuda.h>
#include <map>
#include <fstream>
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "NvInferRuntimeCommon.h"
#include "Logger.h"
#include "Segmentation.h"

/*
 *  C++ Semantic Neural Network Segmentation with TensorRT framework
 */


class Segmentation {
public :
    Segmentation(const std::string& pathNetwork);
    void doInference(cv::Mat);
    cv::Mat getOutput();
    void destroy();
    cv::Size getInputSize();
    cv::Size getOutputSize();

private:
    void loadNetwork(const std::string& pathNetwork); // Load Network weight
    void* _pBuffers[2]{};
    float * _pData{}; // Buffers d'entrée
    float * _pOutput{}; // Buffers de sortie
    nvinfer1::IRuntime* _pRuntime{};
    nvinfer1::ICudaEngine* _pEngine{};
    nvinfer1::IExecutionContext* _pContext{};
    cudaStream_t _stream{};
    Logger _gLogger;
    int _widthInput ;
    int _heightInput;
    int _widthOutput;
    int _heightOutput;
};


#endif //SIMPLE_SEGMENTATION_H
