//
// Created by Florian on 23/01/23.
//

#ifndef OUTDOORNAV_SEGMENTATIONRT_H
#define OUTDOORNAV_SEGMENTATIONRT_H


#include "Segmentation.h"
#include <NvInfer.h>
#include <cuda.h>
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include <map>
#include <fstream>      // std::ifstream
#include "NvInferRuntimeCommon.h"
#include "../utils/logger.h"

/*
 *  C++ Semantic Neural Network Segmentation with TensorRT framework
 */


class SegmentationTensorRT : public Segmentation {
public :
    explicit SegmentationTensorRT(const std::string& pathNetwork);
    void doInference(cv::Mat) override;
    void getOutput(cv::Mat &) override;
    void destroy() override;

private:
    void loadNetwork(const std::string& pathNetwork); // Load Network weight
    void* _pBuffers[2];
    float * _pData{}; // Buffers d'entrée
    float * _pOutput{}; // Buffers de sortie
    nvinfer1::IRuntime* _pRuntime{};
    nvinfer1::ICudaEngine* _pEngine{};
    nvinfer1::IExecutionContext* _pContext{};
    cudaStream_t _stream;
    Logger _gLogger;
};


#endif //OUTDOORNAV_SEGMENTATIONRT_H
