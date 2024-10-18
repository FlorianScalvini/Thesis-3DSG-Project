//
// Created by florian on 30/08/2021.
//

#ifndef SONIFIER_DETECTEUR_TENSORRT_H
#define SONIFIER_DETECTEUR_TENSORRT_H

#include "Detecteur.h"
#include <NvInfer.h>
#include <cuda.h>
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include <map>
#include <fstream>      // std::ifstream
#include "NvInferRuntimeCommon.h"
#include "yolo/yololayer.h"
#include "./utils/logger.h"


class DetecteurTensorRT : public Detecteur {
public :
    DetecteurTensorRT(const std::string& pathNetwork);
    void doInference(cv::Mat& img) override;
    std::map<unsigned int, std::vector<ObjectBoundingBox>> getOutput() override;
    void destroy() override;

protected:
    void preprocess(cv::Mat &img);
    void loadNetwork(const std::string& pathNetwork);
    const int OUTPUT_SIZE = 1000 * sizeof(ObjectBoundingBox) / sizeof(float) + 1;
    void* _pBuffers[2];
    float * _pData{}; // Buffers d'entrée
    float * _pProb{}; // Buffers de sortie
    nvinfer1::IRuntime* _pRuntime{};
    nvinfer1::ICudaEngine* _pEngine{};
    nvinfer1::IExecutionContext* _pContext{};
    cudaStream_t _stream;
    Logger _gLogger;
};


#endif //SONIFIER_DETECTEUR_TENSORRT_H
