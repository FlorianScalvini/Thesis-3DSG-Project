//
// Created by ubuntu on 21/06/23.
//

#ifndef OUTDOORNAV_LOGGER_H
#define OUTDOORNAV_LOGGER_H
#include <NvInfer.h>
#include <cuda.h>
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "NvInferRuntimeCommon.h"
#include <iostream>

class Logger : public nvinfer1::ILogger
{
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override
    {
        // suppress info-level messages
        if (severity != nvinfer1::ILogger::Severity::kINFO)
            std::cout << msg << std::endl;
    }
};


#endif //OUTDOORNAV_LOGGER_H
