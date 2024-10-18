#include <assert.h>
#include <vector>
#include <iostream>
#include "argmax.h"
#include "../../utils/cuda_utils.h"

namespace Tn
{
    template<typename T>
    void write(char*& buffer, const T& val)
    {
        *reinterpret_cast<T*>(buffer) = val;
        buffer += sizeof(T);
    }

    template<typename T>
    void read(const char*& buffer, T& val)
    {
        val = *reinterpret_cast<const T*>(buffer);
        buffer += sizeof(T);
    }
}

namespace nvinfer1
{
    ArgMaxPlugin::ArgMaxPlugin(int netChannels, int netHeight, int netWidth, int axis)
    {
        mChannels = netChannels;
        mWidth = netWidth;
        mHeight = netHeight;
        mAxis = axis;
    }

    ArgMaxPlugin::~ArgMaxPlugin() {}

    // create the plugin at runtime from a byte stream
    ArgMaxPlugin::ArgMaxPlugin(const void* data, size_t length)
    {
        using namespace Tn;
        const char *d = reinterpret_cast<const char *>(data), *a = d;
        read(d, mChannels);
        read(d, mWidth);
        read(d, mHeight);
        read(d, mAxis);
        read(d, mThreadCount);
        assert(d == a + length);
    }

    void ArgMaxPlugin::serialize(void* buffer) const TRT_NOEXCEPT
    {
        using namespace Tn;
        char* d = static_cast<char*>(buffer), *a = d;
        write(d, mChannels);
        write(d, mWidth);
        write(d, mHeight);
        write(d, mAxis);
        write(d, mThreadCount);
        assert(d == a + getSerializationSize());
    }

    size_t ArgMaxPlugin::getSerializationSize() const TRT_NOEXCEPT
    {
        return sizeof(mChannels) + sizeof(mWidth)  + sizeof(mHeight)  + sizeof(mAxis)  + sizeof(mThreadCount);
    }

    int ArgMaxPlugin::initialize() TRT_NOEXCEPT
    {
        return 0;
    }

    Dims ArgMaxPlugin::getOutputDimensions(int index, const Dims* inputs, int nbInputDims) TRT_NOEXCEPT
    {
        Dims3 outputDims = {mChannels, mHeight, mWidth};
        outputDims.d[mAxis] = 1;
        return outputDims;
    }

    // Set plugin namespace
    void ArgMaxPlugin::setPluginNamespace(const char* pluginNamespace) TRT_NOEXCEPT
    {
        mPluginNamespace = pluginNamespace;
    }

    const char* ArgMaxPlugin::getPluginNamespace() const TRT_NOEXCEPT
    {
        return mPluginNamespace;
    }

    // Return the DataType of the plugin output at the requested index
    DataType ArgMaxPlugin::getOutputDataType(int index, const nvinfer1::DataType* inputTypes, int nbInputs) const TRT_NOEXCEPT
    {
        return DataType::kFLOAT;
    }

    // Return true if output tensor is broadcast across a batch.
    bool ArgMaxPlugin::isOutputBroadcastAcrossBatch(int outputIndex, const bool* inputIsBroadcasted, int nbInputs) const TRT_NOEXCEPT
    {
        return false;
    }

    // Return true if plugin can use input that is broadcast across batch without replication.
    bool ArgMaxPlugin::canBroadcastInputAcrossBatch(int inputIndex) const TRT_NOEXCEPT
    {
        return false;
    }

    // Attach the plugin object to an execution context and grant the plugin the access to some context resource.
    void ArgMaxPlugin::attachToContext(cudnnContext* cudnnContext, cublasContext* cublasContext, IGpuAllocator* gpuAllocator) TRT_NOEXCEPT
    {
    }

    // Detach the plugin object from its execution context.
    void ArgMaxPlugin::detachFromContext() TRT_NOEXCEPT {}

    const char* ArgMaxPlugin::getPluginType() const TRT_NOEXCEPT
    {
        return "ArgMax_TRT";
    }

    const char* ArgMaxPlugin::getPluginVersion() const TRT_NOEXCEPT
    {
        return "1";
    }

    void ArgMaxPlugin::destroy() TRT_NOEXCEPT
    {
        delete this;
    }

    // Clone the plugin
    IPluginV2IOExt* ArgMaxPlugin::clone() const TRT_NOEXCEPT
    {
        ArgMaxPlugin* p = new ArgMaxPlugin(mChannels, mHeight, mWidth, mAxis);
        p->setPluginNamespace(mPluginNamespace);
        return p;
    }

    __global__ void argMaxCuda(const float *input, float* output, int axis, int C, int W, int H)
    {
        int maxIndex = -1;
        float maxValue = -INFINITY;
        unsigned int tid = threadIdx.x + blockIdx.x * blockDim.x;
        unsigned int b, c, h, w, remaining, outputIndex, index;
        if(axis == 1)
        {
            b = tid / (C * W);
            remaining = tid % (C * W);
            c = remaining / W;
            w = remaining % W;
            outputIndex = b * C * W + c * W + w;
            for (int w = 0; w < W; c++)
            {

                index = b * C * W * H + c * H * W + h * W + w;
                float value = input[index];

                if (value > maxValue)
                {
                    maxValue = value;
                    maxIndex = w;
                }
            }
        }
        else if(axis == 2)
        {
            b = tid / (C * H);
            remaining = tid % (C * H);
            c = remaining / H;
            h = remaining % H;
            outputIndex = b * C * H + c * H + h;
            for (int w = 0; w < W; c++)
            {

                index = b * C * W * H + c * H * W + h * W + w;
                float value = input[index];

                if (value > maxValue)
                {
                    maxValue = value;
                    maxIndex = w;
                }
            }
        }
        else
        {
            b = tid / (W * H);
            remaining = tid % (W * H);
            h = remaining / W;
            w = remaining % W;
            outputIndex = b * H * W + h * W + w;
            for (int c = 0; c < C; c++)
            {

                index = b * C * W * H + c * H * W + h * W + w;
                float value = input[index];

                if (value > maxValue)
                {
                    maxValue = value;
                    maxIndex = c;
                }
            }
        }
        output[outputIndex] = maxIndex;

    }

    void ArgMaxPlugin::forwardGpu(const float* inputs, float *output, cudaStream_t stream, int batchSize) {
        int outputElem;
        int block_size;
        if(mAxis == 0)
        {
            outputElem = mWidth * mHeight;
            block_size = (outputElem * batchSize + mThreadCount - 1) / mThreadCount;
        }
        else if(mAxis == 1)
        {
            outputElem = mWidth * mChannels;
            block_size = (outputElem * batchSize + mThreadCount - 1) / mThreadCount;
        }
        else
        {
            outputElem = mChannels * mHeight;
            block_size = (outputElem * batchSize + mThreadCount - 1) / mThreadCount;
        }
        for (int idx = 0; idx < batchSize; ++idx) {
            CUDA_CHECK(cudaMemsetAsync(output + idx * outputElem, 0, sizeof(float), stream));
        }
        argMaxCuda<<<block_size, mThreadCount, 0, stream>>>(inputs, output, mAxis, mChannels, mWidth, mHeight);


    }

    int ArgMaxPlugin::enqueue(int batchSize, const void* const* inputs, void* TRT_CONST_ENQUEUE* outputs, void* workspace, cudaStream_t stream) TRT_NOEXCEPT
    {
        const float* inputData = static_cast<const float*>(inputs[0]);
        float* outputData = static_cast<float*>(outputs[0]);
        int outputElem;
        int block_size;
        if(mAxis == 0)
        {
            outputElem = mWidth * mHeight;
            block_size = (outputElem * batchSize + mThreadCount - 1) / mThreadCount;
        }
        else if(mAxis == 1)
        {
            outputElem = mWidth * mChannels;
            block_size = (outputElem * batchSize + mThreadCount - 1) / mThreadCount;
        }
        else
        {
            outputElem = mChannels * mHeight;
            block_size = (outputElem * batchSize + mThreadCount - 1) / mThreadCount;
        }
        for (int idx = 0; idx < batchSize; ++idx) {
            CUDA_CHECK(cudaMemsetAsync(outputData + idx * outputElem, 0, sizeof(float), stream));
        }
        argMaxCuda<<<block_size, mThreadCount, 0, stream>>>(inputData, outputData, mAxis, mChannels, mWidth, mHeight);
        cudaDeviceSynchronize();
        return 0;
    }

    PluginFieldCollection ArgMaxPluginCreator::mFC{};
    std::vector<PluginField> ArgMaxPluginCreator::mPluginAttributes;

    ArgMaxPluginCreator::ArgMaxPluginCreator()
    {
        mPluginAttributes.clear();
        mFC.nbFields = mPluginAttributes.size();
        mFC.fields = mPluginAttributes.data();
    }

    const char* ArgMaxPluginCreator::getPluginName() const TRT_NOEXCEPT
    {
        return "ArgMax_TRT";
    }

    const char* ArgMaxPluginCreator::getPluginVersion() const TRT_NOEXCEPT
    {
        return "1";
    }

    const PluginFieldCollection* ArgMaxPluginCreator::getFieldNames() TRT_NOEXCEPT
    {
        return &mFC;
    }

    IPluginV2IOExt* ArgMaxPluginCreator::createPlugin(const char* name, const PluginFieldCollection* fc) TRT_NOEXCEPT
    {
        assert(fc->nbFields == 1);
        assert(strcmp(fc->fields[0].name, "netinfo") == 0);
        int *p_netinfo = (int*)(fc->fields[0].data);

        ArgMaxPlugin* obj = new ArgMaxPlugin(p_netinfo[0], p_netinfo[1], p_netinfo[2], p_netinfo[3]);
        obj->setPluginNamespace(mNamespace.c_str());
        return obj;
    }

    IPluginV2IOExt* ArgMaxPluginCreator::deserializePlugin(const char* name, const void* serialData, size_t serialLength) TRT_NOEXCEPT
    {
        // This object will be deleted when the network is destroyed, which will
        // call YoloLayerPlugin::destroy()
        ArgMaxPlugin* obj = new ArgMaxPlugin(serialData, serialLength);
        obj->setPluginNamespace(mNamespace.c_str());
        return obj;
    }
}

