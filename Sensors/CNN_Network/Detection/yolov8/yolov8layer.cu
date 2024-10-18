//
// From https://github.com/wang-xinyu/tensorrtx
//
#include "yolov8layer.h"
#include <assert.h>
#include <math.h>
#include <vector>
#include <iostream>
#include "../../utils/cuda_utils.h"

namespace Tn {
    template<typename T>
    void write(char*& buffer, const T& val) {
        *reinterpret_cast<T*>(buffer) = val;
        buffer += sizeof(T);
    }

    template<typename T>
    void read(const char*& buffer, T& val) {
        val = *reinterpret_cast<const T*>(buffer);
        buffer += sizeof(T);
    }
}  // namespace Tn


namespace nvinfer1 {
    Yolov8LayerPlugin::Yolov8LayerPlugin(int classCount, int netWidth, int netHeight, int maxOut) {
    mClassCount = classCount;
    mYoloV8NetWidth = netWidth;
    mYoloV8netHeight = netHeight;
    mMaxOutObject = maxOut;
}

Yolov8LayerPlugin::~Yolov8LayerPlugin() {}

Yolov8LayerPlugin::Yolov8LayerPlugin(const void* data, size_t length) {
    using namespace Tn;
    const char* d = reinterpret_cast<const char*>(data), * a = d;
    read(d, mClassCount);
    read(d, mThreadCount);
    read(d, mYoloV8NetWidth);
    read(d, mYoloV8netHeight);
    read(d, mMaxOutObject);

    assert(d == a + length);
}

void Yolov8LayerPlugin::serialize(void* buffer) const TRT_NOEXCEPT {

    using namespace Tn;
    char* d = static_cast<char*>(buffer), * a = d;
    write(d, mClassCount);
    write(d, mThreadCount);
    write(d, mYoloV8NetWidth);
    write(d, mYoloV8netHeight);
    write(d, mMaxOutObject);

    assert(d == a + getSerializationSize());
}

size_t Yolov8LayerPlugin::getSerializationSize() const TRT_NOEXCEPT {
    return sizeof(mClassCount) + sizeof(mThreadCount) + sizeof(mYoloV8netHeight) + sizeof(mYoloV8NetWidth) + sizeof(mMaxOutObject);
}

int Yolov8LayerPlugin::initialize() TRT_NOEXCEPT {
    return 0;
}

nvinfer1::Dims Yolov8LayerPlugin::getOutputDimensions(int index, const nvinfer1::Dims* inputs, int nbInputDims) TRT_NOEXCEPT {
    int total_size = mMaxOutObject * sizeof(Yolo::Detection) / sizeof(float);
    return nvinfer1::Dims3(total_size + 1, 1, 1);
}

void Yolov8LayerPlugin::setPluginNamespace(const char* pluginNamespace) TRT_NOEXCEPT {
    mPluginNamespace = pluginNamespace;
}

const char* Yolov8LayerPlugin::getPluginNamespace() const TRT_NOEXCEPT {
    return mPluginNamespace;
}

nvinfer1::DataType Yolov8LayerPlugin::getOutputDataType(int index, const nvinfer1::DataType* inputTypes, int nbInputs) const TRT_NOEXCEPT {
    return nvinfer1::DataType::kFLOAT;
}

bool Yolov8LayerPlugin::isOutputBroadcastAcrossBatch(int outputIndex, const bool* inputIsBroadcasted, int nbInputs) const TRT_NOEXCEPT {

    return false;
}

bool Yolov8LayerPlugin::canBroadcastInputAcrossBatch(int inputIndex) const TRT_NOEXCEPT {

    return false;
}

void Yolov8LayerPlugin::configurePlugin(nvinfer1::PluginTensorDesc const* in, int nbInput, nvinfer1::PluginTensorDesc const* out, int nbOutput) TRT_NOEXCEPT {};

void Yolov8LayerPlugin::attachToContext(cudnnContext* cudnnContext, cublasContext* cublasContext, IGpuAllocator* gpuAllocator) TRT_NOEXCEPT {};

void Yolov8LayerPlugin::detachFromContext() TRT_NOEXCEPT {}

const char* Yolov8LayerPlugin::getPluginType() const TRT_NOEXCEPT {

    return "Yolov8Layer_TRT";
}

const char* Yolov8LayerPlugin::getPluginVersion() const TRT_NOEXCEPT {
    return "1";
}

void Yolov8LayerPlugin::destroy() TRT_NOEXCEPT {

    delete this;
}

nvinfer1::IPluginV2IOExt* Yolov8LayerPlugin::clone() const TRT_NOEXCEPT {

    Yolov8LayerPlugin* p = new Yolov8LayerPlugin(mClassCount, mYoloV8NetWidth, mYoloV8netHeight, mMaxOutObject);
    p->setPluginNamespace(mPluginNamespace);
    return p;
}

int Yolov8LayerPlugin::enqueue(int batchSize, const void* TRT_CONST_ENQUEUE* inputs, void* const* outputs, void* workspace, cudaStream_t stream) TRT_NOEXCEPT {

    forwardGpu((const float* const*)inputs, (float*)outputs[0], stream, mYoloV8netHeight, mYoloV8NetWidth, batchSize);
    return 0;
}


__device__ float LogistV8(float data) { return 1.0f / (1.0f + expf(-data)); };

__global__ void CalDetectionV8(const float* input, float* output, int numElements, int maxoutobject,
                             const int grid_h, int grid_w, const int stride, int classes, int outputElem) {
    int idx = threadIdx.x + blockDim.x * blockIdx.x;
    if (idx >= numElements) return;

    int total_grid = grid_h * grid_w;
    int info_len = 4 + classes;
    int batchIdx = idx / total_grid;
    int elemIdx = idx % total_grid;
    const float* curInput = input + batchIdx * total_grid * info_len;
    int outputIdx = batchIdx * outputElem;

    int class_id = 0;
    float max_cls_prob = 0.0;
    for (int i = 4; i < info_len; i++) {
        float p = LogistV8(curInput[elemIdx + i * total_grid]);
        if (p > max_cls_prob) {
            max_cls_prob = p;
            class_id = i - 4;
        }
    }

    if (max_cls_prob < 0.1) return;

    int count = (int)atomicAdd(output + outputIdx, 1);
    if (count >= maxoutobject) return;
    char* data = (char*)(output + outputIdx) + sizeof(float) + count * sizeof(Yolo::Detection);
    Yolo::Detection* det = (Yolo::Detection*)(data);

    int row = elemIdx / grid_w;
    int col = elemIdx % grid_w;

    det->conf = max_cls_prob;
    det->class_id = class_id;
    det->bbox[0] = (col + 0.5f - curInput[elemIdx + 0 * total_grid]) * stride;
    det->bbox[1] = (row + 0.5f - curInput[elemIdx + 1 * total_grid]) * stride;
    det->bbox[2] = (col + 0.5f + curInput[elemIdx + 2 * total_grid]) * stride;
    det->bbox[3] = (row + 0.5f + curInput[elemIdx + 3 * total_grid]) * stride;
}

void Yolov8LayerPlugin::forwardGpu(const float* const* inputs, float* output, cudaStream_t stream, int mYoloV8netHeight,int mYoloV8NetWidth, int batchSize) {
    int outputElem = 1 + mMaxOutObject * sizeof(Yolo::Detection) / sizeof(float);
    cudaMemsetAsync(output, 0, sizeof(float), stream);
    for (int idx = 0; idx < batchSize; ++idx) {
        CUDA_CHECK(cudaMemsetAsync(output + idx * outputElem, 0, sizeof(float), stream));
    }
    int numElem = 0;
    int grids[3][2] = { {mYoloV8netHeight / 8, mYoloV8NetWidth / 8}, {mYoloV8netHeight / 16, mYoloV8NetWidth / 16}, {mYoloV8netHeight / 32, mYoloV8NetWidth / 32} };
    int strides[] = { 8, 16, 32 };
    for (unsigned int i = 0; i < 3; i++) {
        int grid_h = grids[i][0];
        int grid_w = grids[i][1];
        int stride = strides[i];
        numElem = grid_h * grid_w * batchSize;
        if (numElem < mThreadCount) mThreadCount = numElem;

        CalDetectionV8 <<<(numElem + mThreadCount - 1) / mThreadCount, mThreadCount, 0, stream >>>
            (inputs[i], output, numElem, mMaxOutObject, grid_h, grid_w, stride, mClassCount, outputElem);
    }
}

PluginFieldCollection YoloV8PluginCreator::mFC{};
std::vector<PluginField> YoloV8PluginCreator::mPluginAttributes;

YoloV8PluginCreator::YoloV8PluginCreator() {
    mPluginAttributes.clear();
    mFC.nbFields = mPluginAttributes.size();
    mFC.fields = mPluginAttributes.data();
}

const char* YoloV8PluginCreator::getPluginName() const TRT_NOEXCEPT {
    return "Yolov8Layer_TRT";
}

const char* YoloV8PluginCreator::getPluginVersion() const TRT_NOEXCEPT {
    return "1";
}

const PluginFieldCollection* YoloV8PluginCreator::getFieldNames() TRT_NOEXCEPT {
    return &mFC;
}

IPluginV2IOExt* YoloV8PluginCreator::createPlugin(const char* name, const PluginFieldCollection* fc) TRT_NOEXCEPT {
    assert(fc->nbFields == 1);
    assert(strcmp(fc->fields[0].name, "netinfo") == 0);
    int* p_netinfo = (int*)(fc->fields[0].data);
    int class_count = p_netinfo[0];
    int input_w = p_netinfo[1];
    int input_h = p_netinfo[2];
    int max_output_object_count = p_netinfo[3];
    Yolov8LayerPlugin* obj = new Yolov8LayerPlugin(class_count, input_w, input_h, max_output_object_count);
    obj->setPluginNamespace(mNamespace.c_str());
    return obj;
}

IPluginV2IOExt* YoloV8PluginCreator::deserializePlugin(const char* name, const void* serialData, size_t serialLength) TRT_NOEXCEPT {
    // This object will be deleted when the network is destroyed, which will
    // call YoloV8PluginCreator::destroy()
    Yolov8LayerPlugin* obj = new Yolov8LayerPlugin(serialData, serialLength);
    obj->setPluginNamespace(mNamespace.c_str());
    return obj;
}

} // namespace nvinfer1
