#include "model.h"
#include "block.h"
#include "config.h"
#include "calibrator.h"
#include <iostream>
#include <cassert>
#include "argmax.h"
using namespace nvinfer1;


IHostMemory* build_ddrnet(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, const std::string& wts_path, const int planes, const int spp_planes, const int head_planes)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };
    const int highres_planes = planes * 2;
    std::map<std::string, Weights> weightMap = loadWeights(wts_path);

    INetworkDefinition* network = builder->createNetworkV2(0U);
    ITensor* data = network->addInput(kInputTensorName, dt, Dims3{ 3, kInputH, kInputW });

    IActivationLayer* conv1_0 = convBnRelu(network, weightMap, *data, planes, 3, 2, 1, "conv1.0");
    IActivationLayer* conv1_1 = convBnRelu(network, weightMap, *conv1_0->getOutput(0), planes, 3, 2, 1, "conv1.1");

    ITensor* layer1_0 = BasicBLock(network, weightMap, *conv1_1->getOutput(0), planes, planes, 1, true, "layer1.0");
    ITensor* layer1_1 = BasicBLock(network, weightMap, *layer1_0, planes, planes, 1, false, "layer1.1");
    IActivationLayer *reluLayer1_1 = network->addActivation(*layer1_1, ActivationType::kRELU);

    ITensor* layer2_0 = BasicBLock(network, weightMap, *reluLayer1_1->getOutput(0), planes, planes * 2, 2, true, "layer2.0");
    ITensor* layer2_1 = BasicBLock(network, weightMap, *layer2_0, planes * 2, planes * 2, 1, false, "layer2.1");
    IActivationLayer *reluLayer2_1 = network->addActivation(*layer2_1, ActivationType::kRELU);

    ITensor* layer3_0 = BasicBLock(network, weightMap, *reluLayer2_1->getOutput(0), planes * 2, planes * 4, 2, true,"layer3.0");
    ITensor* layer3_1 = BasicBLock(network, weightMap, *layer3_0, planes * 4, planes * 4, 1, false, "layer3.1");
    IActivationLayer *reluLayer3_1 = network->addActivation(*layer3_1, ActivationType::kRELU);

    ITensor* layer_3_0 = BasicBLock(network, weightMap, *reluLayer2_1->getOutput(0), planes * 2, highres_planes, 1, true, "layer3_.0");
    ITensor* layer_3_1 = BasicBLock(network, weightMap, *layer_3_0, highres_planes, highres_planes, 1, false, "layer3_.1");
    IActivationLayer *reluLayer_3_1 = network->addActivation(*layer_3_1, ActivationType::kRELU);


    IScaleLayer* down3= ConvBN(network, weightMap, *reluLayer_3_1->getOutput(0), planes * 4, 3, 2, -1,"down3._conv.weight" , "down3._batch_norm");

    IElementWiseLayer* ew3_0 = network->addElementWise(*layer3_1, *down3->getOutput(0), ElementWiseOperation::kSUM);

    IScaleLayer* compression3= ConvBN(network, weightMap, *reluLayer3_1->getOutput(0), highres_planes, 1, 1, 0, "compression3._conv.weight" , "compression3._batch_norm");
    IResizeLayer *rl3 = network->addResize(*compression3->getOutput(0));
    rl3->setResizeMode(InterpolationMode::kLINEAR);
    rl3->setOutputDimensions(Dims3{highres_planes, kInputH / 8, kInputW / 8});
    IElementWiseLayer* ew3_1 = network->addElementWise(*layer_3_1, *rl3->getOutput(0), ElementWiseOperation::kSUM);
    IActivationLayer* relu3_out = network->addActivation(*ew3_0->getOutput(0), ActivationType::kRELU);
    IActivationLayer* relu3_merge = network->addActivation(*ew3_1->getOutput(0), ActivationType::kRELU);

    ITensor* layer4_0 = BasicBLock(network, weightMap, *relu3_out->getOutput(0), planes * 4, planes * 8, 2, true, "layer4.0");
    ITensor* layer4_1 = BasicBLock(network, weightMap, *layer4_0, planes * 8, planes * 8, 1, false, "layer4.1");
    IActivationLayer *reluLayer4_1 = network->addActivation(*layer4_1, ActivationType::kRELU);

    ITensor* layer_4_0 = BasicBLock(network, weightMap, *relu3_merge->getOutput(0), highres_planes, highres_planes, 1, true, "layer4_.0");
    ITensor* layer_4_1 = BasicBLock(network, weightMap, *layer_4_0, highres_planes, highres_planes, 1, false, "layer4_.1");
    IActivationLayer *reluLayer_4_1 = network->addActivation(*layer_4_1, ActivationType::kRELU);


    IScaleLayer* compression4= ConvBN(network, weightMap, *reluLayer4_1->getOutput(0), highres_planes, 1, 1, 0, "compression4._conv.weight", "compression4._batch_norm");
    IActivationLayer* down4_0= convBnRelu(network, weightMap, *reluLayer_4_1->getOutput(0), planes * 4, 3, 2, 1, "down4.0" );
    IScaleLayer* down4_1= ConvBN(network, weightMap, *down4_0->getOutput(0), planes * 8, 3, 2, 1, "down4.1._conv.weight" , "down4.1._batch_norm");

    IElementWiseLayer* ew4_0 = network->addElementWise(*layer4_1, *down4_1->getOutput(0), ElementWiseOperation::kSUM);
    IResizeLayer *rl4 = network->addResize(*compression4->getOutput(0));
    rl4->setResizeMode(InterpolationMode::kLINEAR);
    rl4->setOutputDimensions(Dims3{highres_planes, kInputH / 8, kInputW / 8});
    IElementWiseLayer* ew4_1 = network->addElementWise(*layer_4_1, *rl4->getOutput(0), ElementWiseOperation::kSUM);;

    IActivationLayer* relu5 = network->addActivation(*ew4_0->getOutput(0), ActivationType::kRELU);
    IActivationLayer* relu_5 = network->addActivation(*ew4_1->getOutput(0), ActivationType::kRELU);
    ITensor* layer5_0 = Bottleneck(network, weightMap, *relu5->getOutput(0), planes * 8, planes * 8, 2, false, "layer5.0");
    ITensor* layer_5_0 = Bottleneck(network, weightMap, *relu_5->getOutput(0), highres_planes, highres_planes, 1, false, "layer5_.0");

    IElementWiseLayer* spp = Dappm(network, weightMap, *layer5_0, spp_planes, planes * 4, "spp");
    IResizeLayer *rl5 = network->addResize(*spp->getOutput(0));
    rl5->setResizeMode(InterpolationMode::kLINEAR);
    rl5->setOutputDimensions(Dims3{planes * 4, kInputH / 8, kInputW / 8});

    IElementWiseLayer* merge_5 = network->addElementWise(*rl5->getOutput(0), *layer_5_0, ElementWiseOperation::kSUM);
    IConvolutionLayer* output_ddrnet = ddrnet_head(network, weightMap, *merge_5->getOutput(0), head_planes, kNumClass, "head");
    IResizeLayer* resize_output = resizeLayer(network, *output_ddrnet->getOutput(0), nvinfer1::Dims3{kNumClass, kInputH, kInputW}, "resize");

    IPluginV2Layer* output = addArgMax(network, *resize_output->getOutput(0), 0,"argmax");
    output->getOutput(0)->setName(kOutputTensorName);
    network->markOutput(*output->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB


#if defined(USE_FP16)
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(nvinfer1::BuilderFlag::kINT8);
    nvinfer1::IInt8EntropyCalibrator2* calibrator = new Calibrator(1, kInputW, kInputH, "../calibrator/", "int8calib.table", kInputTensorName);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    IHostMemory* serialized_model = builder->buildSerializedNetwork(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    delete network;

    // Release host memory
    for (auto& mem : weightMap) {
        free((void*)(mem.second.values));
    }

    return serialized_model;

}