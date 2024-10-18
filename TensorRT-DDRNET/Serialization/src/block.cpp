#include "block.h"
#include "NvInfer.h"
#include <iostream>
#include <fstream>
#include <assert.h>
#include <cmath>
#include <cstring>

using namespace nvinfer1;

// TensorRT weight files have a simple space delimited format:
// [type] [size] <data x size in hex>
std::map<std::string, Weights> loadWeights(const std::string file) {
    std::cout << "Loading weights: " << file << std::endl;
    std::map<std::string, Weights> weightMap;

    // Open weights file
    std::ifstream input(file);
    assert(input.is_open() && "Unable to load weight file. please check if the .wts file path is right!!!!!!");

    // Read number of weight blobs
    int32_t count;
    input >> count;
    assert(count > 0 && "Invalid weight map file.");

    while (count--)
    {
        Weights wt{ DataType::kFLOAT, nullptr, 0 };
        uint32_t size;

        // Read name and type of blob
        std::string name;
        input >> name >> std::dec >> size;
        wt.type = DataType::kFLOAT;

        // Load blob
        uint32_t* val = reinterpret_cast<uint32_t*>(malloc(sizeof(val) * size));
        for (uint32_t x = 0, y = size; x < y; ++x)
        {
            input >> std::hex >> val[x];
        }
        wt.values = val;

        wt.count = size;
        weightMap[name] = wt;
    }

    return weightMap;
}

IScaleLayer* addBatchNorm2d(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, std::string lname, float eps) {
    float* gamma = (float*)weightMap[lname + ".weight"].values;
    float* beta = (float*)weightMap[lname + ".bias"].values;
    float* mean = (float*)weightMap[lname + "._mean"].values;
    float* var = (float*)weightMap[lname + "._variance"].values;
    int len = weightMap[lname + "._variance"].count;
    float* scval = reinterpret_cast<float*>(malloc(sizeof(float) * len));
    for (int i = 0; i < len; i++) {
        scval[i] = gamma[i] / sqrt(var[i] + eps);
    }
    Weights scale{ DataType::kFLOAT, scval, len };

    float* shval = reinterpret_cast<float*>(malloc(sizeof(float) * len));
    for (int i = 0; i < len; i++) {
        shval[i] = beta[i] - mean[i] * gamma[i] / sqrt(var[i] + eps);
    }
    Weights shift{ DataType::kFLOAT, shval, len };

    float* pval = reinterpret_cast<float*>(malloc(sizeof(float) * len));
    for (int i = 0; i < len; i++) {
        pval[i] = 1.0;
    }
    Weights power{ DataType::kFLOAT, pval, len };

    weightMap[lname + ".scale"] = scale;
    weightMap[lname + ".shift"] = shift;
    weightMap[lname + ".power"] = power;
    IScaleLayer* scale_1 = network->addScale(input, ScaleMode::kCHANNEL, shift, scale, power);
    assert(scale_1);
    return scale_1;
}

IElementWiseLayer* convBnSilu(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int c2, int k, int s, int p, std::string lname) {
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    IConvolutionLayer* conv1 = network->addConvolutionNd(input, c2, DimsHW{ k, k }, weightMap[lname + ".conv.weight"], emptywts);
    assert(conv1);
    conv1->setName((lname + ".conv").c_str());
    conv1->setStrideNd(DimsHW{ s, s });
    conv1->setPaddingNd(DimsHW{ p, p });


    IScaleLayer* bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), lname + ".bn", 1e-5);


    // silu = x * sigmoid(x)
    IActivationLayer* sig1 = network->addActivation(*bn1->getOutput(0), ActivationType::kSIGMOID);
    assert(sig1);
    IElementWiseLayer* ew1 = network->addElementWise(*bn1->getOutput(0), *sig1->getOutput(0), ElementWiseOperation::kPROD);
    assert(ew1);
    return ew1;
}


ILayer* DownC(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int c1, int c2, const std::string& lname) {
    int c_ = int(c2 * 0.5);
    IElementWiseLayer* cv1 = convBnSilu(network, weightMap, input, c1, 1, 1, 0, lname + ".cv1");
    IElementWiseLayer* cv2 = convBnSilu(network, weightMap, *cv1->getOutput(0), c_, 3, 2, 1, lname + ".cv2");

    IPoolingLayer* m1 = network->addPoolingNd(input, PoolingType::kMAX, DimsHW{ 2, 2 });
    m1->setStrideNd(DimsHW{ 2, 2 });
    IElementWiseLayer* cv3 = convBnSilu(network, weightMap, *m1->getOutput(0), c_, 1, 1, 0, lname + ".cv3");

    ITensor* input_tensors[] = { cv2->getOutput(0),  cv3->getOutput(0) };
    IConcatenationLayer* concat = network->addConcatenation(input_tensors, 2);

    return concat;

}

IElementWiseLayer* SPPCSPC(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int c2, const std::string& lname) {
    int c_ = int(2 * c2 * 0.5);
    IElementWiseLayer* cv1 = convBnSilu(network, weightMap, input, c_, 1, 1, 0, lname + ".cv1");
    IElementWiseLayer* cv2 = convBnSilu(network, weightMap, input, c_, 1, 1, 0, lname + ".cv2");

    IElementWiseLayer* cv3 = convBnSilu(network, weightMap, *cv1->getOutput(0), c_, 3, 1, 1, lname + ".cv3");
    IElementWiseLayer* cv4 = convBnSilu(network, weightMap, *cv3->getOutput(0), c_, 1, 1, 0, lname + ".cv4");

    IPoolingLayer* m1 = network->addPoolingNd(*cv4->getOutput(0), PoolingType::kMAX, DimsHW{ 5, 5 });
    m1->setStrideNd(DimsHW{ 1, 1 });
    m1->setPaddingNd(DimsHW{ 2, 2 });
    IPoolingLayer* m2 = network->addPoolingNd(*cv4->getOutput(0), PoolingType::kMAX, DimsHW{ 9, 9 });
    m2->setStrideNd(DimsHW{ 1, 1 });
    m2->setPaddingNd(DimsHW{ 4, 4 });
    IPoolingLayer* m3 = network->addPoolingNd(*cv4->getOutput(0), PoolingType::kMAX, DimsHW{ 13, 13 });
    m3->setStrideNd(DimsHW{ 1, 1 });
    m3->setPaddingNd(DimsHW{ 6, 6 });

    ITensor* input_tensors[] = { cv4->getOutput(0), m1->getOutput(0), m2->getOutput(0), m3->getOutput(0) };
    IConcatenationLayer* concat = network->addConcatenation(input_tensors, 4);
    // 0U
    concat->setAxis(0);

    IElementWiseLayer* cv5 = convBnSilu(network, weightMap, *concat->getOutput(0), c_, 1, 1, 0, lname + ".cv5");
    IElementWiseLayer* cv6 = convBnSilu(network, weightMap, *cv5->getOutput(0), c_, 3, 1, 1, lname + ".cv6");

    ITensor* input_tensors2[] = { cv6->getOutput(0), cv2->getOutput(0) };
    IConcatenationLayer* concat1 = network->addConcatenation(input_tensors2, 2);
    // 0U
    concat1->setAxis(0);


    IElementWiseLayer* cv7 = convBnSilu(network, weightMap, *concat1->getOutput(0), c2, 1, 1, 0, lname + ".cv7");
    return cv7;
}

IElementWiseLayer* RepConv(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int c2, int k, int s, const std::string& lname) {
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };
    // 256 * 128 * 3 *3
    IConvolutionLayer* rbr_dense_conv = network->addConvolutionNd(input, c2, DimsHW{ k, k }, weightMap[lname + ".rbr_dense.0.weight"], emptywts);
    assert(rbr_dense_conv);
    rbr_dense_conv->setPaddingNd(DimsHW{ k / 2, k / 2 });
    rbr_dense_conv->setStrideNd(DimsHW{ s, s });
    rbr_dense_conv->setName((lname + ".rbr_dense.0").c_str());
    IScaleLayer* rbr_dense_bn = addBatchNorm2d(network, weightMap, *rbr_dense_conv->getOutput(0), lname + ".rbr_dense.1", 1e-3);

    IConvolutionLayer* rbr_1x1_conv = network->addConvolutionNd(input, c2, DimsHW{ 1, 1 }, weightMap[lname + ".rbr_1x1.0.weight"], emptywts);
    assert(rbr_1x1_conv);
    rbr_1x1_conv->setStrideNd(DimsHW{ s, s });
    rbr_1x1_conv->setName((lname + ".rbr_1x1.0").c_str());
    IScaleLayer* rbr_1x1_bn = addBatchNorm2d(network, weightMap, *rbr_1x1_conv->getOutput(0), lname + ".rbr_1x1.1", 1e-3);

    IElementWiseLayer* ew1 = network->addElementWise(*rbr_dense_bn->getOutput(0), *rbr_1x1_bn->getOutput(0), ElementWiseOperation::kSUM);
    assert(ew1);
    // silu
    IActivationLayer* sigmoid = network->addActivation(*ew1->getOutput(0), ActivationType::kSIGMOID);
    IElementWiseLayer* ew2 = network->addElementWise(*ew1->getOutput(0), *sigmoid->getOutput(0), ElementWiseOperation::kPROD);
    return ew2;
}

IActivationLayer* convBlockLeakRelu(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int outch, int ksize, int s, int p, std::string lname) {
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    IConvolutionLayer* conv1 = network->addConvolutionNd(input, outch, DimsHW{ ksize, ksize }, weightMap[lname + "._conv.weight"], emptywts);
    assert(conv1);
    conv1->setName((lname + ".conv").c_str());
    conv1->setStrideNd(DimsHW{ s, s });
    conv1->setPaddingNd(DimsHW{ p, p });
    //conv1->setNbGroups(g);
    IScaleLayer* bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), lname + "._batch_norm", 1e-5);

    auto ew1 = network->addActivation(*bn1->getOutput(0), ActivationType::kLEAKY_RELU);
    ew1->setAlpha(0.1);
    return ew1;
}



IResizeLayer* resizeLayer(INetworkDefinition* network, ITensor& input, Dims3 dims,  std::string lname)
{

    IResizeLayer* resize = network->addResize(input);
    resize->setResizeMode(InterpolationMode::kLINEAR);
    resize->setOutputDimensions(dims);
    resize->setName((lname + ".resize").c_str());
    return resize;
}
IActivationLayer* convBnRelu(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input,int c2, int k, int s, int p, std::string lname, bool bias)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    IConvolutionLayer* conv1 = network->addConvolutionNd(input, c2, DimsHW{ k, k }, weightMap[lname + "._conv.weight"], emptywts);
    assert(conv1);
    conv1->setName((lname + ".conv").c_str());
    conv1->setStrideNd(DimsHW{ s, s });
    conv1->setPaddingNd(DimsHW{ p, p });


    IScaleLayer* bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), lname + "._batch_norm", 1e-5);

    IActivationLayer* relu1 = network->addActivation(*bn1->getOutput(0), ActivationType::kRELU);
    assert(relu1);
    return relu1;
}


IScaleLayer* ConvBN(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int outch, int k, int s, int p, std::string lnameConv, std::string lnameBatch)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    IConvolutionLayer* conv1 = network->addConvolutionNd(input, outch, DimsHW{ k, k }, weightMap[lnameConv], emptywts);
    assert(conv1);
    conv1->setName((lnameConv).c_str());
    conv1->setStrideNd(DimsHW{ s, s });
    if(p >= 0)
    {
        conv1->setPaddingNd(DimsHW{ p, p });
    }
    else
    {
        conv1->setPaddingMode(PaddingMode::kSAME_LOWER);
        conv1->setPaddingMode(PaddingMode::kSAME_UPPER);
    }
    IScaleLayer* bn1 = addBatchNorm2d(network, weightMap, *conv1->getOutput(0), lnameBatch, 1e-5);
    return bn1;
}


ITensor* BasicBLock(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int inch, int outch, int s, bool relu, std::string lname)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };
    IActivationLayer* convBRelu = convBnRelu(network, weightMap, input, outch, 3, s, 1, lname + ".conv_bn_relu");
    IScaleLayer* convB = ConvBN(network, weightMap, *convBRelu->getOutput(0), outch, 3, 1, 1,  lname + ".conv_bn._conv.weight", lname + ".conv_bn._batch_norm");
    IElementWiseLayer *ew1;
    if (inch != outch || s != 1) {
        IScaleLayer* convBd = ConvBN(network, weightMap, input,  outch, 1, s, 0, lname + ".downsample.0.weight", lname + ".downsample.1");
        ew1 = network->addElementWise(*convBd->getOutput(0), *convB->getOutput(0), ElementWiseOperation::kSUM);
    } else {
        ew1 = network->addElementWise(input, *convB->getOutput(0), ElementWiseOperation::kSUM);
    }
    ew1->setName((lname + "_ew").c_str());
    if(relu)
    {
        IActivationLayer* relu1 = network->addActivation(*ew1->getOutput(0), ActivationType::kRELU);
        return relu1->getOutput(0);
    }
    else
        return ew1->getOutput(0);
}

ITensor* Bottleneck(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int inch, int outch, int s, bool relu, std::string lname) {
    Weights emptywts{DataType::kFLOAT, nullptr, 0};
    IActivationLayer *convBNR1 = convBnRelu(network, weightMap, input, outch, 1, 1, 0, lname + ".conv_bn_relu1");
    IActivationLayer *convBNR2 = convBnRelu(network, weightMap, *convBNR1->getOutput(0), outch, 3, s, 1, lname + ".conv_bn_relu2");
    IScaleLayer* convB = ConvBN(network, weightMap, *convBNR2->getOutput(0), outch * 2, 1, 1, 0, lname + ".conv_bn._conv.weight", lname + ".conv_bn._batch_norm");
    IElementWiseLayer *ew1;
    if (inch != outch * 2 || s != 1) {
        IScaleLayer* convBd = ConvBN(network, weightMap, input, outch * 2, 1, s, 0, lname + ".downsample.0.weight", lname + ".downsample.1");
        ew1 = network->addElementWise(*convBd->getOutput(0), *convB->getOutput(0), ElementWiseOperation::kSUM);
    } else {
        ew1 = network->addElementWise(input, *convB->getOutput(0), ElementWiseOperation::kSUM);
    }
    ew1->setName((lname + "_ew").c_str());
    if(relu)
    {
        IActivationLayer* relu1 = network->addActivation(*ew1->getOutput(0), ActivationType::kRELU);
        return relu1->getOutput(0);
    }
    else
        return ew1->getOutput(0);
}

IConvolutionLayer* ddrnet_head(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int int_planes, int output, std::string lname)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };
    IScaleLayer* bn1 = addBatchNorm2d(network, weightMap, input, lname + ".bn1", 1e-5);
    IActivationLayer* relu1 = network->addActivation(*bn1->getOutput(0), ActivationType::kRELU);
    IActivationLayer* conv_bn_relu = convBnRelu(network, weightMap, *relu1->getOutput(0), int_planes, 3, 1, 1, lname + ".conv_bn_relu");
    assert(conv_bn_relu);
    IConvolutionLayer* conv = network->addConvolutionNd(*conv_bn_relu->getOutput(0), output, DimsHW{1,1}, weightMap[lname + ".conv.weight"], weightMap[lname + ".conv.bias"]);
    assert(conv);
    conv->setName((lname + ".conv").c_str());
    conv->setStrideNd(DimsHW{ 1, 1 });
    conv->setPaddingNd(DimsHW{ 0, 0});
    return conv;
}



IConvolutionLayer* scale(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int outplanes, DimsHW k, int s, int p, std::string lname)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };
    IPoolingLayer* avgpool = network->addPoolingNd(input, PoolingType::kAVERAGE, k);
    avgpool->setStrideNd(DimsHW{s, s});
    avgpool->setPaddingNd(DimsHW{p, p});

    IScaleLayer* bn = addBatchNorm2d(network, weightMap, *avgpool->getOutput(0), lname + ".1", 1e-5);
    IActivationLayer* relu = network->addActivation(*bn->getOutput(0), ActivationType::kRELU);
    assert(relu);
    IConvolutionLayer* conv = network->addConvolutionNd(*relu->getOutput(0), outplanes, DimsHW{1,1}, weightMap[lname + ".3.weight"], emptywts);
    conv->setStrideNd(DimsHW{1,1});
    conv->setName((lname + ".conv").c_str());
    conv->setPaddingNd(DimsHW{0,0});
    return conv;
}

IConvolutionLayer* bnReluConv(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int outplanes, int k, int s, int p, std::string lname)
{
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };
    IScaleLayer* bn = addBatchNorm2d(network, weightMap, input, lname + ".0", 1e-5);
    IActivationLayer* relu = network->addActivation(*bn->getOutput(0), ActivationType::kRELU);
    assert(relu);
    IConvolutionLayer* conv = network->addConvolutionNd(*relu->getOutput(0), outplanes, DimsHW{k,k}, weightMap[lname + ".2.weight"], emptywts);
    conv->setStrideNd(DimsHW{s,s});
    conv->setPaddingNd(DimsHW{p,p});
    conv->setName((lname + ".conv").c_str());
    return conv;
}



IElementWiseLayer* Dappm(INetworkDefinition* network, std::map<std::string, Weights>& weightMap, ITensor& input, int branches_planes, int outplanes, std::string lname){
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    Dims inpDims = input.getDimensions();
    IConvolutionLayer* scale0 = bnReluConv(network, weightMap, input, branches_planes, 1, 1, 0, lname + ".scale0");
    IResizeLayer* scale1 = network->addResize(*scale(network, weightMap, input, branches_planes, DimsHW{5, 5}, 2, 2, lname + ".scale1")->getOutput(0));
    scale1->setResizeMode(InterpolationMode::kLINEAR);
    scale1->setOutputDimensions(Dims3{branches_planes, inpDims.d[1], inpDims.d[2]});
    IResizeLayer* scale2 = network->addResize(*scale(network, weightMap, input, branches_planes, DimsHW{9, 9}, 4, 4, lname + ".scale2")->getOutput(0));
    scale2->setResizeMode(InterpolationMode::kLINEAR);
    scale2->setOutputDimensions(Dims3{branches_planes, inpDims.d[1], inpDims.d[2]});
    IResizeLayer* scale3 = network->addResize(*scale(network, weightMap, input, branches_planes, DimsHW{17, 17}, 8, 8, lname + ".scale3")->getOutput(0));
    scale3->setResizeMode(InterpolationMode::kLINEAR);
    scale3->setOutputDimensions(Dims3{branches_planes, inpDims.d[1], inpDims.d[2]});
    IResizeLayer* scale4 = network->addResize(*scale(network, weightMap, input, branches_planes, DimsHW{inpDims.d[1], inpDims.d[2]}, 1, 0, lname + ".scale4")->getOutput(0));
    scale4->setResizeMode(InterpolationMode::kLINEAR);
    scale4->setOutputDimensions(Dims3{branches_planes, inpDims.d[1], inpDims.d[2]});


    IElementWiseLayer* ew1 = network->addElementWise(*scale1->getOutput(0), *scale0->getOutput(0), ElementWiseOperation::kSUM);
    IConvolutionLayer* process1 = bnReluConv(network, weightMap, *ew1->getOutput(0), branches_planes, 3, 1, 1, lname + ".process1");

    IElementWiseLayer* ew2 = network->addElementWise(*scale2->getOutput(0), *process1->getOutput(0), ElementWiseOperation::kSUM);
    IConvolutionLayer* process2 = bnReluConv(network, weightMap, *ew2->getOutput(0), branches_planes, 3, 1, 1, lname + ".process2");

    IElementWiseLayer* ew3 = network->addElementWise(*scale3->getOutput(0), *process2->getOutput(0), ElementWiseOperation::kSUM);
    IConvolutionLayer* process3 = bnReluConv(network, weightMap, *ew3->getOutput(0), branches_planes, 3, 1, 1, lname + ".process3");

    IElementWiseLayer* ew4 = network->addElementWise(*scale4->getOutput(0), *process3->getOutput(0), ElementWiseOperation::kSUM);
    IConvolutionLayer* process4 = bnReluConv(network, weightMap, *ew4->getOutput(0), branches_planes, 3, 1, 1, lname + ".process4");


    ITensor* concatArr[5];
    concatArr[0] = scale0->getOutput(0);
    concatArr[1] = process1->getOutput(0);
    concatArr[2] = process2->getOutput(0);
    concatArr[3] = process3->getOutput(0);
    concatArr[4] = process4->getOutput(0);

    IConcatenationLayer* concat = network->addConcatenation(concatArr, 5);
    concat->setAxis(0);
    concat->setName((lname + "concat").c_str());


    IConvolutionLayer* compression = bnReluConv(network, weightMap, *concat->getOutput(0), outplanes, 1, 1, 0, lname + ".compression");
    IConvolutionLayer* shortcut = bnReluConv(network, weightMap, input, outplanes, 1, 1, 0, lname + ".shortcut");

    IElementWiseLayer* ew =  network->addElementWise(*compression->getOutput(0), *shortcut->getOutput(0), ElementWiseOperation::kSUM);
    ew->setName((lname + "ew_short").c_str());
    return ew;
}

IPluginV2Layer* addArgMax(nvinfer1::INetworkDefinition *network,  nvinfer1::ITensor& input, int axis ,std::string lname)
{
    auto creator = getPluginRegistry()->getPluginCreator("ArgMax_TRT", "1");
    PluginField plugin_fields[2];
    Dims inpDims = input.getDimensions();
    int netinfo[4] = {inpDims.d[0], inpDims.d[1], inpDims.d[2], axis};
    plugin_fields[0].data = netinfo;
    plugin_fields[0].length = 4;
    plugin_fields[0].name = "netinfo";
    plugin_fields[0].type = PluginFieldType::kINT8;


    PluginFieldCollection plugin_data;
    plugin_data.nbFields = 1;
    plugin_data.fields = plugin_fields;
    IPluginV2 *plugin_obj = creator->createPlugin("YoloLayer_TRT", &plugin_data);
    ITensor* inputs[] = { &input };
    auto argmax = network->addPluginV2(inputs, 1, *plugin_obj);
    return argmax;
}