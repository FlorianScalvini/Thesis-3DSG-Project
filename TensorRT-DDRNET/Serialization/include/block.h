#pragma once

#include "NvInfer.h"
#include <string>
#include <vector>
#include <map>

std::map<std::string, nvinfer1::Weights> loadWeights(const std::string file);

nvinfer1::IElementWiseLayer* convBnSilu(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int c2, int k, int s, int p, std::string lname);

nvinfer1::IActivationLayer* convBnRelu(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int c2, int k, int s, int p, std::string lname, bool bias= false);

nvinfer1::ILayer* ReOrg(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int inch);

nvinfer1::ILayer* DownC(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int c1, int c2, const std::string& lname);

nvinfer1::IElementWiseLayer* SPPCSPC(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int c2, const std::string& lname);

nvinfer1::IElementWiseLayer* RepConv(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int c2, int k, int s, const std::string& lname);

nvinfer1::IActivationLayer* convBlockLeakRelu(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int outch, int ksize, int s, int p, std::string lname);

nvinfer1::IConvolutionLayer* ddrnet_head(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int int_planes, int output, std::string lname);

nvinfer1::IElementWiseLayer* Dappm(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int branches_planes, int outplanes, std::string lname);

nvinfer1::ITensor* Bottleneck(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int inch, int outch, int s, bool relu, std::string lname);

nvinfer1::ITensor* BasicBLock(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int inch, int outch, int s, bool relu, std::string lname);

nvinfer1::IScaleLayer* ConvBN(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, int outch, int k, int s, int p, std::string lnameConv, std::string lnameBatch);

nvinfer1::IPluginV2Layer* addArgMax(nvinfer1::INetworkDefinition *network,  nvinfer1::ITensor& input, int axis ,std::string lname);

nvinfer1::IResizeLayer* resizeLayer(nvinfer1::INetworkDefinition* network, nvinfer1::ITensor& input, nvinfer1::Dims3 dims,  std::string lname);

nvinfer1::IScaleLayer* addBatchNorm2d(nvinfer1::INetworkDefinition* network, std::map<std::string, nvinfer1::Weights>& weightMap, nvinfer1::ITensor& input, std::string lname, float eps);