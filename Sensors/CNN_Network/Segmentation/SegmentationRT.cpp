//
// Created by Florian on 23/01/23.
//

#include "SegmentationRT.h"
#include "Plugins/argmax.h"

SegmentationTensorRT::SegmentationTensorRT(const std::string &pathNetwork) {
    this->loadNetwork(pathNetwork);
    // Allocation mémoire RAM + GPU
    const int inputIndex = _pEngine->getBindingIndex("data");
    const int outputIndex = _pEngine->getBindingIndex("out");
    nvinfer1::Dims inputDims = _pEngine->getBindingDimensions(inputIndex);
    this->_widthInput = inputDims.d[2]; // Récupération de la largeur du modele
    this->_heightInput = inputDims.d[1]; // Récupération de la hauteur du modele
    int max_batchSize = _pEngine->getMaxBatchSize(); // Récupération du nombre de batch maximal

    nvinfer1::Dims outputDims = _pEngine->getBindingDimensions(outputIndex);
    this->_widthOutput = outputDims.d[2]; // Récupération de la largeur de l'image de sortie
    this->_heightOutput = outputDims.d[1]; // Récupération de la hauteur de l'image de sortie

    this->_pData = new float[3*_widthInput * _heightInput]; // Color * Batch * H * W
    this->_pOutput = new float [_widthOutput * _heightOutput]; // Batch * outH * outW
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    cudaMalloc(&_pBuffers[inputIndex], 30 * _widthInput * _heightInput * sizeof(float));
    cudaMalloc(&_pBuffers[outputIndex],  _widthOutput * _heightOutput * sizeof(float));
    // Create stream
    cudaStreamCreate(&_stream);
}

void SegmentationTensorRT::loadNetwork(const std::string &pathNetwork) {
    std::ifstream file(pathNetwork, std::ios::binary);  // Lecture du fichier
    assert(file.good()==true);
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, std::ifstream::end);
    size = file.tellg();
    file.seekg(0, std::ifstream::beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    _pRuntime = nvinfer1::createInferRuntime(_gLogger);
    assert(_pRuntime != nullptr);
    _pEngine = _pRuntime->deserializeCudaEngine(trtModelStream, size);
    assert(_pEngine != nullptr);
    _pContext = _pEngine->createExecutionContext();
    assert(_pContext != nullptr);
    delete[] trtModelStream;
    assert(_pEngine->getNbBindings() == 2);
    cv::Mat firstInference(_widthInput, _heightInput, CV_8UC3, cv::Scalar(0, 0, 0));
    this->doInference(firstInference);
}

void SegmentationTensorRT::doInference(cv::Mat img) {
    int i = 0;
    cv::Mat resizeImg;
    if( _pData == nullptr)
        return;
    img.convertTo(resizeImg, CV_32FC3, 1.0);
    cv::resize(resizeImg, resizeImg, cv::Size(_widthInput, _heightInput));
    float * uc_pixel = &resizeImg.at<float>(0);
    for(int col = 0; col < _widthInput; col++)
    {
        for(int row = 0; row < _heightInput; row++)
        {
            _pData[i] = ((float)(uc_pixel[2] / 255.0) - 0.5) / 0.5;
            _pData[_widthInput * _heightInput + i] = ((float)(uc_pixel[1] / 255.0) - 0.5) / 0.5;
            _pData[_widthInput * _heightInput * 2 + i] = ((float)(uc_pixel[0] / 255.0) - 0.5) / 0.5;
            uc_pixel += 3;
            ++i;
        }
    }


    cudaMemcpyAsync(_pBuffers[0], _pData,  3 * _widthOutput * _heightInput  * sizeof(float), cudaMemcpyHostToDevice, _stream);
    _pContext->enqueue(1, _pBuffers, _stream, nullptr);

    cudaMemcpyAsync(_pOutput, _pBuffers[1], _widthOutput * _heightInput  * sizeof(float), cudaMemcpyDeviceToHost, _stream);
    cudaStreamSynchronize(_stream);

}

void SegmentationTensorRT::getOutput(cv::Mat &img) {


    cv::Mat img_seg = cv::Mat(cv::Size(_widthOutput , _heightOutput), CV_8U);
    if(_pOutput != nullptr)
    {
        for (int i = 0; i < _widthOutput * _heightOutput; ++i)
        {
            img_seg.at<unsigned char>(i) = (unsigned char)_pOutput[i];
        }
        img_seg.copyTo(img);
    }
    else
        fprintf(stderr, "The output buffer is not allocated");
}

void SegmentationTensorRT::destroy() {

}