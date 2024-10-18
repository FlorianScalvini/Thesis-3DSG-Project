//
// Created by florian on 30/08/2021.
//

#include "DetecteurTensorRT.h"
#include <unistd.h>
#include "yolo/yololayer.h"


DetecteurTensorRT::DetecteurTensorRT(const std::string& pathNetwork)
{
    this->loadNetwork(pathNetwork);
    _imgSize = cv::Size(0,0);
    // Allocation mémoire RAM + GPU
    const int inputIndex = _pEngine->getBindingIndex("data");
    const int outputIndex = _pEngine->getBindingIndex("prob");
    nvinfer1::Dims inputDims = _pEngine->getBindingDimensions(inputIndex);
    this->_widthNetwork = inputDims.d[2]; // Récupération de la largeur du modele
    this->_heightNetwork = inputDims.d[1]; // Récupération de la hauteur du modele
    int max_batchSize = _pEngine->getMaxBatchSize(); // Récupération du nombre de batch maximal
    this->_pData = new float[max_batchSize * 3 * _widthNetwork * _heightNetwork];
    this->_pProb = new float [max_batchSize * OUTPUT_SIZE];
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    cudaMalloc(&_pBuffers[inputIndex], 3 * _heightNetwork * _widthNetwork * sizeof(float));
    cudaMalloc(&_pBuffers[outputIndex], OUTPUT_SIZE * sizeof(float));
    // Create stream
    cudaStreamCreate(&_stream);


    cv::Mat firstInference(_widthNetwork, _heightNetwork, CV_8UC3, cv::Scalar(0, 0, 0));
    this->doInference(firstInference);
}

void DetecteurTensorRT::loadNetwork(const std::string& pathNetwork)
{
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
}



void DetecteurTensorRT::doInference(cv::Mat& img)
{
    _imgSize = cv::Size(img.cols, img.rows);
    preprocess(img);
    cudaMemcpyAsync(_pBuffers[0], _pData,  3 * _heightNetwork * _widthNetwork * sizeof(float), cudaMemcpyHostToDevice, _stream);
    _pContext->enqueue(1, _pBuffers, _stream, nullptr);
    cudaMemcpyAsync(_pProb, _pBuffers[1], OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, _stream);
    cudaStreamSynchronize(_stream);
}

void DetecteurTensorRT::preprocess(cv::Mat &img) {
    cv::Mat image = letterBox(img);
    int i = 0;
    for(int row = 0; row < _heightNetwork; ++row)
    {
        uchar* uc_pixel = image.data + row * image.step;
        for(int col = 0; col < _widthNetwork; ++col)
        {
            _pData[i] = (float)(uc_pixel[0] / 255.0);
            _pData[_widthNetwork * _heightNetwork + i] = (float)(uc_pixel[1] / 255.0);
            _pData[_widthNetwork * _heightNetwork * 2 + i] = (float)(uc_pixel[2] / 255.0);
            uc_pixel += 3;
            ++i;
        }
    }
}



std::map<unsigned int, std::vector<ObjectBoundingBox>>DetecteurTensorRT::getOutput()
{

    int det_size = 6; // 4 Bbox  + Conf + Class id
    std::map<unsigned int, std::vector<ObjectBoundingBox>> output;
    for (int i = 0; i < _pProb[0] && i < 1000; i++) {
        if (_pProb[1 + det_size * i + 4] <= CONF_THRESH) continue;
        yoloToRect(&_pProb[1 + det_size * i]);
        ObjectBoundingBox det = ObjectBoundingBox(&_pProb[1 + det_size * i]);
        if (output.count((int)det.class_id) == 0)
            output.emplace((unsigned int) det.class_id, std::vector<ObjectBoundingBox>());
        output[(int)det.class_id].push_back(det);
    }
    nms(output);
    return output;
}

void DetecteurTensorRT::destroy()
{

}

