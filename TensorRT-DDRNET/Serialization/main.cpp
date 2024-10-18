#include "config.h"
#include "model.h"
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include <chrono>
#include <fstream>


using namespace nvinfer1;

const static int kOutputSize = kInputW * kInputH;
static Logger gLogger;

void serialize_engine(unsigned int maxBatchSize, std::string& wts_name, std::string& engine_name) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    IHostMemory* serialized_engine = build_ddrnet(maxBatchSize, builder, config, DataType::kFLOAT, wts_name, 64, 128, 128);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cerr << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    delete config;
    delete serialized_engine;
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device, float** output_buffer_host) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));

    *output_buffer_host = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchSize) {
    // infer on the batch asynchronously, and DMA output back to host
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
}

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, std::string& img_dir) {
    if (argc < 3) return false;
    if (std::string(argv[1]) == "-s" && argc == 4) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
    } else if (std::string(argv[1]) == "-d" && argc == 4) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
    } else {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    cudaSetDevice(kGpuId);
    std::string wts_name = "/home/ubuntu/Bureau/Archive_Flo/Ddrnet/model.wts";
    std::string engine_name = "/home/ubuntu/Bureau/Archive_Flo/ddrnet.engine";
    std::string img_dir = "/home/ubuntu/Bureau/Archive_Flo/img_test/";

    if (!wts_name.empty()) {
        serialize_engine(1, wts_name, engine_name);
        return 0;
    }


    // Deserialize the engine from file
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    deserialize_engine(engine_name, &runtime, &engine, &context);
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    float* device_buffers[2];
    float* output_buffer_host = nullptr;
    prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host);

    // Read images from directory
    std::vector<std::string> file_names;
    if (read_files_in_dir(img_dir.c_str(), file_names) < 0) {
        std::cerr << "read_files_in_dir failed." << std::endl;
        return -1;
    }
    cv::Mat img = cv::imread(img_dir + "/" + file_names[0]);
    img.convertTo(img, CV_32FC3, 1.0);
    cv::resize(img, img, cv::Size(kInputW ,kInputH));
    float* _pData = new float[3 * kInputW * kInputH]; // Color * Batch * H * W
    float* _pOutput = new float [kInputW * kInputH ]; // Batch * outH * outW

    void* _pBuffers[2];

    const int inputIndex = engine->getBindingIndex("data");
    const int outputIndex = engine->getBindingIndex("out");
    // Create GPU buffers on device
    cudaMalloc(&_pBuffers[inputIndex], 3 * kInputW * kInputH * sizeof(float));
    cudaMalloc(&_pBuffers[outputIndex],  kInputW * kInputH * sizeof(float));


    int i = 0;
    float * uc_pixel = &img.at<float>(0);
    for(int col = 0; col < img.cols; ++col)
    {
        for(int row = 0; row < img.rows; ++row)
        {

            _pData[i] = ((float)(uc_pixel[2] / 255.0) - 0.5) / 0.5;
            _pData[img.rows * img.cols + i] = ((float)(uc_pixel[1] / 255.0) - 0.5) / 0.5;
            _pData[img.rows * img.cols * 2 + i] = ((float)(uc_pixel[0] / 255.0) - 0.5) / 0.5;
            uc_pixel += 3;
            ++i;
        }
    }
    cudaMemcpyAsync(_pBuffers[0], _pData,  3 * kInputW * kInputH  * sizeof(float), cudaMemcpyHostToDevice, stream);
    context->enqueue(1, _pBuffers, stream, nullptr);

    cudaMemcpyAsync(_pOutput, _pBuffers[1], kInputW * kInputH  * sizeof(float), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    cv::Mat img_seg = cv::Mat(cv::Size(kInputW , kInputH), CV_8U);
    for (int i = 0; i < kInputW * kInputH; ++i)
    {
        img_seg.at<unsigned char>(i) = (unsigned char)_pOutput[i] * 50;
    }
    // Release stream and buffers
    cudaStreamDestroy(stream);
    cv::imwrite("./test.png", img_seg);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    delete[] output_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;

    // Print histogram of the output distribution
    //std::cout << "\nOutput:\n\n";
    //for (unsigned int i = 0; i < kOutputSize; i++)
    //{
    //    std::cout << prob[i] << ", ";
    //    if (i % 10 == 0) std::cout << std::endl;
    //}
    //std::cout << std::endl;

    return 0;
}

