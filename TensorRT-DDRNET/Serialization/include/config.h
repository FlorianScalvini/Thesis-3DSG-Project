#pragma once

/* --------------------------------------------------------
 * These configs are related to tensorrt model, if these are changed,
 * please re-compile and re-serialize the tensorrt model.
 * --------------------------------------------------------*/

#define USE_FP16 // set USE_INT8 or USE_FP16 or USE_FP32

// These are used to define input/output tensor names,
// you can set them to whatever you want.
const static char* kInputTensorName = "data";
const static char* kOutputTensorName = "out";

const static int kNumClass = 5;
const static int kBatchSize = 1;

const static int kInputH = 512;
const static int kInputW = 1024;


/* --------------------------------------------------------
 * These configs are not related to tensorrt model, if these are changed,
 * please re-compile, but no need to re-serialize the tensorrt model.
 * --------------------------------------------------------*/

const static int kGpuId = 0;

// If your image size is larger than 4096 * 3112, please increase this value
const static int kMaxInputImageSize = 4096 * 3112;

