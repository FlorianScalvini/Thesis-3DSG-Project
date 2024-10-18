cmake_minimum_required(VERSION 3.16)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Check C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -w  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)
include_directories(/home/ubuntu/TensorRT-8.5.1.7/include)
link_directories(/home/ubuntu/TensorRT-8.5.1.7/lib)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/app)

# Find installed package
find_package(OpenCV REQUIRED )
find_package(ALSA REQUIRED)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

add_library(plugins_argmax SHARED Plugins/argmax.cu)
link_directories(add_library ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(plugins_argmax nvinfer cudart)

link_directories(add_library ${PROJECT_SOURCE_DIR}/lib)

add_library(segmentation SHARED Ddrnet/main.cpp)
link_directories(segmentation ${PROJECT_SOURCE_DIR}/lib)
target_include_directories(segmentation PUBLIC "${PROJECT_SOURCE_DIR}")
target_sources(segmentation PUBLIC SegmentationRT.cpp  PRIVATE SegmentationRT.h Segmentation.h macros.h ../utils/logger.h ../utils/cuda_utils.h)
target_link_libraries(
        segmentation
        ${OpenCV_LIBS}
        nvinfer
        cudart
)
message(STATUS "Library Detection linked")