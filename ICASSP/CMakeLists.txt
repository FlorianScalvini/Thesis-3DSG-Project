cmake_minimum_required(VERSION 3.16)
project(ICASSP)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RELEASE)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Check C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -w  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)
include_directories(/home/ubuntu/TensorRT-8.5.1.7/include)
link_directories(/home/ubuntu/TensorRT-8.5.1.7/lib)
include(CheckLanguage)
check_language(CUDA)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/app)
set(CMAKE_CUDA_ARCHITECTURES 52 60 61 75 CACHE STRING "CUDA architectures" FORCE)
# Find installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ALSA REQUIRED)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

enable_language(CUDA)
add_subdirectory(Detection)
add_subdirectory(Camera)
link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(ICASSP ${SOURCE_FILES})
target_link_libraries(ICASSP detection camera plugins_yolo detection  -lrealsense2  ${OpenCV_LIBS} ${TORCH_LIBRARIES} ${ALSA_LIBRARIES} pthread  nvinfer cudart)
