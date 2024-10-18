cmake_minimum_required(VERSION 3.16)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RELEASE)
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


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/app)

# Find installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED )

add_library(camera SHARED)
target_include_directories(camera PRIVATE "${PROJECT_SOURCE_DIR}")
target_sources(camera PRIVATE realsense_camera.cpp PUBLIC ${PROJECT_SOURCE_DIR} video_capture.h realsense_camera.h)

target_link_libraries(
        camera
        ${OpenCV_LIBS} ${realsense2_LIBRARY}
)
message(STATUS "Library Camera linked")