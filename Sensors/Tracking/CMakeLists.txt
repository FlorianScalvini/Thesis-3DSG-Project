cmake_minimum_required(VERSION 3.16)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
include(CheckCXXCompilerFlag)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -w  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w ")

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../app)

# Find installed package
find_package(OpenCV REQUIRED )
find_package(ALSA REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB SOURCE_FILES *.cpp ./ByteTrack/*.cpp ./Sort/*.cpp)
file(GLOB HEADER_FILES *.h ./ByteTrack/*.h ./Sort/*.h)

add_library(tracking SHARED TrackingAlgorithm.cpp TrackingAlgorithm.h)
link_directories(tracking ../lib)
link_directories(${Eigen_INCLUDE_DIRS})
target_include_directories(tracking PUBLIC "${PROJECT_SOURCE_DIR}")
target_sources(tracking PUBLIC ${SOURCE_FILES} PRIVATE ${HEADER_FILES})

target_link_libraries(
        tracking
        ${OpenCV_LIBS}
        ${Eigen_LIBRARIES}
)
message(STATUS "Library Tracking linked")
