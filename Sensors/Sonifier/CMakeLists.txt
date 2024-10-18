cmake_minimum_required(VERSION 3.16)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
MESSAGE("Build type: " ${PROJECT_SOURCE_DIR})

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
find_package(OpenCV REQUIRED )
find_package(ALSA REQUIRED)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)
add_library(sonifier)

link_directories(sonifier ${PROJECT_SOURCE_DIR}/lib)
target_include_directories(sonifier PUBLIC "${PROJECT_SOURCE_DIR}")
target_sources(sonifier PUBLIC ${SOURCE_FILES} PRIVATE ${HEADER_FILES})
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Add the necessary include directories and compiler flags for Qt 5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


target_link_libraries(
        sonifier
        camera
        common
        sound
        sensor
        qt
        ${OpenCV_LIBS}
        Qt5::Widgets
)
message(STATUS "Library Sonifier linked")