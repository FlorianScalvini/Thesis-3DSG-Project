cmake_minimum_required(VERSION 3.16)
project(SITIS)

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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/app)
# Find installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ALSA REQUIRED)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

add_subdirectory(marker)
add_subdirectory(Camera)
add_subdirectory(sonifier)
add_subdirectory(sound)
add_subdirectory(path)
link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(SITIS ${SOURCE_FILES})
target_link_libraries(SITIS marker camera sonifier path sound -lrealsense2  ${OpenCV_LIBS} ${ALSA_LIBRARIES} pthread)
