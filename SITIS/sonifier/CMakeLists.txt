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

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/app)

# Find installed package

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

find_package(OpenCV REQUIRED )
add_library(sonifier SHARED)
link_directories(sonifier ${PROJECT_SOURCE_DIR}/lib)
target_include_directories(sonifier PUBLIC "${PROJECT_SOURCE_DIR}")
target_sources(sonifier PUBLIC ${SOURCE_FILES} PRIVATE ${HEADER_FILES})

target_link_libraries(
        sonifier
        camera
        marker
        ${OpenCV_LIBS}
)
message(STATUS "Library Sonifier linked")