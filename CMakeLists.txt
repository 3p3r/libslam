CMAKE_MINIMUM_REQUIRED(VERSION 3.2 FATAL_ERROR)

# CMake configuration
SET(CMAKE_CXX_STANDARD 11)
SET(CXX_STANDARD_REQUIRED ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
  MESSAGE(AUTHOR_WARNING "CMAKE_BUILD_TYPE is set to Debug.")
ENDIF ()

# Project global variables and configuration
SET(SLAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
SET(SLAM_SOURCES "${SLAM_ROOT}/sources")
SET(SLAM_THIRDPARTY "${SLAM_ROOT}/thirdparty")

# Root project name
PROJECT(libslam CXX)

# This is where we store all our cmake modules
LIST(APPEND CMAKE_MODULE_PATH
    "${SLAM_ROOT}/tools"
    "${SLAM_ROOT}/tools/share/cmake")

# Build everything under the "thirdparty" tree
INCLUDE(BuildThirdparty)

# Setup compiler paths
INCLUDE_DIRECTORIES(
    "${G2O_INCLUDES}"
    "${DBoW3_INCLUDE_DIRS}"
    "${Eigen_INCLUDES}"
    "${Random_INCLUDES}"
    "${OpenCV_INCLUDE_DIRS}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${SLAM_SOURCES}/include")

# Silence G2O / Eigen warnings
ADD_DEFINITIONS("-Wno-deprecated-declarations" "-Wno-deprecated")

# Setup linker inputs (variables set by BuildThirdparty script)
SET(AUX_LIBS ${DBoW3_LIBRARIES} ${G2O_LIBRARY} ${OpenCV_LIBS})

# Add the SLAM library in
FILE(GLOB_RECURSE SLAM_HEADERS "sources/*.h")
FILE(GLOB_RECURSE SLAM_SOURCES "sources/*.cpp")
ADD_LIBRARY(slam SHARED ${SLAM_SOURCES} ${SLAM_HEADERS})
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} slam ${AUX_LIBS})
