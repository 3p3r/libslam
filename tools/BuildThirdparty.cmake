MESSAGE(STATUS "Building third party depenedencies.")
SET(CMAKE_TRY_COMPILE_CONFIGURATION "Release")
INCLUDE(TryBuildAndInstall)

# ______________________________________________________________________________
# Eigen3
SET(Eigen_ROOT "${SLAM_THIRDPARTY}/eigen")
FIND_PACKAGE(Eigen REQUIRED)

# ______________________________________________________________________________
# Random
SET(Random_ROOT "${SLAM_THIRDPARTY}/random")
FIND_PACKAGE(Random REQUIRED)

# ______________________________________________________________________________
# g2o
SET(G2O_ROOT "${SLAM_THIRDPARTY}/g2o")

TRY_BUILD_AND_INSTALL("${G2O_ROOT}" FLAGS
    "-Dg2o_EIGEN3_INCLUDE=${Eigen_INCLUDE_DIR}"
    "-DCMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}"
    "-DCMAKE_CONFIGURATION_TYPES=Release"
    "-DCMAKE_BUILD_TYPE=Release")

FIND_PACKAGE(G2O REQUIRED)

# ______________________________________________________________________________
# OpenCV
SET(OpenCV_ROOT "${SLAM_THIRDPARTY}/opencv")

TRY_BUILD_AND_INSTALL("${OpenCV_ROOT}" FLAGS
    -DBUILD_WITH_DEBUG_INFO=OFF
    -DBUILD_FAT_JAVA_LIB=OFF
    -DBUILD_opencv_apps=OFF
    -DBUILD_SHARED_LIBS=OFF
    -DBUILD_PERF_TESTS=OFF
    -DBUILD_EXAMPLES=OFF
    -DBUILD_PACKAGE=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_DOCS=OFF
    -DWITH_OPENEXR=OFF
    -DWITH_OPENCL=OFF
    -DWITH_JASPER=OFF
    -DWITH_WEBP=OFF
    -DWITH_JPEG=ON
    -DWITH_TIFF=OFF
    -DWITH_PNG=ON
    -DWITH_IPP=OFF
    -DCV_TRACE=OFF
    -DWITH_CUDA=OFF
    -DWITH_FFMPEG=OFF
    -DWITH_VFW=OFF
    -DWITH_DSHOW=OFF
    -DCMAKE_CONFIGURATION_TYPES=Release
    -DCMAKE_BUILD_TYPE=Release)

IF (MSVC)
  SET(OpenCV_DIR "${OpenCV_ROOT}/install")
ELSE ()
  SET(OpenCV_DIR "${OpenCV_ROOT}/install/share/OpenCV")
ENDIF ()
FIND_PACKAGE(OpenCV REQUIRED NO_DEFAULT_PATH)

# ______________________________________________________________________________
# DBoW3
SET(DBoW3_ROOT "${SLAM_THIRDPARTY}/dbow3")

TRY_BUILD_AND_INSTALL("${DBoW3_ROOT}" FLAGS
    "-DINSTALL_DOC=OFF"
    "-DUSE_O3=ON"
    "-DUSE_SSE3=ON"
    "-DUSE_FAST_MATH=ON"
    "-DBUILD_SHARED_LIBS=OFF"
    "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
    "-DOpenCV_LIBS=${OpenCV_LIBS}"
    "-DOpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}"
    "-DCMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}"
    "-DCMAKE_CONFIGURATION_TYPES=Release"
    "-DCMAKE_BUILD_TYPE=Release")

SET(DBoW3_DIR "${DBoW3_ROOT}/build")
FIND_PACKAGE(DBoW3 REQUIRED)
