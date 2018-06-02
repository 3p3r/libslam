#pragma once

#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2 {

class Settings {
 public:
  explicit Settings(const std::string &settingJson) :
      mSettings(settingJson, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON) {
    if (!mSettings.isOpened()) {
      std::cerr << "Failed to open the settings file at: " << settingJson << std::endl;
      exit(1);
    }
  }

  template<typename T>
  T Get(const std::string &key, T default_value) const {
    if (mSettings[key].empty())
      return default_value;
    else return static_cast<T>(Get(key));
  }

  cv::FileNode Get(const std::string &key) const {
    if (mSettings[key].empty()) {
      std::cerr << "Required settings key [" << key << "] does not exist." << std::endl;
      exit(1);
    } else return mSettings[key];
  }

 private:
  cv::FileStorage mSettings;
};

namespace {
const char CAMERA_FX[] = "Camera.fx"; // Camera focal length X
const char CAMERA_FY[] = "Camera.fy"; // Camera focal length Y
const char CAMERA_CX[] = "Camera.cx"; // Camera focal center X
const char CAMERA_CY[] = "Camera.cy"; // Camera focal center Y
const char CAMERA_K1[] = "Camera.k1"; // Distortion coefficient
const char CAMERA_K2[] = "Camera.k2"; // Distortion coefficient
const char CAMERA_P1[] = "Camera.p1"; // Distortion coefficient
const char CAMERA_P2[] = "Camera.p2"; // Distortion coefficient
const char CAMERA_K3[] = "Camera.k3"; // Distortion coefficient
const char CAMERA_BF[] = "Camera.bf";
const char CAMERA_FPS[] = "Camera.fps";
const char CAMERA_RGB[] = "Camera.RGB";
const char ORB_NFEATURES[] = "ORBextractor.nFeatures";
const char ORB_SCALEFACTOR[] = "ORBextractor.scaleFactor";
const char ORB_NLEVELS[] = "ORBextractor.nLevels";
const char ORB_INITHFAST[] = "ORBextractor.iniThFAST"; // Initial Fast Threshold
const char ORB_MINTHFAST[] = "ORBextractor.minThFAST"; // Minimum Fast Threshold
const char ORB_VOCABULARY[] = "ORBVocabulary.path"; // Path to ORB vocabulary
}

}
