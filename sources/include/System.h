#pragma once

#include <string>
#include <thread>
#include <functional>

#include "Tracking.h"
#include "Settings.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM2 {

class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System {
 public:

  // Initialize the SLAM system. It launches the Local Mapping and Loop Closing threads.
  System(const std::string &strSettingsJson);

  // Process the given monocular frame
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  cv::Mat TrackMonocular(const cv::Mat &im);

  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear map)
  void Reset();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();

  // Information from most recent processed frame
  // You can call this right after TrackMonocular
  int GetTrackingState();
  std::vector<MapPoint *> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  // Get ORB settings
  const Settings& GetSettings() const { return mSettings; }

  // Saves keyframe trajectories in the TUM data format
  void SaveKeyFrameTrajectoryTUM(const std::string &filename);

 private:
  int mMapLastBigChangeIdx;
  int mTrackingState;

  // System threads: Local Mapping, and Loop Closing.
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  std::thread* mptLocalMapping;
  std::thread* mptLoopClosing;
  std::mutex mMutexState;
  std::mutex mMutexReset;
  std::mutex mMutexMode;

  std::vector<MapPoint *> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;

  // ORB settings
  Settings mSettings;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Map *mpMap;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints and
  // performs relocalization if tracking fails.
  Tracking *mpTracker;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  LocalMapping *mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
  // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
  LoopClosing *mpLoopCloser;

  bool mbReset;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM
