#include "System.h"

namespace ORB_SLAM2 {

System::System(const std::string &strSettingsJson) :
    mMapLastBigChangeIdx(0),
    mTrackingState(Tracking::eTrackingState::NO_IMAGES_YET),
    mSettings(strSettingsJson),
    mpKeyFrameDatabase(nullptr),
    mpMap(nullptr),
    mpTracker(nullptr),
    mpLocalMapper(nullptr),
    mpLoopCloser(nullptr),
    mbReset(false),
    mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false) {
  std::cout << "ORB-SLAM2 patched by VirtuLabs to run headless." << std::endl;
  std::cout << "Input sensor was set to: ";

  //Load ORB Vocabulary
  std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;

  try {
    std::string strVocFile = mSettings.Get(ORB_VOCABULARY, std::string("orbvoc.dbow3"));
    std::cout << "Vocabulary from: " << strVocFile << std::endl;
    mpVocabulary.load(strVocFile);
    std::cout << "Vocabulary loaded!" << std::endl;
  } catch (const std::exception& ex) {
    std::cerr << "Wrong path to vocabulary. " << std::endl;
    std::cerr << "Error: " << ex.what() << std::endl;
    exit(-1);
  }

  //Create KeyFrame Database
  mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);

  //Create the Map
  mpMap = new Map();

  //Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this constructor)
  mpTracker = new Tracking(this, &mpVocabulary, mpMap, mpKeyFrameDatabase);

  //Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(mpMap, true);
  mptLocalMapping = new std::thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

  //Initialize the Loop Closing thread and launch
  mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, &mpVocabulary, false);
  mptLoopClosing = new std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

  //Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  std::cout << "Local Mapping thread: " << mptLocalMapping->get_id();
  std::cout << "Loop Closing thread: " << mptLoopClosing->get_id();
}

cv::Mat System::TrackMonocular(const cv::Mat &im) {
  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      mpLocalMapper->RequestStop();

      // Wait until Local Mapping has effectively stopped
      while (!mpLocalMapper->isStopped()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }

      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
    }
    if (mbDeactivateLocalizationMode) {
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

  return Tcw;
}

void System::ActivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
  int current = mpMap->GetLastBigChangeIdx();
  if (mMapLastBigChangeIdx < current) {
    mMapLastBigChangeIdx = current;
    return true;
  } else
    return false;
}

void System::Reset() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbReset = true;
}

void System::Shutdown() {
  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
    std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }
}

int System::GetTrackingState() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackingState;
}

std::vector<MapPoint *> System::GetTrackedMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
