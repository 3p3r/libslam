#include "Tracking.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "PnPsolver.h"

using namespace std;

namespace ORB_SLAM2 {

Tracking::Tracking(
    System *sys,
    ORBVocabulary *vocabulary,
    Map *map,
    KeyFrameDatabase *keyFrameDatabase)
    :
    mState(NO_IMAGES_YET),
    mbOnlyTracking(false),
    mbVO(false),
    mpORBVocabulary(vocabulary),
    mpKeyFrameDB(keyFrameDatabase),
    mpInitializer(nullptr),
    mpSystem(sys),
    mpMap(map),
    mnLastRelocFrameId(0) {
  // Load camera parameters from settings file
  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = sys->GetSettings().Get(CAMERA_FX);
  K.at<float>(1, 1) = sys->GetSettings().Get(CAMERA_FY);
  K.at<float>(0, 2) = sys->GetSettings().Get(CAMERA_CX);
  K.at<float>(1, 2) = sys->GetSettings().Get(CAMERA_CY);
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = sys->GetSettings().Get(CAMERA_K1);
  DistCoef.at<float>(1) = sys->GetSettings().Get(CAMERA_K2);
  DistCoef.at<float>(2) = sys->GetSettings().Get(CAMERA_P1);
  DistCoef.at<float>(3) = sys->GetSettings().Get(CAMERA_P2);
  const float k3 = sys->GetSettings().Get(CAMERA_K3, 0.0f);
  if (k3!=0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = sys->GetSettings().Get(CAMERA_BF, 0.0f);

  // Max/Min Frames to insert keyframes and to check relocalization
  mMinFrames = 0;
  mMaxFrames = sys->GetSettings().Get(CAMERA_FPS, 30);

  cout << endl << "Camera Parameters: " << endl;
  cout << "- k1: " << DistCoef.at<float>(0) << endl;
  cout << "- k2: " << DistCoef.at<float>(1) << endl;
  if (DistCoef.rows==5)
    cout << "- k3: " << DistCoef.at<float>(4) << endl;
  cout << "- p1: " << DistCoef.at<float>(2) << endl;
  cout << "- p2: " << DistCoef.at<float>(3) << endl;
  cout << "- fps: " << mMaxFrames << endl;

  int nRGB = sys->GetSettings().Get(CAMERA_RGB, 1);
  mbRGB = (nRGB!=0);

  if (mbRGB)
    cout << "- color order: RGB (ignored if grayscale)" << endl;
  else
    cout << "- color order: BGR (ignored if grayscale)" << endl;

  // Load ORB parameters

  int nFeatures = sys->GetSettings().Get(ORB_NFEATURES, 1000);
  float fScaleFactor = sys->GetSettings().Get(ORB_SCALEFACTOR, 1.2f);
  int nLevels = sys->GetSettings().Get(ORB_NLEVELS, 8);
  int fIniThFAST = sys->GetSettings().Get(ORB_INITHFAST, 20);
  int fMinThFAST = sys->GetSettings().Get(ORB_MINTHFAST, 7);

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
  mpIniORBextractor = new ORBextractor(2*nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

  cout << endl << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
  cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
  mImGray = im;

  if (mImGray.channels()==3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGR2GRAY);
  } else if (mImGray.channels()==4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
  }

  if (mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
    mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
  else
    mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

  Track();

  return mCurrentFrame.mTcw.clone();
}

void Tracking::Track() {
  if (mState==NO_IMAGES_YET) {
    mState = NOT_INITIALIZED;
  }

  // Get Map Mutex -> Map cannot be changed
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

  if (mState==NOT_INITIALIZED) {
    MonocularInitialization();

    if (mState!=OK)
      return;
  } else {
    // System is initialized. Track Frame.
    bool bOK;

    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
    if (!mbOnlyTracking) {
      // Local Mapping is activated. This is the normal behaviour, unless
      // you explicitly activate the "only tracking" mode.

      if (mState==OK) {
        // Local Mapping might have changed some MapPoints tracked in last frame
        CheckReplacedInLastFrame();

        if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
          bOK = TrackReferenceKeyFrame();
        } else {
          bOK = TrackWithMotionModel();
          if (!bOK)
            bOK = TrackReferenceKeyFrame();
        }
      } else {
        bOK = Relocalization();
      }
    } else {
      // Localization Mode: Local Mapping is deactivated

      if (mState==LOST) {
        bOK = Relocalization();
      } else {
        if (!mbVO) {
          // In last frame we tracked enough MapPoints in the map

          if (!mVelocity.empty()) {
            bOK = TrackWithMotionModel();
          } else {
            bOK = TrackReferenceKeyFrame();
          }
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing relocalization.
          // If relocalization is successful we choose that solution, otherwise we retain
          // the "visual odometry" solution.

          bool bOKMM = false;
          vector<MapPoint *> vpMPsMM;
          vector<bool> vbOutMM;
          cv::Mat TcwMM;
          if (!mVelocity.empty()) {
            bOKMM = TrackWithMotionModel();
            vpMPsMM = mCurrentFrame.mvpMapPoints;
            vbOutMM = mCurrentFrame.mvbOutlier;
            TcwMM = mCurrentFrame.mTcw.clone();
          }

          bool bOKReloc = Relocalization();
          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            mCurrentFrame.mvpMapPoints = vpMPsMM;
            mCurrentFrame.mvbOutlier = vbOutMM;

            if (mbVO) {
              for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
              }
            }
          } else if (bOKReloc) {
            mbVO = false;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }

    mCurrentFrame.mpReferenceKF = mpReferenceKF;

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (!mbOnlyTracking) {
      if (bOK)
        bOK = TrackLocalMap();
    } else {
      // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
      // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
      // the camera we will use the local map again.
      if (bOK && !mbVO)
        bOK = TrackLocalMap();
    }

    if (bOK)
      mState = OK;
    else
      mState = LOST;

    // If tracking were good, check if we insert a keyframe
    if (bOK) {
      // Update motion model
      if (!mLastFrame.mTcw.empty()) {
        cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
        mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
        mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
        mVelocity = mCurrentFrame.mTcw*LastTwc;
      } else
        mVelocity = cv::Mat();

      // Clean VO matches
      for (int i = 0; i < mCurrentFrame.N; i++) {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations() < 1) {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = nullptr;
          }
      }

      // Delete temporal MapPoints
      for (auto pMP : mlpTemporalPoints) delete pMP;
      mlpTemporalPoints.clear();

      // Check if we need to insert a new keyframe
      if (NeedNewKeyFrame())
        CreateNewKeyFrame();

      // We allow points with high innovation (considered outliers by the Huber Function)
      // pass to the new keyframe, so that bundle adjustment will finally decide
      // if they are outliers or not. We don't want next frame to estimate its position
      // with those points so we discard them in the frame.
      for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = nullptr;
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState==LOST) {
      if (mpMap->KeyFramesInMap() <= 5) {
        cout << "Track lost soon after initialisation, resetting..." << endl;
        mpSystem->Reset();
        return;
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    mLastFrame = Frame(mCurrentFrame);
  }

  // Store frame pose information to retrieve the complete camera trajectory afterwards.
  if (!mCurrentFrame.mTcw.empty()) {
    cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
    mlRelativeFramePoses.push_back(Tcr);
    mlpReferences.push_back(mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    mlbLost.push_back(mState==LOST);
  } else {
    // This can happen if tracking is lost
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState==LOST);
  }

}

void Tracking::MonocularInitialization() {

  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {
      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      delete mpInitializer;

      mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      return;
    }
  } else {
    // Try to initialize
    if ((int) mCurrentFrame.mvKeys.size() <= 100) {
      delete mpInitializer;
      mpInitializer = nullptr;
      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9f, true);
    int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // Check if there are enough correspondences
    if (nmatches < 100) {
      delete mpInitializer;
      mpInitializer = nullptr;
      return;
    }

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i] = -1;
          nmatches--;
        }
      }

      // Set Frame Poses
      mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
      cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      tcw.copyTo(Tcw.rowRange(0, 3).col(3));
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  auto pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
  auto pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0)
      continue;

    //Create MapPoint.
    cv::Mat worldPos(mvIniP3D[i]);

    auto pMP = new MapPoint(worldPos, pKFcur, mpMap);

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, static_cast<size_t>(mvIniMatches[i]));

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, static_cast<size_t>(mvIniMatches[i]));

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    //Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    //Add to Map
    mpMap->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  // Bundle Adjustment
  cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

  Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

  // Set median depth to 1
  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth = 1.0f/medianDepth;

  if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
    cout << "Wrong initialization, reseting..." << endl;
    Reset();
    return;
  }

  // Scale initial baseline
  cv::Mat Tc2w = pKFcur->GetPose();
  Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (auto &vpAllMapPoint : vpAllMapPoints) {
    if (vpAllMapPoint) {
      MapPoint *pMP = vpAllMapPoint;
      pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
    }
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = static_cast<unsigned int>(mCurrentFrame.mnId);

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  mLastFrame = Frame(mCurrentFrame);

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPoint *pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      MapPoint *pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.7f, true);
  vector<MapPoint *> vpMapPointMatches;

  int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

  if (nmatches < 15)
    return false;

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;
  mCurrentFrame.SetPose(mLastFrame.mTcw);

  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = nullptr;
        mCurrentFrame.mvbOutlier[i] = false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFrame *pRef = mLastFrame.mpReferenceKF;
  cv::Mat Tlr = mlRelativeFramePoses.back();

  mLastFrame.SetPose(Tlr*pRef->GetPose());
}

bool Tracking::TrackWithMotionModel() {
  ORBmatcher matcher(0.9f, true);

  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  UpdateLastFrame();

  mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);

  // Project points seen in previous frame
  int th = 15;
  int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, true /*mSensor==System::MONOCULAR*/);

  // If few matches, uses a wider window search
  if (nmatches < 20) {
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);
    nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*th, true /*mSensor==System::MONOCULAR*/);
  }

  if (nmatches < 20)
    return false;

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = nullptr;
        mCurrentFrame.mvbOutlier[i] = false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if (mbOnlyTracking) {
    mbVO = nmatchesMap < 10;
    return nmatches > 20;
  }

  return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the frame.
  // We retrieve the local map and try to find matches to points in the local map.

  UpdateLocalMap();

  SearchLocalPoints();

  // Optimize Pose
  Optimizer::PoseOptimization(&mCurrentFrame);
  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (!mbOnlyTracking) {
          if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
            mnMatchesInliers++;
        } else
          mnMatchesInliers++;
      }
    }
  }

  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
    return false;

  return mnMatchesInliers >= 30;
}

bool Tracking::NeedNewKeyFrame() {
  if (mbOnlyTracking)
    return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  auto nKFs = static_cast<int>(mpMap->KeyFramesInMap());

  // Do not insert keyframes if not enough frames have passed from last relocalisation
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
    return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2)
    nMinObs = 2;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Check how many "close" points are being tracked and how many could be potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;
  bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

  // Thresholds
  float thRefRatio = 0.9f;

  // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
  const bool c2 = ((mnMatchesInliers < nRefMatches*thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

  if ((c1a || c1b) && c2) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  if (!mpLocalMapper->SetNotStop(true))
    return;

  auto pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = static_cast<unsigned int>(mCurrentFrame.mnId);
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (auto &mvpMapPoint : mCurrentFrame.mvpMapPoints) {
    MapPoint *pMP = mvpMapPoint;
    if (pMP) {
      if (pMP->isBad()) {
        mvpMapPoint = nullptr;
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;
      }
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (auto pMP : mvpLocalMapPoints) {
    if (pMP->mnLastFrameSeen==mCurrentFrame.mnId)
      continue;
    if (pMP->isBad())
      continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8f);
    int th = 1;
    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
      th = 5;
    matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
       itKF!=itEndKF; itKF++) {
    KeyFrame *pKF = *itKF;
    const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

    for (auto pMP : vpMPs) {
      if (!pMP)
        continue;
      if (pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
        continue;
      if (!pMP->isBad()) {
        mvpLocalMapPoints.push_back(pMP);
        pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
      }
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame *, int> keyframeCounter;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
      if (!pMP->isBad()) {
        const map<KeyFrame *, size_t> observations = pMP->GetObservations();
        for (auto observation : observations)
          keyframeCounter[observation.first]++;
      } else {
        mCurrentFrame.mvpMapPoints[i] = nullptr;
      }
    }
  }

  if (keyframeCounter.empty())
    return;

  int max = 0;
  KeyFrame *pKFmax = nullptr;

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
  for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it!=itEnd;
       it++) {
    KeyFrame *pKF = it->first;

    if (pKF->isBad())
      continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }


  // Include also some not-already-included keyframes that are neighbors to already-included keyframes
  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
       itKF!=itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80)
      break;

    KeyFrame *pKF = *itKF;

    const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (auto pNeighKF : vNeighs) {
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    const set<KeyFrame *> spChilds = pKF->GetChilds();
    for (auto pChildKF : spChilds) {
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame *pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }

  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
  vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

  if (vpCandidateKFs.empty())
    return false;

  auto nKFs = static_cast<int>(vpCandidateKFs.size());

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true);

  vector<PnPsolver *> vpPnPsolvers;
  vpPnPsolvers.resize(static_cast<unsigned long>(nKFs));

  vector<vector<MapPoint *> > vvpMapPointMatches;
  vvpMapPointMatches.resize(static_cast<unsigned long>(nKFs));

  vector<bool> vbDiscarded;
  vbDiscarded.resize(static_cast<unsigned long>(nKFs));

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame *pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        auto pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5f, 5.991f);
        vpPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  ORBmatcher matcher2(0.9f, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i])
        continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      PnPsolver *pSolver = vpPnPsolvers[i];
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (!Tcw.empty()) {
        Tcw.copyTo(mCurrentFrame.mTcw);

        set<MapPoint *> sFound;

        auto np = static_cast<int>(vbInliers.size());

        for (int j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.mvpMapPoints[j] = nullptr;
        }

        int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        if (nGood < 10)
          continue;

        for (int io = 0; io < mCurrentFrame.N; io++)
          if (mCurrentFrame.mvbOutlier[io])
            mCurrentFrame.mvpMapPoints[io] = nullptr;

        // If few inliers, search by projection in a coarse window and optimize again
        if (nGood < 50) {
          int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

          if (nadditional + nGood >= 50) {
            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

            // If many inliers but still not enough, search by projection again in a narrower window
            // the camera has been already optimized with many points
            if (nGood > 30 && nGood < 50) {
              sFound.clear();
              for (int ip = 0; ip < mCurrentFrame.N; ip++)
                if (mCurrentFrame.mvpMapPoints[ip])
                  sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
              nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

              // Final optimization
              if (nGood + nadditional >= 50) {
                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                for (int io = 0; io < mCurrentFrame.N; io++)
                  if (mCurrentFrame.mvbOutlier[io])
                    mCurrentFrame.mvpMapPoints[io] = nullptr;
              }
            }
          }
        }


        // If the pose is supported by enough inliers stop ransacs and continue
        if (nGood >= 50) {
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    return false;
  } else {
    mnLastRelocFrameId = static_cast<unsigned int>(mCurrentFrame.mnId);
    return true;
  }

}

void Tracking::Reset() {
  cout << "System Resetting" << endl;

  // Reset Local Mapping
  cout << "Resetting Local Mapper...";
  mpLocalMapper->RequestReset();
  cout << " done" << endl;

  // Reset Loop Closing
  cout << "Resetting Loop Closing...";
  mpLoopClosing->RequestReset();
  cout << " done" << endl;

  // Clear BoW Database
  cout << "Resetting Database...";
  mpKeyFrameDB->clear();
  cout << " done" << endl;

  // Clear Map (this erase MapPoints and KeyFrames)
  mpMap->clear();

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;

  if (mpInitializer) {
    delete mpInitializer;
    mpInitializer = nullptr;
  }

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();
}

void Tracking::InformOnlyTracking(const bool &flag) {
  mbOnlyTracking = flag;
}

} //namespace ORB_SLAM
