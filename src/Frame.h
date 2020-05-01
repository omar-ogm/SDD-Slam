/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  The following code is a derivative work of the code from the ORB-SLAM2 project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/raulmur/ORB_SLAM2>.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef SD_SLAM_FRAME_H
#define SD_SLAM_FRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Config.h"

namespace SD_SLAM {

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame {
 public:
  Frame();

  // Copy constructor.
  Frame(const Frame &frame);

  // Constructor for RGB-D cameras.
  Frame(const cv::Mat &imGray, const cv::Mat &imDepth, ORBextractor* extractor, const Eigen::Matrix3d &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, ORBextractor* extractor, const Eigen::Matrix3d &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

  // Extract ORB on the image
  void ExtractORB(const cv::Mat &im);

  // Set the camera pose.
  void SetPose(const Eigen::Matrix4d &Tcw);

  // Computes rotation, translation and camera center matrices from the camera pose.
  void UpdatePoseMatrices();

  // Returns frame pose
  inline Eigen::Matrix4d GetPose() const {
    return mTcw;
  }

  // Returns frame pose inverse
  inline Eigen::Matrix4d GetPoseInverse() const {
    return mTwc;
  }

  // Returns the camera center.
  inline Eigen::Vector3d GetCameraCenter() {
    return mOw;
  }

  // Returns inverse of rotation
  inline Eigen::Matrix3d GetRotationInverse() {
    return mRwc;
  }

  // Check if a MapPoint is in the frustum of the camera
  // and fill variables of the MapPoint to be used by the tracking
  bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

  // Compute the cell of a keypoint (return false if outside the grid)
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

  std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

  // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
  void ComputeStereoFromRGBD(const cv::Mat &imDepth);

  // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
  Eigen::Vector3d UnprojectStereo(const int &i);

  // Undistort image with current camera parameters
  void Undistort(const cv::Mat& im, cv::Mat& im_out);

 public:
  // Feature extractor.
  ORBextractor* mpORBextractorLeft;

  // Calibration matrix and OpenCV distortion parameters.
  Eigen::Matrix3d mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int N;

  // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
  // In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors;

  // MapPoints associated to keypoints, NULL pointer if no association.
  std::vector<MapPoint*> mvpMapPoints;

  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;

  // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // Camera pose.
  Eigen::Matrix4d mTcw;
  Eigen::Matrix4d mTwc;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame* mpReferenceKF = nullptr;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  std::vector<float> mvScaleFactors;
  std::vector<float> mvInvScaleFactors;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

  // Image pyramid
  std::vector<cv::Mat> mvImagePyramid;
  cv::Mat mDepthImage;

 private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case.
  // (called in the constructor).
  void UndistortKeyPoints();

  // Computes image bounds for the undistorted image (called in the constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
  void AssignFeaturesToGrid();

  // Rotation, translation and camera center
  Eigen::Matrix3d mRcw;
  Eigen::Vector3d mtcw;
  Eigen::Matrix3d mRwc;
  Eigen::Vector3d mOw;  //  == mtwc

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_FRAME_H
