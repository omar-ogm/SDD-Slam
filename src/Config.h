/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
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

#ifndef SD_SLAM_CONFIG_H_
#define SD_SLAM_CONFIG_H_

#include <iostream>
#include <string>
#include "extra/log.h"

namespace SD_SLAM {

struct CameraParameters {
  int w;
  int h;
  double fx;
  double fy;
  double cx;
  double cy;
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
  double fps;
  double bf;
  // Used only by DIFODO
  float fovh_degrees;
  float fovv_degrees;
};

struct DifodoParameters {
  // Check CVDIFODO.h and MRPT::CDIFODO for the meaning of each parameter
  double min_depth_value_filter;
  double max_depth_value_filter;
  int downsample;
  int ctf_levels;
  bool fast_pyramid;
};


class Config {
 public:
  // Singleton
  static Config& GetInstance() {
    static Config instance;
    return instance;
  }

  // Read parameters from file
  bool ReadParameters(std::string filename);

  // Set parameters
  void SetCameraIntrinsics(double w, double h, double fx, double fy, double cx, double cy);
  void SetCameraDistortion(double k1, double k2, double p1, double p2, double k3);
  void SetUsePattern(bool use_pattern);

  // Get parameters
  static double Width() { return GetInstance().camera_params_.w; }
  static double Height() { return GetInstance().camera_params_.h; }
  static double fx() { return GetInstance().camera_params_.fx; }
  static double fy() { return GetInstance().camera_params_.fy; }
  static double cx() { return GetInstance().camera_params_.cx; }
  static double cy() { return GetInstance().camera_params_.cy; }
  static double k1() { return GetInstance().camera_params_.k1; }
  static double k2() { return GetInstance().camera_params_.k2; }
  static double p1() { return GetInstance().camera_params_.p1; }
  static double p2() { return GetInstance().camera_params_.p2; }
  static double k3() { return GetInstance().camera_params_.k3; }
  static double fps() { return GetInstance().camera_params_.fps; }
  static double bf() { return GetInstance().camera_params_.bf; }
  static float fovh() { return GetInstance().camera_params_.fovh_degrees; }
  static float fovv() { return GetInstance().camera_params_.fovv_degrees; }

  static double difodo_min_depth_value_filter() { return GetInstance().difodo_params_.min_depth_value_filter; }
  static double difodo_max_depth_value_filter() { return GetInstance().difodo_params_.max_depth_value_filter; }
  static int difodo_downsample() { return GetInstance().difodo_params_.downsample; }
  static int difodo_ctf_levels() { return GetInstance().difodo_params_.ctf_levels; }
  static bool difodo_fast_pyramid() { return GetInstance().difodo_params_.fast_pyramid; }

  static double UsePattern() { return GetInstance().kUsePattern_; }
  static double ThDepth() { return GetInstance().kThDepth_; }
  static double DepthMapFactor() { return GetInstance().kDepthMapFactor_; }

  static int NumFeatures() { return GetInstance().kNumFeatures_; }
  static double ScaleFactor() { return GetInstance().kScaleFactor_; }
  static int NumLevels() { return GetInstance().kNumLevels_; }
  static int ThresholdFAST() { return GetInstance().kThresholdFAST_; }

  static double KeyFrameSize() { return GetInstance().kKeyFrameSize_; }
  static double KeyFrameLineWidth() { return GetInstance().kKeyFrameLineWidth_; }
  static double GraphLineWidth() { return GetInstance().kGraphLineWidth_; }
  static double PointSize() { return GetInstance().kPointSize_; }
  static double CameraSize() { return GetInstance().kCameraSize_; }
  static double CameraLineWidth() { return GetInstance().kCameraLineWidth_; }
  static double LastPositionsMaxSize() { return GetInstance().kLastPositionsMaxSize_; }
  static double LastPositionsLineWidth() { return GetInstance().kLastPositionsLineWidth_; }
  static double LastPositionsPointWidth() { return GetInstance().kLastPositionsPointWidth_; }
  static double ViewpointX() { return GetInstance().kViewpointX_; }
  static double ViewpointY() { return GetInstance().kViewpointY_; }
  static double ViewpointZ() { return GetInstance().kViewpointZ_; }
  static double ViewpointF() { return GetInstance().kViewpointF_; }

  static std::string CameraTopic() { return GetInstance().kCameraTopic_; }
  static std::string DepthTopic() { return GetInstance().kDepthTopic_; }
  static std::string IMUTopic() { return GetInstance().kIMUTopic_; }
  static std::string BaseFrame() { return GetInstance().kBaseFrame_; }
  static std::string CameraFrame() { return GetInstance().kCameraFrame_; }
  static bool UseImagesTimeStamps() { return GetInstance().kUseImagesTimeStamps_; }

  static cv::Mat NoRGBTimeIntervals() { return GetInstance().noRGBTimeIntervals; }

 private:
  Config();

  // Camera
  CameraParameters camera_params_;

  // DIFODO configuration
  DifodoParameters difodo_params_;

  bool kUsePattern_;
  double kThDepth_;
  double kDepthMapFactor_;

  // ORB Extractor
  int kNumFeatures_;
  double kScaleFactor_;
  int kNumLevels_;
  int kThresholdFAST_;

  // UI
  double kKeyFrameSize_;
  double kKeyFrameLineWidth_;
  double kGraphLineWidth_;
  double kPointSize_;
  double kCameraSize_;
  double kCameraLineWidth_;
  double kLastPositionsMaxSize_;
  double kLastPositionsLineWidth_;
  double kLastPositionsPointWidth_;
  double kViewpointX_;
  double kViewpointY_;
  double kViewpointZ_;
  double kViewpointF_;

  // ROS
  std::string kCameraTopic_;
  std::string kDepthTopic_;
  std::string kIMUTopic_;
  std::string kBaseFrame_;
  std::string kCameraFrame_;
  bool kUseImagesTimeStamps_;

  /**
  * A matrix of any rows (at least 1) and 2 columns. It can be seen as a set of pair of values where each pair,
  * represents the timelapses in seconds where we want to have black images, simulating an environment without light
  */
  cv::Mat noRGBTimeIntervals;
};

}  // namespace SD_SLAM


#endif  // SD_SLAM_CONFIG_H_
