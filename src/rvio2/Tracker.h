/**
 * This file is part of R-VIO2.
 *
 * Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang
 * <ghuang@udel.edu> For more information see <http://github.com/rpng/R-VIO2>
 *
 * R-VIO2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * R-VIO2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <list>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Feature.h"
#include "FeatureDetector.h"
#include "Ransac.h"

namespace RVIO2 {

enum Type {
  // Original values
  // 0: Init slam - reach the max. tracking length
  // 1: Pose only - reach the max. tracking length
  // 2: Pose only - lose track
  // 3: Exploration - local SLAM feature
  INIT_SLAM,
  POSE_ONLY_M,
  POSE_ONLY,
  EXPLO,
  // Return values
  // 5: Unused measurement
  // 6: Bad measurement
  UNUSED,
  BAD
};

class Tracker {
 public:
  explicit Tracker(const cv::FileStorage& fsSettings);

  ~Tracker();

  void track(int nImageId, const cv::Mat& image,
             const Eigen::Matrix3f& RcG, const Eigen::Vector3f& tcG,
             int nMapPtsNeeded, std::unordered_map<int, Feature*>& mFeatures);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  bool start(int nImageId, const cv::Mat& image,
             const Eigen::Matrix3f& RcG, const Eigen::Vector3f& tcG,
             std::unordered_map<int, Feature*>& mFeatures);

  void manage(int nImageId, const cv::Mat& image,
              const Eigen::Matrix3f& RcG, const Eigen::Vector3f& tcG,
              const std::unordered_map<int, Feature*>& mFeatures);

  void preprocess(int nImageId, const cv::Mat& image,
                  const Eigen::Matrix3f& RcG, const Eigen::Vector3f& tcG);

  void undistort(const std::vector<cv::Point2f>& src,
                 std::vector<cv::Point2f>& dst) const;

  void VisualTracking(int nImageId, const cv::Mat& image,
                      int nMapPtsNeeded,
                      std::unordered_map<int, Feature*>& mFeatures);

  static void DisplayTrack(int nImageId, const cv::Mat& image,
                    const std::vector<cv::Point2f>& vPrevFeatUVs,
                    const std::vector<cv::Point2f>& vCurrFeatUVs,
                    const std::vector<unsigned char>& vInlierFlags,
                    cv_bridge::CvImage& imOut);

  static void DisplayNewer(int nImageId, const cv::Mat& image,
                    const std::vector<cv::Point2f>& vRefFeatUVs,
                    const std::vector<cv::Point2f>& vNewFeatUVs,
                    cv_bridge::CvImage& imOut);

  static void OrienVec(const cv::Point2f& pt, Eigen::Vector3f& e) {
    float phi = atan2(pt.y, sqrt(pow(pt.x, 2) + 1));
    float psi = atan2(pt.x, 1);
    e << cos(phi) * sin(psi), sin(phi), cos(phi) * cos(psi);
  }

  float Parallax(const cv::Point2f& pt0, const cv::Point2f& ptk) const {
    Eigen::Vector3f e0, ek;
    OrienVec(pt0, e0);
    OrienVec(ptk, ek);
    float theta = fabs(acos(ek.dot(mRx * e0)));
    return 40 * sin(theta) > 1 ? theta * 180 / M_PI: 0;
  }

 public:
  std::vector<std::pair<int, Type>> mvFeatInfoForInitSlam;
  std::vector<std::vector<cv::Point2f>> mvvFeatMeasForInitSlam;

  std::vector<std::pair<int, Type>> mvFeatInfoForPoseOnly;
  std::vector<std::vector<cv::Point2f>> mvvFeatMeasForPoseOnly;

  std::vector<std::pair<int, cv::Point2f>> mvFeatMeasForExploration;

 private:
  bool mbIsRGB;
  bool mbIsFisheye;

  bool mbRestartVT; // restart visual tracking
  bool mbRefreshVT; // refresh visual tracking

  bool mbEnableSlam;
  bool mbEnableFilter; // Use image filter or not
  bool mbEnableEqualizer;

  int mnMaxFeatsPerImage;

  int mnMinTrackingLength;
  int mnMaxTrackingLength;

  float mnGoodParallax;

  cv::Mat mLastImage;

  cv::Mat mK;
  cv::Mat mD;

  Eigen::Matrix3f mRx; // 当前帧相机坐标系到滑窗内最早相机坐标系的旋转矩阵
  Eigen::Matrix3f mRr; // 当前帧相机坐标系到滑窗内最晚相机坐标系的旋转矩阵

  std::list<Eigen::Matrix3f> mlCamOrientations; // 相机到全局的姿态
  std::list<Eigen::Vector3f> mlCamPositions; // 相机到全局的位置

  std::unordered_map<int, std::vector<cv::Point2f>> mmFeatTrackingHistory; // 所有特征点的追踪历史
  std::vector<int> mvFeatIDsToTrack; // 要追踪的特征点id
  std::vector<cv::Point2f> mvFeatPtsToTrack; // 要追踪的特征点坐标

  std::vector<int> mvFeatIDsInactive;
  std::vector<int> mvFeatIDsLoseTrack;

  Eigen::MatrixXf PointsForRansac;
  std::vector<cv::Point2f> mvFeatCandidates;

  Ransac* mpRansac;
  FeatureDetector* mpFeatureDetector;

  // Interact with rviz
  ros::NodeHandle mTrackerNode;
  ros::Publisher mTrackPub;
  ros::Publisher mNewerPub;

  bool mbShowTrack;
  bool mbShowNewer;
};

}  // namespace RVIO2

#endif
