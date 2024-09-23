/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

// Standard
#include <cassert>
#include <memory>
// 3rdparty
#include <Eigen/Core>
#include <boost/serialization/serialization.hpp>
#include <opencv2/core.hpp>
// Local
#include "orbslam3/CameraModels/GeometricCamera.h"
#include "orbslam3/TwoViewReconstruction.h"

namespace ORB_SLAM3 {

// Pinhole camera model: parameters corresponds to [fx, fy, cx, cy]
class Pinhole : public GeometricCamera {
public:
  // ──────────────────────────── //
  // Constructors and Destructors

  Pinhole();
  Pinhole(const std::vector<float>& params);

  // ──────────────────────────── //
  // Overriden methods

  Eigen::Vector2f project  (const Eigen::Vector3f& pt) const override;
  cv::Point2f     project  (const cv::Point3f&     pt) const override;
  Eigen::Vector3f unproject(const Eigen::Vector2f& pt) const override;
  cv::Point3f     unproject(const cv::Point2f&     pt) const override;

  JacobianMatrix jacobian(const Eigen::Vector3f& pt) const override;

  Eigen::Matrix3f K() const override;

  float uncertainty(const Eigen::Vector2f& pt) const override;

  bool reconstructFromTwoViews(
    // Inputs.
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector<int>& matches_12,
    // Outputs.
    Sophus::SE3f& T_21, // Transformation from camera 1 to camera 2
    std::vector<cv::Point3f>& points,
    std::vector<bool>& triangulated_flags // Corresponding to each point above
  ) override;

  bool checkEpipolarConstrain(
    const GeometricCamera& camera_2,
    const cv::KeyPoint& keypoint_1,
    const cv::KeyPoint& keypoint_2,
    const Eigen::Matrix3f& R_12, // Rotation from camera 1 to camera 2
    const Eigen::Vector3f& t_12, // Translation from camera 1 to camera 2
    const float sigma_level,
    const float uncertainty
  ) const override;

  bool triangulateKeyPoints(
    // Inputs.
    const GeometricCamera& camera_2,
    const cv::KeyPoint& keypoint_1,
    const cv::KeyPoint& keypoint_2,
    const Eigen::Matrix3f& R_12, // Rotation from camera 1 to camera 2
    const Eigen::Vector3f& t_12, // Translation from camera 1 to camera 2
    const float sigma_level,
    const float uncertainty,
    // Outputs.
    Eigen::Vector3f& triangulated_point
  ) const override;

  // ──────────────────────────── //
  // Public methods

  bool isEqual(const GeometricCamera& other_camera) const;

  // ──────────────────────────── //
  // Operators

  friend std::ostream& operator<<(std::ostream& os, const Pinhole& pinhole);
  friend std::istream& operator>>(std::istream& is,       Pinhole& pinhole);

private:
  // ──────────────────────────── //
  // Serialization

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int) {
    ar& boost::serialization::base_object<GeometricCamera>(*this);
  }

private:
  std::unique_ptr<TwoViewReconstruction> reconstructor_;
};

} // namespace ORB_SLAM3

// BOOST_CLASS_EXPORT_KEY(ORBSLAM2::Pinhole)

#endif // CAMERAMODELS_PINHOLE_H
