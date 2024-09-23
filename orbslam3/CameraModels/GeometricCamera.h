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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

// Standard
#include <vector>
// 3rdparty
#include <Eigen/Core>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core.hpp>
#include <orbslam3/external/Sophus/sophus/se3.hpp>

namespace ORB_SLAM3 {

class GeometricCamera {
public:
  enum class Type : uint8_t {
    Pinhole = 0,
    Fisheye = 1
  };

  using JacobianMatrix = Eigen::Matrix<float, 2, 3>;

  // ──────────────────────────── //
  // Constructors and Destructors

  GeometricCamera() = default;
  GeometricCamera(const std::vector<float>& params) : params_(params) {}
  virtual ~GeometricCamera() = default;

  // ──────────────────────────── //
  // Pure virtual methods

  virtual Eigen::Vector2f project  (const Eigen::Vector3f& pt) const = 0;
  virtual cv::Point2f     project  (const cv::Point3f&     pt) const = 0;
  virtual Eigen::Vector3f unproject(const Eigen::Vector2f& pt) const = 0;
  virtual cv::Point3f     unproject(const cv::Point2f&     pt) const = 0;

  virtual JacobianMatrix jacobian(const Eigen::Vector3f& pt) const = 0;

  virtual Eigen::Matrix3f K() const = 0;

  virtual float uncertainty(const Eigen::Vector2f& pt) const = 0;

  virtual bool reconstructFromTwoViews(
    // Inputs.
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector<int>& matches_12,
    // Outputs.
    Sophus::SE3f& T_21, // Transformation from camera 1 to camera 2
    std::vector<cv::Point3f>& points,
    std::vector<bool>& triangulated_flags // Corresponding to each point above
  ) = 0;

  virtual bool checkEpipolarConstrain(
    const GeometricCamera& camera_2,
    const cv::KeyPoint& keypoint_1,
    const cv::KeyPoint& keypoint_2,
    const Eigen::Matrix3f& R_12, // Rotation from camera 1 to camera 2
    const Eigen::Vector3f& t_12, // Translation from camera 1 to camera 2
    const float sigma_level,
    const float uncertainty
  ) const = 0;

  virtual bool triangulateKeyPoints(
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
  ) const = 0;

  // ──────────────────────────── //
  // Setters and Getters

  float getParameter(const std::size_t idx) const {
    return params_[idx];
  }

  void setParameter(const float param, const std::size_t idx) {
    params_[idx] = param;
  }

  std::size_t getNumParams() const {
    return params_.size();
  }

  uint8_t id() const {
    return id_;
  }

  Type type() const {
    return type_;
  }

  // ──────────────────────────── //
  // Static members
  static uint8_t next_id;

private:
  // ──────────────────────────── //
  // Serialization

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int) {
    ar& id_;
    ar& type_;
    ar& params_;
  }

protected:
  std::vector<float> params_;
  uint8_t id_;
  Type type_;
};

} // namespace ORB_SLAM3

#endif // CAMERAMODELS_GEOMETRICCAMERA_H
