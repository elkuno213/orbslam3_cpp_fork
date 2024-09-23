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

// Local
#include "orbslam3/CameraModels/Pinhole.h"
#include "orbslam3/Converter.h"

// BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Pinhole)

namespace ORB_SLAM3 {

// BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")

// TODO: why not in KannalaBrandt8?
uint8_t GeometricCamera::next_id = 0;

Pinhole::Pinhole() : GeometricCamera(std::vector<float>(4, 0.f)) {
  id_   = next_id++;
  type_ = Type::Pinhole;
}

Pinhole::Pinhole(const std::vector<float>& params) : GeometricCamera(params) {
  assert(params_.size() == 4);
  id_   = next_id++;
  type_ = Type::Pinhole;
}

Eigen::Vector2f Pinhole::project(const Eigen::Vector3f& pt) const {
  Eigen::Vector2f projected;
  projected[0] = params_[0] * pt[0] / pt[2] + params_[2]; // fx * x / z + cx
  projected[1] = params_[1] * pt[1] / pt[2] + params_[3]; // fy * y / z + cy
  return projected;
}

cv::Point2f Pinhole::project(const cv::Point3f& pt) const {
  return Converter::toCvPoint2f(project(Converter::toEigenVector3f(pt)));
}

Eigen::Vector3f Pinhole::unproject(const Eigen::Vector2f& pt) const {
  return Eigen::Vector3f(
    (pt.x() - params_[2]) / params_[0], // x = (u - cx) / fx
    (pt.y() - params_[3]) / params_[1], // y = (v - cy) / fy
    1.f                                 // z = 1
  );
}

cv::Point3f Pinhole::unproject(const cv::Point2f& pt) const {
  return Converter::toCvPoint3f(unproject(Converter::toEigenVector2f(pt)));
}

GeometricCamera::JacobianMatrix
Pinhole::jacobian(const Eigen::Vector3f& pt) const {
  JacobianMatrix jacobian;
  jacobian(0, 0) =  params_[0] / pt[2];                   // fx / z
  jacobian(0, 1) =  0.f;
  jacobian(0, 2) = -params_[0] * pt[0] / (pt[2] * pt[2]); // -fx * x / z^2
  jacobian(1, 0) =  0.f;
  jacobian(1, 1) =  params_[1] / pt[2];                   // fy / z
  jacobian(1, 2) = -params_[1] * pt[1] / (pt[2] * pt[2]); // -fy * y / z^2

  return jacobian;
}

Eigen::Matrix3f Pinhole::K() const {
  Eigen::Matrix3f K;
  K << params_[0],        0.f, params_[2],
              0.f, params_[1], params_[3],
              0.f,        0.f,        1.f;
  return K;
}

float Pinhole::uncertainty(const Eigen::Vector2f&) const {
  return 1.0;
}

bool Pinhole::reconstructFromTwoViews(
  const std::vector<cv::KeyPoint>& keypoints_1,
  const std::vector<cv::KeyPoint>& keypoints_2,
  const std::vector<int>& matches_12,
  Sophus::SE3f& T_21,
  std::vector<cv::Point3f>& points,
  std::vector<bool>& triangulated_flags
) {
  if (!reconstructor_) {
    reconstructor_ = std::make_unique<TwoViewReconstruction>(K());
  }

  return reconstructor_->reconstruct(
    keypoints_1,
    keypoints_2,
    matches_12,
    T_21,
    points,
    triangulated_flags
  );
}

bool Pinhole::checkEpipolarConstrain(
  const GeometricCamera& camera_2,
  const cv::KeyPoint& keypoint_1,
  const cv::KeyPoint& keypoint_2,
  const Eigen::Matrix3f& R_12,
  const Eigen::Vector3f& t_12,
  const float sigma_level,
  const float uncertainty
) const {
  // Compute Fundamental Matrix.
  Eigen::Matrix3f t_12_hat = Sophus::SO3f::hat(t_12);
  Eigen::Matrix3f K_1      = K();
  Eigen::Matrix3f K_2      = camera_2.K();
  Eigen::Matrix3f F_12     = K_1.transpose().inverse() * t_12_hat * R_12 * K_2.inverse();

  // Compute epipolar line in second image: l = x1'F12 = [a b c]
  const float a = keypoint_1.pt.x * F_12(0, 0) + keypoint_1.pt.y * F_12(1, 0) + F_12(2, 0);
  const float b = keypoint_1.pt.x * F_12(0, 1) + keypoint_1.pt.y * F_12(1, 1) + F_12(2, 1);
  const float c = keypoint_1.pt.x * F_12(0, 2) + keypoint_1.pt.y * F_12(1, 2) + F_12(2, 2);

  // Calculate the squared distance from the keypoint in the second image to the
  // epipolar line.
  const float numerator   = a * keypoint_2.pt.x + b * keypoint_2.pt.y + c;
  const float denominator = a * a + b * b;

  if (denominator == 0.f) {
    return false;
  }

  const float dist_squared = numerator * numerator / denominator;

  // Check if the distance is less than a threshold.
  return dist_squared < 3.84f * uncertainty;
}

bool Pinhole::triangulateKeyPoints(
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
) const {
  return false;
}

bool Pinhole::isEqual(const GeometricCamera& other_camera) const {
  // Check if the type matches.
  if (other_camera.type() != Type::Pinhole) {
    return false;
  }

  const Pinhole* other_pinhole = dynamic_cast<const Pinhole*>(&other_camera);

  // Check if the number of parameters matches.
  if (getNumParams() != other_pinhole->getNumParams()) {
    return false;
  }

  // Check if the parameters match.
  bool is_same = true;
  for (std::size_t i = 0; i < getNumParams(); ++i) {
    if (std::abs(params_[i] - other_pinhole->getParameter(i)) > 1e-6) {
      is_same = false;
      break;
    }
  }

  return is_same;
}

std::ostream& operator<<(std::ostream& os, const Pinhole& pinhole) {
  os << pinhole.params_[0] << " "
     << pinhole.params_[1] << " "
     << pinhole.params_[2] << " "
     << pinhole.params_[3];
  return os;
}

std::istream& operator>>(std::istream& is, Pinhole& pinhole) {
  float next_param;
  for (std::size_t i = 0; i < 4; i++) {
    assert(is.good()); // Make sure the input stream is good
    is >> next_param;
    pinhole.params_[i] = next_param;
  }
  return is;
}

} // namespace ORB_SLAM3
