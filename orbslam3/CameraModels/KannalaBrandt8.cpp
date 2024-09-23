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

// Standard
#include <algorithm>
#include <cassert>
#include <cmath>
// 3rdparty
#include <boost/serialization/export.hpp>
#include <opencv2/calib3d.hpp>
// Local
#include "orbslam3/CameraModels/KannalaBrandt8.h"
#include "orbslam3/Converter.h"
#include "orbslam3/GeometricTools.h"

// BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::KannalaBrandt8)

namespace ORB_SLAM3 {

// BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

KannalaBrandt8::KannalaBrandt8()
  : GeometricCamera(std::vector<float>(8, 0.f))
  , precision_(1e-6)
{
  id_   = next_id++;
  type_ = Type::Fisheye;
}

KannalaBrandt8::KannalaBrandt8(const std::vector<float>& params)
  : GeometricCamera(params)
  , mvLappingArea(2, 0)
  , precision_(1e-6)
{
  assert(params_.size() == 8);
  id_   = next_id++;
  type_ = Type::Fisheye;
}

Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f& pt) const {
  // Compute angles theta and psi.
  const float theta = std::atan2(std::sqrt(pt.x() * pt.x() + pt.y() * pt.y()), pt.z());
  const float psi   = std::atan2(pt.y(), pt.x());

  // Compute the order 9 polynomial r(theta).
  const float theta_2nd = theta * theta;
  const float theta_3rd = theta * theta_2nd;
  const float theta_5th = theta_3rd * theta_2nd;
  const float theta_7th = theta_5th * theta_2nd;
  const float theta_9th = theta_7th * theta_2nd;
  const float r         = theta                   // theta
                        + params_[4] * theta_3rd  // k0 * theta^3
                        + params_[5] * theta_5th  // k1 * theta^5
                        + params_[6] * theta_7th  // k2 * theta^7
                        + params_[7] * theta_9th; // k3 * theta^9

  Eigen::Vector2f projected;
  projected[0] = params_[0] * r * std::cos(psi) + params_[2]; // fx * r * cos(psi) + cx
  projected[1] = params_[1] * r * std::sin(psi) + params_[3]; // fy * r * sin(psi) + cy

  return projected;
}

cv::Point2f KannalaBrandt8::project(const cv::Point3f& pt) const {
  return Converter::toCvPoint2f(project(Converter::toEigenVector3f(pt)));
}

Eigen::Vector3f KannalaBrandt8::unproject(const Eigen::Vector2f& pt) const {
  // Convert pixel coordinates to world coordinates.
  const cv::Point2f pt_w(
    (pt.x() - params_[2]) / params_[0], // (u - cx) / fx
    (pt.y() - params_[3]) / params_[1]  // (v - cy) / fy
  );

  // Compute initial guess for theta, clamp to [-pi/2, pi/2]
  const float distorted_theta = std::clamp(
    std::sqrt(pt_w.x * pt_w.x + pt_w.y * pt_w.y),
    -M_PI_2f,
    M_PI_2f
  );

  float scale = 1.f;

  // Use Newton method to correct for the distortion iteratively until
  // convergence if the distortion is not negligible.
  if (distorted_theta > 1e-8) {
    float theta = distorted_theta;
    for (int j = 0; j < 10; j++) {
      const float theta_2nd = theta * theta;
      const float theta_4th = theta_2nd * theta_2nd;
      const float theta_6th = theta_4th * theta_2nd;
      const float theta_8th = theta_4th * theta_4th;

      const float k0_term = params_[4] * theta_2nd; // k0 * theta^2
      const float k1_term = params_[5] * theta_4th; // k1 * theta^4
      const float k2_term = params_[6] * theta_6th; // k2 * theta^6
      const float k3_term = params_[7] * theta_8th; // k3 * theta^8

      const float polynomial = theta * (1 + k0_term + k1_term + k2_term + k3_term);
      const float derivative = 1 + 3 * k0_term + 5 * k1_term + 7 * k2_term + 9 * k3_term;

      const float theta_correction = (polynomial - distorted_theta) / derivative;

      theta -= theta_correction;

      if (std::abs(theta_correction) < precision_) {
        break;
      }
    }

    // scale = theta - distorted_theta;
    scale = std::tan(theta) / distorted_theta;
  }

  return Eigen::Vector3f(pt_w.x * scale, pt_w.y * scale, 1.f);
}

cv::Point3f KannalaBrandt8::unproject(const cv::Point2f& pt) const {
  return Converter::toCvPoint3f(unproject(Converter::toEigenVector2f(pt)));
}

GeometricCamera::JacobianMatrix KannalaBrandt8::jacobian(
  const Eigen::Vector3f& pt
) const {
  // Compute squared powers of x, y, and z.
  const double x_2nd = pt.x() * pt.x();
  const double y_2nd = pt.y() * pt.y();
  const double z_2nd = pt.z() * pt.z();

  // Compute radius in the image plane and its powers.
  const double r_2nd = x_2nd + y_2nd;
  const double r     = std::sqrt(r_2nd);
  const double r_3rd = r_2nd * r;

  // Compute powers of theta.
  const double theta = std::atan2(r, pt.z());
  const double theta_2nd = theta * theta;
  const double theta_3rd = theta_2nd * theta;
  const double theta_4th = theta_2nd * theta_2nd;
  const double theta_5th = theta_4th * theta;
  const double theta_6th = theta_2nd * theta_4th;
  const double theta_7th = theta_6th * theta;
  const double theta_8th = theta_4th * theta_4th;
  const double theta_9th = theta_8th * theta;

  // Compute f(theta) - radial distortion polynomial.
  const double f = theta
                 + theta_3rd * params_[4]
                 + theta_5th * params_[5]
                 + theta_7th * params_[6]
                 + theta_9th * params_[7];

  // Compute f'(theta) - derivative of the radial distortion polynomial.
  double f_d = 1
                      + 3 * params_[4] * theta_2nd
                      + 5 * params_[5] * theta_4th
                      + 7 * params_[6] * theta_6th
                      + 9 * params_[7] * theta_8th;

  // Compute the Jacobian matrix.
  JacobianMatrix jacobian;

  double factor  = f_d * pt.z() / (r_2nd * (r_2nd + z_2nd));
  jacobian(0, 0) = params_[0] * (x_2nd * factor + y_2nd * f / r_3rd);
  jacobian(1, 1) = params_[1] * (y_2nd * factor + x_2nd * f / r_3rd);

  factor = f_d * pt.z() * pt.y() * pt.x() / (r_2nd * (r_2nd + z_2nd)) - f * pt.y() * pt.x() / r_3rd;
  jacobian(0, 1) = params_[0] * factor;
  jacobian(1, 0) = params_[1] * factor;

  factor = f_d / (r_2nd + z_2nd);
  jacobian(0, 2) = -params_[0] * pt.x() * factor;
  jacobian(1, 2) = -params_[1] * pt.y() * factor;

  return jacobian;
}

Eigen::Matrix3f KannalaBrandt8::K() const {
  Eigen::Matrix3f K;
  K << params_[0],        0.f, params_[2],
              0.f, params_[1], params_[3],
              0.f,        0.f,        1.f;
  return K;
}

float KannalaBrandt8::uncertainty(const Eigen::Vector2f&) const {
  return 1.f;
}

bool KannalaBrandt8::reconstructFromTwoViews(
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

  // Extract 2D points from keypoints.
  std::vector<cv::Point2f> points_1(keypoints_1.size());
  std::transform(
    keypoints_1.begin(),
    keypoints_1.end(),
    points_1.begin(),
    [](const cv::KeyPoint& kp) {
      return kp.pt;
    }
  );
  std::vector<cv::Point2f> points_2(keypoints_2.size());
  std::transform(
    keypoints_2.begin(),
    keypoints_2.end(),
    points_2.begin(),
    [](const cv::KeyPoint& kp) {
      return kp.pt;
    }
  );

  // Undistort the points using the Fisheye model.
  const cv::Mat D = (cv::Mat_<float>(4, 1) << params_[4], params_[5], params_[6], params_[7]);
  const cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  const cv::Mat K_cv = Converter::toCvMat(K());
  cv::fisheye::undistortPoints(points_1, points_1, K_cv, D, R, K_cv);
  cv::fisheye::undistortPoints(points_2, points_2, K_cv, D, R, K_cv);

  // Update the original keypoints with the undistorted points.
  std::vector<cv::KeyPoint> undistorted_keypoints_1(keypoints_1.size());
  std::transform(
    points_1.begin(),
    points_1.end(),
    undistorted_keypoints_1.begin(),
    [](const cv::Point2f& pt) {
      cv::KeyPoint kp;
      kp.pt = pt;
      return kp;
    }
  );
  std::vector<cv::KeyPoint> undistorted_keypoints_2(keypoints_2.size());
  std::transform(
    points_2.begin(),
    points_2.end(),
    undistorted_keypoints_2.begin(),
    [](const cv::Point2f& pt) {
      cv::KeyPoint kp;
      kp.pt = pt;
      return kp;
    }
  );

  return reconstructor_->reconstruct(
    undistorted_keypoints_1,
    undistorted_keypoints_2,
    matches_12,
    T_21,
    points,
    triangulated_flags
  );
}

bool KannalaBrandt8::checkEpipolarConstrain(
  const GeometricCamera& camera_2,
  const cv::KeyPoint& keypoint_1,
  const cv::KeyPoint& keypoint_2,
  const Eigen::Matrix3f& R_12,
  const Eigen::Vector3f& t_12,
  const float sigma_level,
  const float uncertainty
) const {
  Eigen::Vector3f triangulated_point;
  const bool success = triangulateKeyPoints(
    camera_2,
    keypoint_1,
    keypoint_2,
    R_12,
    t_12,
    sigma_level,
    uncertainty,
    triangulated_point
  );
  return success;
}

bool KannalaBrandt8::triangulateKeyPoints(
  const GeometricCamera& camera_2,
  const cv::KeyPoint& keypoint_1,
  const cv::KeyPoint& keypoint_2,
  const Eigen::Matrix3f& R_12,
  const Eigen::Vector3f& t_12,
  const float sigma_level,
  const float uncertainty,
  Eigen::Vector3f& triangulated_point
) const {
  // Unproject keypoints to 3D rays.
  Eigen::Vector3f ray_1 = Converter::toEigenVector3f(unproject(keypoint_1.pt));
  Eigen::Vector3f ray_2 = Converter::toEigenVector3f(camera_2.unproject(keypoint_2.pt));

  // Rotate the second ray to the first camera frame.
  Eigen::Vector3f ray_2_in_1 = R_12 * ray_2;

  // Check parallax.
  const float cos_parallax = ray_1.dot(ray_2_in_1) / (ray_1.norm() * ray_2_in_1.norm());
  if (cos_parallax > 0.9998f) {
    return false;
  }

  // Triangulate 3D point.
  Eigen::Vector3f point;

  Eigen::Matrix<float, 3, 4> T_c1w;
  T_c1w << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

  Eigen::Matrix<float, 3, 4> T_c2w;
  const Eigen::Matrix3f R_21 = R_12.transpose();
  T_c2w << R_21, -R_21 * t_12;

  GeometricTools::triangulate(ray_1, ray_2, T_c1w, T_c2w, point);

  // Check depth in camera 1 frame.
  if (point.z() <= 0.f) {
    return false;
  }

  // Check depth in camera 2 frame.
  if (R_21.row(2).dot(point) + T_c2w(2, 3) <= 0.f) {
    return false;
  }

  // Check reprojection error for keypoint 1.
  if (checkReprojectionError(project(point), keypoint_1.pt, sigma_level)) {
    return false;
  }

  // Check reprojection error for keypoint 2.
  const Eigen::Vector3f triangulated_in_2 = R_21 * point + T_c2w.col(3);
  if (checkReprojectionError(camera_2.project(triangulated_in_2), keypoint_2.pt, uncertainty)) {
    return false;
  }

  triangulated_point = point;

  return true;
}

float KannalaBrandt8::precision() const {
  return precision_;
}

bool KannalaBrandt8::isEqual(const GeometricCamera& other_camera) const {
  // Check if the type matches.
  if (other_camera.type() != Type::Fisheye) {
    return false;
  }

  const KannalaBrandt8* other_kb = dynamic_cast<const KannalaBrandt8*>(&other_camera);

  // Check if the precision matches.
  if (std::abs(precision_ - other_kb->precision()) > 1e-6) {
    return false;
  }

  // Check if the number of parameters matches.
  if (getNumParams() != other_kb->getNumParams()) {
    return false;
  }

  // Check if the parameters match.
  bool is_same = true;
  for (std::size_t i = 0; i < getNumParams(); ++i) {
    if (std::abs(params_[i] - other_kb->getParameter(i)) > 1e-6) {
      is_same = false;
      break;
    }
  }

  return is_same;
}

std::ostream& operator<<(std::ostream& os, const KannalaBrandt8& kb) {
  os << kb.params_[0] << " " << kb.params_[1] << " " << kb.params_[2] << " "
     << kb.params_[3] << " " << kb.params_[4] << " " << kb.params_[5] << " "
     << kb.params_[6] << " " << kb.params_[7];
  return os;
}

std::istream& operator>>(std::istream& is, KannalaBrandt8& kb) {
  float nextParam;
  for (std::size_t i = 0; i < 8; i++) {
    assert(is.good()); // Make sure the input stream is good
    is >> nextParam;
    kb.params_[i] = nextParam;
  }
  return is;
}

bool KannalaBrandt8::checkReprojectionError(
  const Eigen::Vector2f& projected_point,
  const cv::Point2f& keypoint,
  const float sigma_level
) const {
  const float dx = projected_point.x() - keypoint.x;
  const float dy = projected_point.y() - keypoint.y;
  return (dx * dx + dy * dy) > 5.991f * sigma_level;
}

} // namespace ORB_SLAM3
