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

// 3rdparty
#include <glog/logging.h>
// Local
#include "orbslam3/CameraModels/GeometricCamera.h"
#include "orbslam3/Frame.h"
#include "orbslam3/G2oTypes.h"
#include "orbslam3/KeyFrame.h"

namespace ORB_SLAM3 {

// ────────────────────────────────────────────────────────────────────────── //
// Functions

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W <<    0.0,    -w.z(),    w.y(),
        w.z(),       0.0,   -w.x(),
       -w.y(),     w.x(),      0.0;
  return W;
}

Eigen::Matrix3d normalizeRotation(const Eigen::Matrix3d& R) {
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    R,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  const double theta_squared = w.squaredNorm();
  const double theta         = std::sqrt(theta_squared);
  const Eigen::Matrix3d W    = skew(w);

  if (theta < 1e-5) {
    // Approximation for small angles.
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
    return normalizeRotation(R);
  } else {
    // Rodrigues' formula.
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                            + W * std::sin(theta) / theta
                            + W * W * (1.0 - std::cos(theta)) / theta_squared;
    return normalizeRotation(R);
  }
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  const Eigen::Vector3d w(
    (R(2, 1) - R(1, 2)) / 2.0,
    (R(0, 2) - R(2, 0)) / 2.0,
    (R(1, 0) - R(0, 1)) / 2.0
  );

  const double cos_theta = (R.trace() - 1.0) * 0.5;
  if (std::abs(cos_theta) > 1.0) {
    // No rotation.
    return w;
  }

  const double theta     = std::acos(cos_theta);
  const double sin_theta = std::sin(theta);
  if (std::abs(sin_theta) < 1e-5) {
    // Small angle approximation.
    return w;
  } else {
    return theta * w / sin_theta;
  }
}

Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& w) {
  const double theta_squared = w.squaredNorm();
  const double theta         = std::sqrt(theta_squared);
  const Eigen::Matrix3d W    = skew(w);

  if (theta < 1e-5) {
    // No rotation.
    return Eigen::Matrix3d::Identity();
  } else {
    return Eigen::Matrix3d::Identity()
         - W * (1.0 - std::cos(theta)) / theta_squared
         + W * W * (theta - std::sin(theta)) / (theta_squared * theta);
  }
}

Eigen::Matrix3d inverseRightJacobianSO3(const Eigen::Vector3d& w) {
  const double theta_squared = w.squaredNorm();
  const double theta         = std::sqrt(theta_squared);
  const Eigen::Matrix3d W    = skew(w);

  if (theta < 1e-5) {
    // No rotation.
    return Eigen::Matrix3d::Identity();
  } else {
    return Eigen::Matrix3d::Identity()
         + W / 2.0
         + W * W * (1.0 / theta_squared - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta)));
  }
}

// ────────────────────────────────────────────────────────────────────────── //
// Classes

ImuCamPose::ImuCamPose(const KeyFrame* keyframe) : iterations_(0) {
  // Load IMU pose.
  t_wb = keyframe->GetImuPosition().cast<double>();
  R_wb = keyframe->GetImuRotation().cast<double>();

  // Determine number of cameras.
  std::size_t num_cams = 1;
  if (keyframe->mpCamera2) {
    num_cams = 2;
  }

  // Initialize.
  R_cw.resize(num_cams);
  t_cw.resize(num_cams);
  R_cb.resize(num_cams);
  t_cb.resize(num_cams);
  R_bc.resize(num_cams);
  t_bc.resize(num_cams);
  cameras.resize(num_cams);

  // Initialize left camera.
  R_cw[0]    = keyframe->GetRotation().cast<double>();
  t_cw[0]    = keyframe->GetTranslation().cast<double>();
  R_cb[0]    = keyframe->mImuCalib.T_cb.rotationMatrix().cast<double>();
  t_cb[0]    = keyframe->mImuCalib.T_cb.translation().cast<double>();
  R_bc[0]    = R_cb[0].transpose();
  t_bc[0]    = keyframe->mImuCalib.T_bc.translation().cast<double>();
  cameras[0] = keyframe->mpCamera;
  bf         = keyframe->mbf;

  // Initialize right camera.
  if (num_cams > 1) {
    // Retrieve relative pose between left and right cameras.
    const Eigen::Matrix4d T_rl = keyframe->GetRelativePoseTrl().matrix().cast<double>();
    const Eigen::Matrix3d R_rl = T_rl.block<3, 3>(0, 0);
    const Eigen::Vector3d t_rl = T_rl.block<3, 1>(0, 3);

    R_cw[1]    = R_rl * R_cw[0];
    t_cw[1]    = R_rl * t_cw[0] + t_rl;
    R_cb[1]    = R_rl * R_cb[0];
    t_cb[1]    = R_rl * t_cb[0] + t_rl;
    R_bc[1]    = R_cb[1].transpose();
    t_bc[1]    = -R_bc[1] * t_cb[1];
    cameras[1] = keyframe->mpCamera2;
  }

  // Initialize internal variables.
  R0_wb_ = R_wb;
  dR_.setIdentity();
}

ImuCamPose::ImuCamPose(const Frame* frame) : iterations_(0) {
  // Load IMU pose.
  t_wb = frame->GetImuPosition().cast<double>();
  R_wb = frame->GetImuRotation().cast<double>();

  // Determine number of cameras.
  std::size_t num_cams = 1;
  if (frame->mpCamera2) {
    num_cams = 2;
  }

  // Initialize.
  t_cw.resize(num_cams);
  R_cw.resize(num_cams);
  t_cb.resize(num_cams);
  R_cb.resize(num_cams);
  R_bc.resize(num_cams);
  t_bc.resize(num_cams);
  cameras.resize(num_cams);

  // Initialize left camera.
  R_cw[0]    = frame->GetPose().rotationMatrix().cast<double>();
  t_cw[0]    = frame->GetPose().translation().cast<double>();
  R_cb[0]    = frame->mImuCalib.T_cb.rotationMatrix().cast<double>();
  t_cb[0]    = frame->mImuCalib.T_cb.translation().cast<double>();
  R_bc[0]    = R_cb[0].transpose();
  t_bc[0]    = frame->mImuCalib.T_bc.translation().cast<double>();
  cameras[0] = frame->mpCamera;
  bf         = frame->mbf;

  // Initialize right camera.
  if (num_cams > 1) {
    const Eigen::Matrix4d T_rl = frame->GetRelativePoseTrl().matrix().cast<double>();
    const Eigen::Matrix3d R_rl = T_rl.block<3, 3>(0, 0);
    const Eigen::Vector3d t_rl = T_rl.block<3, 1>(0, 3);

    R_cw[1]    = R_rl * R_cw[0];
    t_cw[1]    = R_rl * t_cw[0] + t_rl;
    R_cb[1]    = R_rl * R_cb[0];
    t_cb[1]    = R_rl * t_cb[0] + t_rl;
    R_bc[1]    = R_cb[1].transpose();
    t_bc[1]    = -R_bc[1] * t_cb[1];
    cameras[1] = frame->mpCamera2;
  }

  // Initialize internal variables.
  R0_wb_ = R_wb;
  dR_.setIdentity();
}

ImuCamPose::ImuCamPose(
  const Eigen::Matrix3d& R_wc,
  const Eigen::Vector3d& t_wc,
  const KeyFrame* keyframe
)
  : iterations_(0) {
  // This is only for posegrpah, we do not care about multicamera.
  t_cw.resize(1);
  R_cw.resize(1);
  t_cb.resize(1);
  R_cb.resize(1);
  R_bc.resize(1);
  t_bc.resize(1);
  cameras.resize(1);

  // Initialize left camera.
  R_cb[0]    = keyframe->mImuCalib.T_cb.rotationMatrix().cast<double>();
  t_cb[0]    = keyframe->mImuCalib.T_cb.translation().cast<double>();
  R_bc[0]    = R_cb[0].transpose();
  t_bc[0]    = keyframe->mImuCalib.T_bc.translation().cast<double>();
  R_wb       = R_wc * R_cb[0];
  t_wb       = R_wc * t_cb[0] + t_wc;
  R_cw[0]    = R_wc.transpose();
  t_cw[0]    = -R_cw[0] * t_wc;
  cameras[0] = keyframe->mpCamera;
  bf         = keyframe->mbf;

  // Initialize internal variables for posegraph 4DoF.
  R0_wb_ = R_wb;
  dR_.setIdentity();
}

void ImuCamPose::setParameters(
  const std::vector<Eigen::Matrix3d>& _R_cw,
  const std::vector<Eigen::Vector3d>& _t_cw,
  const std::vector<Eigen::Matrix3d>& _R_bc,
  const std::vector<Eigen::Vector3d>& _t_bc,
  const double _bf
) {
  const std::size_t num_cams = R_bc.size();

  // Get rotations/translations for:
  // - camera to body
  // - world to camera
  R_bc = _R_bc;
  t_bc = _t_bc;
  R_cw = _R_cw;
  t_cw = _t_cw;

  // Calculate rotations/translations from body to camera.
  R_cb.resize(num_cams);
  t_cb.resize(num_cams);
  for (std::size_t i = 0; i < t_cb.size(); i++) {
    R_cb[i] = R_bc[i].transpose();
    t_cb[i] = -R_cb[i] * t_bc[i];
  }

  // Calculate rotations/translations from body to world.
  R_wb = R_cw[0].transpose() * R_cb[0];
  t_wb = R_cw[0].transpose() * (t_cb[0] - t_cw[0]);

  bf = _bf;
}

Eigen::Vector2d ImuCamPose::projectMonocular(
  const Eigen::Vector3d& pt,
  const std::size_t cam_idx
) const {
  // Project 3D point from world to camera frame.
  const Eigen::Vector3d projected = R_cw[cam_idx] * pt + t_cw[cam_idx];

  return cameras[cam_idx]->project(projected.cast<float>()).cast<double>();
}

Eigen::Vector3d ImuCamPose::projectStereo(
  const Eigen::Vector3d& pt,
  const std::size_t cam_idx
) const {
  // Project 3D point from world to camera frame.
  const Eigen::Vector3d projected = R_cw[cam_idx] * pt + t_cw[cam_idx];

  // Continue to project to the image plane and output the 3rd element as
  // x-coordinate in other camera
  const Eigen::Vector2d uv
    = cameras[cam_idx]->project(projected.cast<float>()).cast<double>();

  return Eigen::Vector3d(uv.x(), uv.y(), uv.x() - bf / projected.z());
}

bool ImuCamPose::isDepthPositive(
  const Eigen::Vector3d& pt,
  const std::size_t cam_idx
) const {
  const double depth = R_cw[cam_idx].row(2) * pt + t_cw[cam_idx](2);
  return depth > 0.0;
}

void ImuCamPose::updateInBodyFrame(const double* update) {
  // No update if the pointer is null.
  if (update == nullptr) {
    LOG(WARNING) << "No update for IMU pose.";
    return;
  }

  // The update of the rotation.
  const Eigen::Vector3d ur(update[0], update[1], update[2]);
  // The update of the translation.
  const Eigen::Vector3d ut(update[3], update[4], update[5]);

  // Update body pose.
  t_wb += R_wb * ut;
  R_wb = R_wb * expSO3(ur);

  // TODO: maybe parameterize the frequency of normalization.
  // Normalize rotation after 3 updates.
  iterations_++;
  if (iterations_ >= 3) {
    normalizeRotation(R_wb);
    iterations_ = 0;
  }

  // Update camera poses.
  const Eigen::Matrix3d R_bw = R_wb.transpose();
  const Eigen::Vector3d t_bw = -R_bw * t_wb;
  for (std::size_t i = 0; i < cameras.size(); i++) {
    R_cw[i] = R_cb[i] * R_bw;
    t_cw[i] = R_cb[i] * t_bw + t_cb[i];
  }
}

void ImuCamPose::UpdateInWorldFrame(const double* update) {
  // No update if the pointer is null.
  if (update == nullptr) {
    LOG(WARNING) << "No update for IMU pose.";
    return;
  }

  // The update of the rotation.
  const Eigen::Vector3d ur(update[0], update[1], update[2]);
  // The update of the translation.
  const Eigen::Vector3d ut(update[3], update[4], update[5]);

  // Update body pose.
  dR_  = expSO3(ur) * dR_;
  R_wb = dR_ * R0_wb_;
  t_wb += ut;

  // TODO: maybe parameterize the frequency of normalization.
  // Normalize rotation after 5 updates.
  iterations_++;
  if (iterations_ >= 5) {
    dR_(0, 2) = 0.0;
    dR_(1, 2) = 0.0;
    dR_(2, 0) = 0.0;
    dR_(2, 1) = 0.0;
    normalizeRotation(dR_);
    iterations_ = 0;
  }

  // Update camera poses.
  const Eigen::Matrix3d R_bw = R_wb.transpose();
  const Eigen::Vector3d t_bw = -R_bw * t_wb;
  for (std::size_t i = 0; i < cameras.size(); i++) {
    R_cw[i] = R_cb[i] * R_bw;
    t_cw[i] = R_cb[i] * t_bw + t_cb[i];
  }
}

VertexPose::VertexPose(const KeyFrame* keyframe) {
  setEstimate(ImuCamPose(keyframe));
}

VertexPose::VertexPose(const Frame* frame) {
  setEstimate(ImuCamPose(frame));
}

bool VertexPose::read(std::istream& is) {
  const std::size_t num_cams = _estimate.cameras.size();

  std::vector<Eigen::Matrix3d> R_cw;
  std::vector<Eigen::Vector3d> t_cw;
  std::vector<Eigen::Matrix3d> R_bc;
  std::vector<Eigen::Vector3d> t_bc;

  // Loop over the number of cameras.
  for (std::size_t idx = 0; idx < num_cams; idx++) {
    // Deserialize rotation and translation from world to camera.
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        is >> R_cw[idx](i, j);
      }
    }
    for (std::size_t i = 0; i < 3; i++) {
      is >> t_cw[idx](i);
    }
    // Deserialize rotation and translation from camera to body.
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        is >> R_bc[idx](i, j);
      }
    }
    for (std::size_t i = 0; i < 3; i++) {
      is >> t_bc[idx](i);
    }

    // Deserialize camera parameters.
    float next_param;
    for (std::size_t i = 0; i < _estimate.cameras[idx]->getNumParams(); i++) {
      is >> next_param;
      _estimate.cameras[idx]->setParameter(next_param, i);
    }
  }

  // Deserialize bf.
  double bf;
  is >> bf;

  // Set parameters for the estimate.
  _estimate.setParameters(R_cw, t_cw, R_bc, t_bc, bf);
  updateCache();

  return true;
}

bool VertexPose::write(std::ostream& os) const {
  const std::size_t num_cams = _estimate.cameras.size();

  const std::vector<Eigen::Matrix3d>& R_cw = _estimate.R_cw;
  const std::vector<Eigen::Vector3d>& t_cw = _estimate.t_cw;
  const std::vector<Eigen::Matrix3d>& R_bc = _estimate.R_bc;
  const std::vector<Eigen::Vector3d>& t_bc = _estimate.t_bc;

  // Loop over the number of cameras.
  for (std::size_t idx = 0; idx < num_cams; idx++) {
    // Serialize rotation and translation from world to camera.
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        os << R_cw[idx](i, j) << " ";
      }
    }
    for (std::size_t i = 0; i < 3; i++) {
      os << t_cw[idx](i) << " ";
    }
    // Serialize rotation and translation from camera to body.
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        os << R_bc[idx](i, j) << " ";
      }
    }
    for (std::size_t i = 0; i < 3; i++) {
      os << t_bc[idx](i) << " ";
    }

    // Serialize camera parameters.
    for (std::size_t i = 0; i < _estimate.cameras[idx]->getNumParams(); i++) {
      os << _estimate.cameras[idx]->getParameter(i) << " ";
    }
  }

  // Serialize bf.
  os << _estimate.bf << " ";

  return os.good();
}

VertexPose4DoF::VertexPose4DoF(const KeyFrame* keyframe) {
  setEstimate(ImuCamPose(keyframe));
}

VertexPose4DoF::VertexPose4DoF(const Frame* frame) {
  setEstimate(ImuCamPose(frame));
}

VertexPose4DoF::VertexPose4DoF(
  const Eigen::Matrix3d& R_wc,
  const Eigen::Vector3d& t_wc,
  const KeyFrame* keyframe
) {
  setEstimate(ImuCamPose(R_wc, t_wc, keyframe));
}

VertexVelocity::VertexVelocity(const KeyFrame* keyframe) {
  setEstimate(keyframe->GetVelocity().cast<double>());
}

VertexVelocity::VertexVelocity(const Frame* frame) {
  setEstimate(frame->GetVelocity().cast<double>());
}

VertexGyroBias::VertexGyroBias(const KeyFrame* keyframe) {
  setEstimate(keyframe->GetGyroBias().cast<double>());
}

VertexGyroBias::VertexGyroBias(const Frame* frame) {
  const Eigen::Vector3d bias(frame->mImuBias.gyro().cast<double>());
  setEstimate(bias);
}

VertexAccBias::VertexAccBias(const KeyFrame* keyframe) {
  setEstimate(keyframe->GetAccBias().cast<double>());
}

VertexAccBias::VertexAccBias(const Frame* frame) {
  setEstimate(frame->mImuBias.acc().cast<double>());
}

VertexGravityDirection::VertexGravityDirection(const Eigen::Matrix3d& R_wg) {
  setEstimate(GravityDirection(R_wg));
}

VertexScale::VertexScale() {
  setEstimate(1.0);
}

VertexScale::VertexScale(const double scale) {
  setEstimate(scale);
}

InverseDepthPoint::InverseDepthPoint(
  const double _rho,
  const double _u,
  const double _v,
  const KeyFrame* host_keyframe
)
  : rho(_rho)
  , u(_u)
  , v(_v)
  , fx(host_keyframe->fx)
  , fy(host_keyframe->fy)
  , cx(host_keyframe->cx)
  , cy(host_keyframe->cy)
  , bf(host_keyframe->mbf)
{}

void InverseDepthPoint::update(const double* update) {
  rho += *update;
}

VertexInverseDepthPoint::VertexInverseDepthPoint(
  const double rho,
  const double u,
  const double v,
  const KeyFrame* keyframe
) {
  setEstimate(InverseDepthPoint(rho, u, v, keyframe));
}

EdgeMono::EdgeMono(const std::size_t cam_idx) : cam_idx_(cam_idx) {
}

void EdgeMono::linearizeOplus() {
  // Retrieve pointers to the vertices.
  const g2o::VertexSBAPointXYZ* vpoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[1]);

  // Transform the point from world to camera frame, then to body frame.
  const Eigen::Matrix3d& R_cw = vpose->estimate().R_cw[cam_idx_];
  const Eigen::Vector3d& t_cw = vpose->estimate().t_cw[cam_idx_];
  const Eigen::Vector3d x_c   = R_cw * vpoint->estimate() + t_cw;

  const Eigen::Matrix3d& R_bc = vpose->estimate().R_bc[cam_idx_];
  const Eigen::Vector3d& t_bc = vpose->estimate().t_bc[cam_idx_];
  const Eigen::Vector3d x_b   = R_bc * x_c + t_bc;

  const Eigen::Matrix3d& R_cb = vpose->estimate().R_cb[cam_idx_];

  // Compute the camera projection Jacobian and update the Jacobians.
  const Eigen::Matrix<double, 2, 3> proj_jac
    = vpose->estimate().cameras[cam_idx_]->jacobian(x_c.cast<float>()).cast<double>();

  _jacobianOplusXi = -proj_jac * R_cw;

  // Compute the derivative of the point in the body frame and update the
  // Jacobians.
  Eigen::Matrix<double, 3, 6> derivation_se3;
  const double x = x_b.x();
  const double y = x_b.y();
  const double z = x_b.z();

  derivation_se3 << 0.0,   z,  -y, 1.0, 0.0, 0.0,
                     -z, 0.0,   x, 0.0, 1.0, 0.0,
                      y,  -x, 0.0, 0.0, 0.0, 1.0;

  _jacobianOplusXj = proj_jac * R_cb * derivation_se3; // TODO: optimize this product
}

void EdgeMono::computeError() {
  const g2o::VertexSBAPointXYZ* vpoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[1]);
  const Eigen::Vector2d obs(_measurement);
  _error = obs - vpose->estimate().projectMonocular(vpoint->estimate(), cam_idx_);
}

bool EdgeMono::isDepthPositive() {
  const g2o::VertexSBAPointXYZ* vpoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[1]);
  return vpose->estimate().isDepthPositive(vpoint->estimate(), cam_idx_);
}

Matrix9d EdgeMono::getHessian() {
  linearizeOplus();
  Eigen::Matrix<double, 2, 9> J;
  J.block<2, 3>(0, 0) = _jacobianOplusXi;
  J.block<2, 6>(0, 3) = _jacobianOplusXj;
  return J.transpose() * information() * J;
}

EdgeMonoOnlyPose::EdgeMonoOnlyPose(
  const Eigen::Vector3f& x_w,
  const std::size_t cam_idx
)
  : x_w_(x_w.cast<double>())
  , cam_idx_(cam_idx)
{}

void EdgeMonoOnlyPose::linearizeOplus() {
  // Retrieve pointers to the vertices.
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[0]);

  // Transform the point from world to camera frame, then to body frame.
  const Eigen::Matrix3d& R_cw = vpose->estimate().R_cw[cam_idx_];
  const Eigen::Vector3d& t_cw = vpose->estimate().t_cw[cam_idx_];
  const Eigen::Vector3d x_c   = R_cw * x_w_ + t_cw;

  const Eigen::Matrix3d& R_bc = vpose->estimate().R_bc[cam_idx_];
  const Eigen::Vector3d& t_bc = vpose->estimate().t_bc[cam_idx_];
  const Eigen::Vector3d x_b   = R_bc * x_c + t_bc;

  const Eigen::Matrix3d& R_cb = vpose->estimate().R_cb[cam_idx_];

  // Compute the camera projection Jacobian, SE3 derivation in body frame and
  // update the Jacobians.
  const Eigen::Matrix<double, 2, 3> proj_jac
    = vpose->estimate().cameras[cam_idx_]->jacobian(x_c.cast<float>()).cast<double>();

  Eigen::Matrix<double, 3, 6> derivation_se3;
  const double x = x_b.x();
  const double y = x_b.y();
  const double z = x_b.z();
  derivation_se3 << 0.0,   z,  -y, 1.0, 0.0, 0.0,
                     -z, 0.0,   x, 0.0, 1.0, 0.0,
                      y,  -x, 0.0, 0.0, 0.0, 1.0;

  _jacobianOplusXi = proj_jac * R_cb * derivation_se3; // symbol different because of update mode
}

void EdgeMonoOnlyPose::computeError() {
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[0]);
  const Eigen::Vector2d obs(_measurement);
  _error = obs - vpose->estimate().projectMonocular(x_w_, cam_idx_);
}

bool EdgeMonoOnlyPose::isDepthPositive() {
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[0]);
  return vpose->estimate().isDepthPositive(x_w_, cam_idx_);
}

Matrix6d EdgeMonoOnlyPose::getHessian() {
  linearizeOplus();
  return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
}

EdgeStereo::EdgeStereo(const std::size_t cam_idx) : cam_idx_(cam_idx) {}

void EdgeStereo::linearizeOplus() {
  // Retrieve pointers to the vertices.
  const g2o::VertexSBAPointXYZ* vpoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[1]);

  // Transform the point from world to camera frame, then to body frame.
  const Eigen::Matrix3d& R_cw = vpose->estimate().R_cw[cam_idx_];
  const Eigen::Vector3d& t_cw = vpose->estimate().t_cw[cam_idx_];
  const Eigen::Vector3d x_c   = R_cw * vpoint->estimate() + t_cw;

  const Eigen::Matrix3d& R_bc = vpose->estimate().R_bc[cam_idx_];
  const Eigen::Vector3d& t_bc = vpose->estimate().t_bc[cam_idx_];
  const Eigen::Vector3d x_b   = R_bc * x_c + t_bc;

  const Eigen::Matrix3d& R_cb = vpose->estimate().R_cb[cam_idx_];
  const double bf             = vpose->estimate().bf;

  // Compute the camera projection Jacobian and update the Jacobians.
  Eigen::Matrix3d proj_jac;
  proj_jac.block<2, 3>(0, 0)
    = vpose->estimate().cameras[cam_idx_]->jacobian(x_c.cast<float>()).cast<double>();
  proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
  proj_jac(2, 2) += bf / (x_c.z() * x_c.x());

  _jacobianOplusXi = -proj_jac * R_cw;

  // Compute the derivative of the point in the body frame and update the
  // Jacobians.
  Eigen::Matrix<double, 3, 6> derivation_se3;
  const double x = x_b.x();
  const double y = x_b.y();
  const double z = x_b.z();
  derivation_se3 << 0.0,   z,  -y, 1.0, 0.0, 0.0,
                     -z, 0.0,   x, 0.0, 1.0, 0.0,
                      y,  -x, 0.0, 0.0, 0.0, 1.0;

  _jacobianOplusXj = proj_jac * R_cb * derivation_se3;
}

void EdgeStereo::computeError() {
  const g2o::VertexSBAPointXYZ* vpoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[1]);
  const Eigen::Vector3d obs(_measurement);
  _error = obs - vpose->estimate().projectStereo(vpoint->estimate(), cam_idx_);
}

Matrix9d EdgeStereo::getHessian() {
  linearizeOplus();
  Eigen::Matrix<double, 3, 9> J;
  J.block<3, 3>(0, 0) = _jacobianOplusXi;
  J.block<3, 6>(0, 3) = _jacobianOplusXj;
  return J.transpose() * information() * J;
}

EdgeStereoOnlyPose::EdgeStereoOnlyPose(
  const Eigen::Vector3f& x_w,
  const std::size_t cam_idx
)
  : x_w_(x_w.cast<double>()),
  cam_idx_(cam_idx)
{}

void EdgeStereoOnlyPose::linearizeOplus() {
  // Retrieve pointers to the vertices.
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[0]);

  // Transform the point from world to camera frame, then to body frame.
  const Eigen::Matrix3d& R_cw = vpose->estimate().R_cw[cam_idx_];
  const Eigen::Vector3d& t_cw = vpose->estimate().t_cw[cam_idx_];
  const Eigen::Vector3d x_c   = R_cw * x_w_ + t_cw;

  const Eigen::Matrix3d& R_bc = vpose->estimate().R_bc[cam_idx_];
  const Eigen::Vector3d& t_bc = vpose->estimate().t_bc[cam_idx_];
  const Eigen::Vector3d x_b   = R_bc * x_c + t_bc;

  const Eigen::Matrix3d& R_cb = vpose->estimate().R_cb[cam_idx_];
  const double bf             = vpose->estimate().bf;

  // Compute the camera projection Jacobian, SE3 derivation in body frame and
  // update the Jacobians.
  Eigen::Matrix3d proj_jac;
  proj_jac.block<2, 3>(0, 0)
    = vpose->estimate().cameras[cam_idx_]->jacobian(x_c.cast<float>()).cast<double>();
  proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
  proj_jac(2, 2) += bf / (x_c.z() * x_c.z());

  Eigen::Matrix<double, 3, 6> derivation_se3;
  const double x = x_b.x();
  const double y = x_b.y();
  const double z = x_b.z();
  derivation_se3 << 0.0,   z,  -y, 1.0, 0.0, 0.0,
                     -z, 0.0,   x, 0.0, 1.0, 0.0,
                      y,  -x, 0.0, 0.0, 0.0, 1.0;

  _jacobianOplusXi = proj_jac * R_cb * derivation_se3;
}

void EdgeStereoOnlyPose::computeError() {
  const VertexPose* vpose = static_cast<const VertexPose*>(_vertices[0]);
  const Eigen::Vector3d obs(_measurement);
  _error = obs - vpose->estimate().projectStereo(x_w_, cam_idx_);
}

Matrix6d EdgeStereoOnlyPose::getHessian() {
  linearizeOplus();
  return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
}

EdgeInertial::EdgeInertial(IMU::Preintegrated* preintegrated)
  : JR_gyro_(preintegrated->JR_gyro.cast<double>())
  , JV_gyro_(preintegrated->JV_gyro.cast<double>())
  , JP_gyro_(preintegrated->JP_gyro.cast<double>())
  , JV_acc_(preintegrated->JV_acc.cast<double>())
  , JP_acc_(preintegrated->JP_acc.cast<double>())
  , preintegrated_(preintegrated)
  , dt_(preintegrated->t)
{
  // This edge links 6 vertices.
  resize(6);

  // Initialize gravity vector.
  g_ << 0.0, 0.0, -IMU::kGravity;

  // Construct the information matrix.
  Matrix9d info = preintegrated->C.block<9, 9>(0, 0).cast<double>().inverse();
  info = (info + info.transpose()) / 2.0;
  Eigen::SelfAdjointEigenSolver<Matrix9d> solver(info);
  Vector9d eigens = solver.eigenvalues();
  for (std::size_t i = 0; i < 9; i++) {
    if (eigens[i] < 1e-12) {
      eigens[i] = 0.0;
    }
  }
  info = solver.eigenvectors() * eigens.asDiagonal() * solver.eigenvectors().transpose();
  setInformation(info);
}

void EdgeInertial::computeError() {
  // TODO: Maybe reintegrate inertial measurments when difference between
  // linearization point and current estimate is too big.

  // Retrieve pointers to the vertices, bias and delta bias of 1st frame.
  const VertexPose*         vpose_1 = static_cast<const VertexPose*    >(_vertices[0]);
  const VertexVelocity* vvelocity_1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias*  vbias_gyro = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias*    vbias_acc = static_cast<const VertexAccBias* >(_vertices[3]);
  const VertexPose*         vpose_2 = static_cast<const VertexPose*    >(_vertices[4]);
  const VertexVelocity* vvelocity_2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const IMU::Bias bias(
    vbias_acc ->estimate().x(),
    vbias_acc ->estimate().y(),
    vbias_acc ->estimate().z(),
    vbias_gyro->estimate().x(),
    vbias_gyro->estimate().y(),
    vbias_gyro->estimate().z()
  );

  // Retrieve the estimates and calculate the error.
  const Eigen::Matrix3d dR = preintegrated_->getDeltaRotation(bias).cast<double>();
  const Eigen::Vector3d dV = preintegrated_->getDeltaVelocity(bias).cast<double>();
  const Eigen::Vector3d dP = preintegrated_->getDeltaPosition(bias).cast<double>();

  const Eigen::Matrix3d R_wb_1 = vpose_1->estimate().R_wb;
  const Eigen::Vector3d t_wb_1 = vpose_1->estimate().t_wb;
  const Eigen::Matrix3d R_wb_2 = vpose_2->estimate().R_wb;
  const Eigen::Vector3d t_wb_2 = vpose_2->estimate().t_wb;

  const Eigen::Matrix3d R_bw_1 = R_wb_1.transpose();

  const Eigen::Vector3d v_1 = vvelocity_1->estimate();
  const Eigen::Vector3d v_2 = vvelocity_2->estimate();

  const Eigen::Vector3d er = logSO3(dR.transpose() * R_bw_1 * R_wb_2);
  const Eigen::Vector3d ev = R_bw_1 * (v_2 - v_1 - g_ * dt_) - dV;
  const Eigen::Vector3d ep = R_bw_1 * (t_wb_2 - t_wb_1 - v_1 * dt_ - g_ * dt_ * dt_ / 2.0) - dP;

  _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus() {
  // Retrieve pointers to the vertices, bias and delta bias of 1st frame.
  const VertexPose*         vpose_1 = static_cast<const VertexPose*    >(_vertices[0]);
  const VertexVelocity* vvelocity_1 = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias*  vbias_gyro = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias*    vbias_acc = static_cast<const VertexAccBias* >(_vertices[3]);
  const VertexPose*         vpose_2 = static_cast<const VertexPose*    >(_vertices[4]);
  const VertexVelocity* vvelocity_2 = static_cast<const VertexVelocity*>(_vertices[5]);
  const IMU::Bias bias(
    vbias_acc ->estimate().x(),
    vbias_acc ->estimate().y(),
    vbias_acc ->estimate().z(),
    vbias_gyro->estimate().x(),
    vbias_gyro->estimate().y(),
    vbias_gyro->estimate().z()
  );
  const Eigen::Vector3d dbias_gyro = preintegrated_->getDeltaBias(bias).gyro().cast<double>();

  // Retrieve the estimates and calculate the error.
  const Eigen::Matrix3d R_wb_1 = vpose_1->estimate().R_wb;
  const Eigen::Vector3d t_wb_1 = vpose_1->estimate().t_wb;
  const Eigen::Matrix3d R_wb_2 = vpose_2->estimate().R_wb;
  const Eigen::Vector3d t_wb_2 = vpose_2->estimate().t_wb;

  const Eigen::Matrix3d R_bw_1 = R_wb_1.transpose();
  const Eigen::Matrix3d R_bw_2 = R_wb_2.transpose();

  const Eigen::Vector3d v_1 = vvelocity_1->estimate();
  const Eigen::Vector3d v_2 = vvelocity_2->estimate();

  const Eigen::Matrix3d dR     = preintegrated_->getDeltaRotation(bias).cast<double>();
  const Eigen::Matrix3d eR     = dR.transpose() * R_bw_1 * R_wb_2;
  const Eigen::Vector3d er     = logSO3(eR);
  const Eigen::Matrix3d Jr_inv = inverseRightJacobianSO3(er);

  // Jacobians wrt Pose 1
  _jacobianOplus[0].setZero();
  // rotation
  _jacobianOplus[0].block<3, 3>(0, 0) = -Jr_inv * R_bw_2 * R_wb_1;
  _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(R_bw_1 * (v_2 - v_1 - g_ * dt_));
  _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(
    R_bw_1 * (t_wb_2 - t_wb_1 - v_1 * dt_ - 0.5 * g_ * dt_ * dt_)
  );
  // translation
  _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();

  // Jacobians wrt Velocity 1
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(3, 0) = -R_bw_1;
  _jacobianOplus[1].block<3, 3>(6, 0) = -R_bw_1 * dt_;

  // Jacobians wrt Gyro 1
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(0, 0)
    = -Jr_inv * eR.transpose() * rightJacobianSO3(JR_gyro_ * dbias_gyro) * JR_gyro_;
  _jacobianOplus[2].block<3, 3>(3, 0) = -JV_gyro_;
  _jacobianOplus[2].block<3, 3>(6, 0) = -JP_gyro_;

  // Jacobians wrt Accelerometer 1
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(3, 0) = -JV_acc_;
  _jacobianOplus[3].block<3, 3>(6, 0) = -JP_acc_;

  // Jacobians wrt Pose 2
  _jacobianOplus[4].setZero();
  // rotation
  _jacobianOplus[4].block<3, 3>(0, 0) = Jr_inv;
  // translation
  _jacobianOplus[4].block<3, 3>(6, 3) = R_bw_1 * R_wb_2;

  // Jacobians wrt Velocity 2
  _jacobianOplus[5].setZero();
  _jacobianOplus[5].block<3, 3>(3, 0) = R_bw_1;
}

Matrix24d EdgeInertial::getHessian() {
  linearizeOplus();
  Eigen::Matrix<double, 9, 24> J;
  J.block<9, 6>(0, 0)  = _jacobianOplus[0];
  J.block<9, 3>(0, 6)  = _jacobianOplus[1];
  J.block<9, 3>(0, 9)  = _jacobianOplus[2];
  J.block<9, 3>(0, 12) = _jacobianOplus[3];
  J.block<9, 6>(0, 15) = _jacobianOplus[4];
  J.block<9, 3>(0, 21) = _jacobianOplus[5];
  return J.transpose() * information() * J;
}

Matrix9d EdgeInertial::getHessian2() {
  linearizeOplus();
  Matrix9d J;
  J.block<9, 6>(0, 0) = _jacobianOplus[4];
  J.block<9, 3>(0, 6) = _jacobianOplus[5];
  return J.transpose() * information() * J;
}

EdgeInertialGS::EdgeInertialGS(IMU::Preintegrated* preintegrated)
  : JR_gyro_(preintegrated->JR_gyro.cast<double>())
  , JV_gyro_(preintegrated->JV_gyro.cast<double>())
  , JP_gyro_(preintegrated->JP_gyro.cast<double>())
  , JV_acc_(preintegrated->JV_acc.cast<double>())
  , JP_acc_(preintegrated->JP_acc.cast<double>())
  , preintegrated_(preintegrated)
  , dt(preintegrated->t)
{
  // This edge links 8 vertices.
  resize(8);

  // Initialize gravity vector.
  g0_ << 0.0, 0.0, -IMU::kGravity;

  // Construct the information matrix.
  Matrix9d info = preintegrated->C.block<9, 9>(0, 0).cast<double>().inverse();
  info = (info + info.transpose()) / 2.0;
  Eigen::SelfAdjointEigenSolver<Matrix9d> solver(info);
  Vector9d eigens = solver.eigenvalues();
  for (std::size_t i = 0; i < 9; i++){
    if (eigens[i] < 1e-12) {
      eigens[i] = 0.0;
    }
  }
  info = solver.eigenvectors() * eigens.asDiagonal() * solver.eigenvectors().transpose();
  setInformation(info);
}

void EdgeInertialGS::computeError() {
  // TODO: Maybe Reintegrate inertial measurments when difference between
  // linearization point and current estimate is too big.

  // Retrieve pointers to the vertices.
  const VertexPose*              vpose_1 = static_cast<const VertexPose*            >(_vertices[0]);
  const VertexVelocity*      vvelocity_1 = static_cast<const VertexVelocity*        >(_vertices[1]);
  const VertexGyroBias*       vbias_gyro = static_cast<const VertexGyroBias*        >(_vertices[2]);
  const VertexAccBias*         vbias_acc = static_cast<const VertexAccBias*         >(_vertices[3]);
  const VertexPose*              vpose_2 = static_cast<const VertexPose*            >(_vertices[4]);
  const VertexVelocity*      vvelocity_2 = static_cast<const VertexVelocity*        >(_vertices[5]);
  const VertexGravityDirection* vgravity = static_cast<const VertexGravityDirection*>(_vertices[6]);
  const VertexScale*              vscale = static_cast<const VertexScale*           >(_vertices[7]);
  const IMU::Bias bias(
    vbias_acc ->estimate().x(),
    vbias_acc ->estimate().y(),
    vbias_acc ->estimate().z(),
    vbias_gyro->estimate().x(),
    vbias_gyro->estimate().y(),
    vbias_gyro->estimate().z()
  );

  // Retrieve the estimates and calculate the error.
  g_ = vgravity->estimate().R_wg * g0_;
  const double scale = vscale->estimate();
  const Eigen::Matrix3d dR = preintegrated_->getDeltaRotation(bias).cast<double>();
  const Eigen::Vector3d dV = preintegrated_->getDeltaVelocity(bias).cast<double>();
  const Eigen::Vector3d dP = preintegrated_->getDeltaPosition(bias).cast<double>();

  const Eigen::Matrix3d R_wb_1 = vpose_1->estimate().R_wb;
  const Eigen::Vector3d t_wb_1 = vpose_1->estimate().t_wb;
  const Eigen::Matrix3d R_wb_2 = vpose_2->estimate().R_wb;
  const Eigen::Vector3d t_wb_2 = vpose_2->estimate().t_wb;

  const Eigen::Matrix3d R_bw_1 = R_wb_1.transpose();

  const Eigen::Vector3d v_1 = vvelocity_1->estimate();
  const Eigen::Vector3d v_2 = vvelocity_2->estimate();

  const Eigen::Vector3d er = logSO3(dR.transpose() * R_bw_1 * R_wb_2);
  const Eigen::Vector3d ev = R_bw_1 * (scale * (v_2 - v_1) - g_ * dt) - dV;
  const Eigen::Vector3d ep
    = R_bw_1 * (scale * (t_wb_2 - t_wb_1 - v_1 * dt) - g_ * dt * dt / 2.0) - dP;

  _error << er, ev, ep;
}

void EdgeInertialGS::linearizeOplus() {
  // Retrieve pointers to the vertices.
  const VertexPose*              vpose_1 = static_cast<const VertexPose*            >(_vertices[0]);
  const VertexVelocity*      vvelocity_1 = static_cast<const VertexVelocity*        >(_vertices[1]);
  const VertexGyroBias*       vbias_gyro = static_cast<const VertexGyroBias*        >(_vertices[2]);
  const VertexAccBias*         vbias_acc = static_cast<const VertexAccBias*         >(_vertices[3]);
  const VertexPose*              vpose_2 = static_cast<const VertexPose*            >(_vertices[4]);
  const VertexVelocity*      vvelocity_2 = static_cast<const VertexVelocity*        >(_vertices[5]);
  const VertexGravityDirection* vgravity = static_cast<const VertexGravityDirection*>(_vertices[6]);
  const VertexScale*              vscale = static_cast<const VertexScale*           >(_vertices[7]);

  // Retrieve the estimates and calculate the Jacobians.
  const IMU::Bias bias(
    vbias_acc ->estimate().x(),
    vbias_acc ->estimate().y(),
    vbias_acc ->estimate().z(),
    vbias_gyro->estimate().x(),
    vbias_gyro->estimate().y(),
    vbias_gyro->estimate().z()
  );
  const Eigen::Vector3d dbias_gyro
    = preintegrated_->getDeltaBias(bias).gyro().cast<double>();

  const Eigen::Matrix3d R_wb_1 = vpose_1->estimate().R_wb;
  const Eigen::Vector3d t_wb_1 = vpose_1->estimate().t_wb;
  const Eigen::Matrix3d R_wb_2 = vpose_2->estimate().R_wb;
  const Eigen::Vector3d t_wb_2 = vpose_2->estimate().t_wb;
  const Eigen::Matrix3d R_wg   = vgravity->estimate().R_wg;

  const Eigen::Matrix3d R_bw_1 = R_wb_1.transpose();
  const Eigen::Matrix3d R_bw_2 = R_wb_2.transpose();

  const Eigen::Vector3d v_1 = vvelocity_1->estimate();
  const Eigen::Vector3d v_2 = vvelocity_2->estimate();

  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(3, 2);
  G(0, 1)           = -IMU::kGravity;
  G(1, 0)           =  IMU::kGravity;

  const Eigen::MatrixXd dGdTheta = R_wg * G;
  const Eigen::Matrix3d dR = preintegrated_->getDeltaRotation(bias).cast<double>();
  const Eigen::Matrix3d eR = dR.transpose() * R_bw_1 * R_wb_2;
  const Eigen::Vector3d er = logSO3(eR);
  const Eigen::Matrix3d Jr_inv = inverseRightJacobianSO3(er);

  const double scale = vscale->estimate();

  // Jacobians wrt Pose 1
  _jacobianOplus[0].setZero();
  // rotation
  _jacobianOplus[0].block<3, 3>(0, 0) = -Jr_inv * R_bw_2 * R_wb_1;
  _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(R_bw_1 * (scale * (v_2 - v_1) - g_ * dt));
  _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(
    R_bw_1 * (scale * (t_wb_2 - t_wb_1 - v_1 * dt) - 0.5 * g_ * dt * dt)
  );
  // translation
  _jacobianOplus[0].block<3, 3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(-scale, -scale, -scale);

  // Jacobians wrt Velocity 1
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(3, 0) = -scale * R_bw_1;
  _jacobianOplus[1].block<3, 3>(6, 0) = -scale * R_bw_1 * dt;

  // Jacobians wrt Gyro bias
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(0, 0)
    = -Jr_inv * eR.transpose() * rightJacobianSO3(JR_gyro_ * dbias_gyro) * JR_gyro_;
  _jacobianOplus[2].block<3, 3>(3, 0) = -JV_gyro_;
  _jacobianOplus[2].block<3, 3>(6, 0) = -JP_gyro_;

  // Jacobians wrt Accelerometer bias
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(3, 0) = -JV_acc_;
  _jacobianOplus[3].block<3, 3>(6, 0) = -JP_acc_;

  // Jacobians wrt Pose 2
  _jacobianOplus[4].setZero();
  // rotation
  _jacobianOplus[4].block<3, 3>(0, 0) = Jr_inv;
  // translation
  _jacobianOplus[4].block<3, 3>(6, 3) = scale * R_bw_1 * R_wb_2;

  // Jacobians wrt Velocity 2
  _jacobianOplus[5].setZero();
  _jacobianOplus[5].block<3, 3>(3, 0) = scale * R_bw_1;

  // Jacobians wrt Gravity direction
  _jacobianOplus[6].setZero();
  _jacobianOplus[6].block<3, 2>(3, 0) = -R_bw_1 * dGdTheta * dt;
  _jacobianOplus[6].block<3, 2>(6, 0) = -0.5 * R_bw_1 * dGdTheta * dt * dt;

  // Jacobians wrt scale factor
  _jacobianOplus[7].setZero();
  _jacobianOplus[7].block<3, 1>(3, 0) = R_bw_1 * (v_2 - v_1);
  _jacobianOplus[7].block<3, 1>(6, 0) = R_bw_1 * (t_wb_2 - t_wb_1 - v_1 * dt);
}

void EdgeGyroRW::linearizeOplus() {
  _jacobianOplusXi = -Eigen::Matrix3d::Identity();
  _jacobianOplusXj.setIdentity();
}

void EdgeGyroRW::computeError() {
  const VertexGyroBias* vbias_gyro_1 = static_cast<const VertexGyroBias*>(_vertices[0]);
  const VertexGyroBias* vbias_gyro_2 = static_cast<const VertexGyroBias*>(_vertices[1]);
  _error = vbias_gyro_2->estimate() - vbias_gyro_1->estimate();
}

Matrix6d EdgeGyroRW::getHessian() {
  linearizeOplus();
  Eigen::Matrix<double, 3, 6> J;
  J.block<3, 3>(0, 0) = _jacobianOplusXi;
  J.block<3, 3>(0, 3) = _jacobianOplusXj;
  return J.transpose() * information() * J;
}

Eigen::Matrix3d EdgeGyroRW::getHessian2() {
  linearizeOplus();
  return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
}

void EdgeAccRW::linearizeOplus() {
  _jacobianOplusXi = -Eigen::Matrix3d::Identity();
  _jacobianOplusXj.setIdentity();
}

void EdgeAccRW::computeError() {
  const VertexAccBias* vbias_acc_1 = static_cast<const VertexAccBias*>(_vertices[0]);
  const VertexAccBias* vbias_acc_2 = static_cast<const VertexAccBias*>(_vertices[1]);
  _error = vbias_acc_2->estimate() - vbias_acc_1->estimate();
}

Matrix6d EdgeAccRW::getHessian() {
  linearizeOplus();
  Eigen::Matrix<double, 3, 6> J;
  J.block<3, 3>(0, 0) = _jacobianOplusXi;
  J.block<3, 3>(0, 3) = _jacobianOplusXj;
  return J.transpose() * information() * J;
}

Eigen::Matrix3d EdgeAccRW::getHessian2() {
  linearizeOplus();
  return _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
}

ConstraintPoseImu::ConstraintPoseImu(
  const Eigen::Matrix3d& R_wb,
  const Eigen::Vector3d& t_wb,
  const Eigen::Vector3d& v_wb,
  const Eigen::Vector3d& bias_gyro,
  const Eigen::Vector3d& bias_acc,
  const Matrix15d& H
)
  : R_wb_(R_wb)
  , t_wb_(t_wb)
  , v_wb_(v_wb)
  , bias_gyro_(bias_gyro)
  , bias_acc_(bias_acc)
  , H_(solveHessian(H))
{}

Matrix15d ConstraintPoseImu::solveHessian(Matrix15d H) {
  H = (H + H) / 2.0;
  Eigen::SelfAdjointEigenSolver<Matrix15d> solver(H);
  Vector15d eigens = solver.eigenvalues();
  for (std::size_t i = 0; i < 15; i++) {
    if (eigens[i] < 1e-12) {
      eigens[i] = 0.0;
    }
  }
  H = solver.eigenvectors() * eigens.asDiagonal() * solver.eigenvectors().transpose();
}

EdgePriorPoseImu::EdgePriorPoseImu(const ConstraintPoseImu* constraint) {
  // This edge links 4 vertices.
  resize(4);

  R_wb_      = constraint->R_wb_;
  t_wb_      = constraint->t_wb_;
  v_wb_      = constraint->v_wb_;
  bias_gyro_ = constraint->bias_gyro_;
  bias_acc_  = constraint->bias_acc_;
  setInformation(constraint->H_);
}

void EdgePriorPoseImu::computeError() {
  // Retrieve pointers to the vertices.
  const VertexPose*          vpose = static_cast<const VertexPose*    >(_vertices[0]);
  const VertexVelocity*  vvelocity = static_cast<const VertexVelocity*>(_vertices[1]);
  const VertexGyroBias* vbias_gyro = static_cast<const VertexGyroBias*>(_vertices[2]);
  const VertexAccBias*   vbias_acc = static_cast<const VertexAccBias* >(_vertices[3]);

  // Calculate the error.
  const Eigen::Vector3d er  = logSO3(R_wb_.transpose() * vpose->estimate().R_wb);
  const Eigen::Vector3d et  = R_wb_.transpose() * (vpose->estimate().t_wb - t_wb_);
  const Eigen::Vector3d ev  = vvelocity->estimate() - v_wb_;
  const Eigen::Vector3d ebias_gyro = vbias_gyro->estimate() - bias_gyro_;
  const Eigen::Vector3d ebias_acc  = vbias_acc ->estimate() - bias_acc_ ;

  _error << er, et, ev, ebias_gyro, ebias_acc;
}

void EdgePriorPoseImu::linearizeOplus() {
  const VertexPose* vpose  = static_cast<const VertexPose*>(_vertices[0]);
  const Eigen::Vector3d er = logSO3(R_wb_.transpose() * vpose->estimate().R_wb);
  _jacobianOplus[0].setZero();
  _jacobianOplus[0].block<3, 3>(0, 0) = inverseRightJacobianSO3(er);
  _jacobianOplus[0].block<3, 3>(3, 3) = R_wb_.transpose() * vpose->estimate().R_wb;
  _jacobianOplus[1].setZero();
  _jacobianOplus[1].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
  _jacobianOplus[2].setZero();
  _jacobianOplus[2].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
  _jacobianOplus[3].setZero();
  _jacobianOplus[3].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
}

Matrix15d EdgePriorPoseImu::getHessian() {
  linearizeOplus();
  Matrix15d J;
  J.block<15, 6>(0, 0)  = _jacobianOplus[0];
  J.block<15, 3>(0, 6)  = _jacobianOplus[1];
  J.block<15, 3>(0, 9)  = _jacobianOplus[2];
  J.block<15, 3>(0, 12) = _jacobianOplus[3];
  return J.transpose() * information() * J;
}

void EdgePriorAcc::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

}

void EdgePriorGyro::linearizeOplus()
{
    // Jacobian wrt bias
    _jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

}

} // namespace ORB_SLAM3
