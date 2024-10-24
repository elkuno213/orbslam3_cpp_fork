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

InvDepthPoint::InvDepthPoint(
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

void InvDepthPoint::update(const double* update) {
  rho += *update;
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

void EdgeMono::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().R_cw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().t_cw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().R_bc[cam_idx]*Xc+VPose->estimate().t_bc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().R_cb[cam_idx];

    const Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().cameras[cam_idx]->jacobian(Xc.cast<float>()).cast<double>();
    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv; // TODO optimize this product
}

void EdgeMonoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().R_cw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().t_cw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().R_bc[cam_idx]*Xc+VPose->estimate().t_bc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().R_cb[cam_idx];

    Eigen::Matrix<double,2,3> proj_jac = VPose->estimate().cameras[cam_idx]->jacobian(Xc.cast<float>()).cast<double>();

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv; // symbol different becasue of update mode
}

void EdgeStereo::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().R_cw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().t_cw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().R_bc[cam_idx]*Xc+VPose->estimate().t_bc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().R_cb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0/(Xc(2)*Xc(2));

    Eigen::Matrix<double,3,3> proj_jac;
    proj_jac.block<2,3>(0,0) = VPose->estimate().cameras[cam_idx]->jacobian(Xc.cast<float>()).cast<double>();
    proj_jac.block<1,3>(2,0) = proj_jac.block<1,3>(0,0);
    proj_jac(2,2) += bf*inv_z2;

    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

void EdgeStereoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().R_cw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().t_cw[cam_idx];
    const Eigen::Vector3d Xc = Rcw*Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().R_bc[cam_idx]*Xc+VPose->estimate().t_bc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().R_cb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0/(Xc(2)*Xc(2));

    Eigen::Matrix<double,3,3> proj_jac;
    proj_jac.block<2,3>(0,0) = VPose->estimate().cameras[cam_idx]->jacobian(Xc.cast<float>()).cast<double>();
    proj_jac.block<1,3>(2,0) = proj_jac.block<1,3>(0,0);
    proj_jac(2,2) += bf*inv_z2;

    Eigen::Matrix<double,3,6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z,   -y, 1.0, 0.0, 0.0,
            -z , 0.0, x, 0.0, 1.0, 0.0,
            y ,  -x , 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv;
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

EdgeInertial::EdgeInertial(IMU::Preintegrated *pInt):JRg(pInt->JR_gyro.cast<double>()),
    JVg(pInt->JV_gyro.cast<double>()), JPg(pInt->JP_gyro.cast<double>()), JVa(pInt->JV_acc.cast<double>()),
    JPa(pInt->JP_acc.cast<double>()), mpInt(pInt), dt(pInt->t)
{
    // This edge links 6 vertices
    resize(6);
    g << 0, 0, -IMU::kGravity;

    Matrix9d Info = pInt->C.block<9,9>(0,0).cast<double>().inverse();
    Info = (Info+Info.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9> > es(Info);
    Eigen::Matrix<double,9,1> eigs = es.eigenvalues();
    for(int i=0;i<9;i++)
        if(eigs[i]<1e-12)
            eigs[i]=0;
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}




void EdgeInertial::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
    const Eigen::Matrix3d dR = mpInt->getDeltaRotation(b1).cast<double>();
    const Eigen::Vector3d dV = mpInt->getDeltaVelocity(b1).cast<double>();
    const Eigen::Vector3d dP = mpInt->getDeltaPosition(b1).cast<double>();

    const Eigen::Vector3d er = logSO3(dR.transpose()*VP1->estimate().R_wb.transpose()*VP2->estimate().R_wb);
    const Eigen::Vector3d ev = VP1->estimate().R_wb.transpose()*(VV2->estimate() - VV1->estimate() - g*dt) - dV;
    const Eigen::Vector3d ep = VP1->estimate().R_wb.transpose()*(VP2->estimate().t_wb - VP1->estimate().t_wb
                                                               - VV1->estimate()*dt - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus()
{
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2= static_cast<const VertexVelocity*>(_vertices[5]);
    const IMU::Bias b1(VA1->estimate()[0],VA1->estimate()[1],VA1->estimate()[2],VG1->estimate()[0],VG1->estimate()[1],VG1->estimate()[2]);
    const IMU::Bias db = mpInt->getDeltaBias(b1);
    Eigen::Vector3d dbg;
    dbg << db.wx, db.wy, db.wz;

    const Eigen::Matrix3d Rwb1 = VP1->estimate().R_wb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().R_wb;

    const Eigen::Matrix3d dR = mpInt->getDeltaRotation(b1).cast<double>();
    const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
    const Eigen::Vector3d er = logSO3(eR);
    const Eigen::Matrix3d invJr = inverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
     // rotation
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1; // OK
    _jacobianOplus[0].block<3,3>(3,0) = Sophus::SO3d::hat(Rbw1*(VV2->estimate() - VV1->estimate() - g*dt)); // OK
    _jacobianOplus[0].block<3,3>(6,0) = Sophus::SO3d::hat(Rbw1*(VP2->estimate().t_wb - VP1->estimate().t_wb
                                                   - VV1->estimate()*dt - 0.5*g*dt*dt)); // OK
    // translation
    _jacobianOplus[0].block<3,3>(6,3) = -Eigen::Matrix3d::Identity(); // OK

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(3,0) = -Rbw1; // OK
    _jacobianOplus[1].block<3,3>(6,0) = -Rbw1*dt; // OK

    // Jacobians wrt Gyro 1
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*rightJacobianSO3(JRg*dbg)*JRg; // OK
    _jacobianOplus[2].block<3,3>(3,0) = -JVg; // OK
    _jacobianOplus[2].block<3,3>(6,0) = -JPg; // OK

    // Jacobians wrt Accelerometer 1
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(3,0) = -JVa; // OK
    _jacobianOplus[3].block<3,3>(6,0) = -JPa; // OK

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3,3>(0,0) = invJr; // OK
    // translation
    _jacobianOplus[4].block<3,3>(6,3) = Rbw1*Rwb2; // OK

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3,3>(3,0) = Rbw1; // OK
}

EdgeInertialGS::EdgeInertialGS(IMU::Preintegrated *pInt):JRg(pInt->JR_gyro.cast<double>()),
    JVg(pInt->JV_gyro.cast<double>()), JPg(pInt->JP_gyro.cast<double>()), JVa(pInt->JV_acc.cast<double>()),
    JPa(pInt->JP_acc.cast<double>()), mpInt(pInt), dt(pInt->t)
{
    // This edge links 8 vertices
    resize(8);
    gI << 0, 0, -IMU::kGravity;

    Matrix9d Info = pInt->C.block<9,9>(0,0).cast<double>().inverse();
    Info = (Info+Info.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9> > es(Info);
    Eigen::Matrix<double,9,1> eigs = es.eigenvalues();
    for(int i=0;i<9;i++)
        if(eigs[i]<1e-12)
            eigs[i]=0;
    Info = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    setInformation(Info);
}



void EdgeInertialGS::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between linearization point and current estimate is too big
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGravityDirection* VGDir = static_cast<const VertexGravityDirection*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
    const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    g = VGDir->estimate().R_wg*gI;
    const double s = VS->estimate();
    const Eigen::Matrix3d dR = mpInt->getDeltaRotation(b).cast<double>();
    const Eigen::Vector3d dV = mpInt->getDeltaVelocity(b).cast<double>();
    const Eigen::Vector3d dP = mpInt->getDeltaPosition(b).cast<double>();

    const Eigen::Vector3d er = logSO3(dR.transpose()*VP1->estimate().R_wb.transpose()*VP2->estimate().R_wb);
    const Eigen::Vector3d ev = VP1->estimate().R_wb.transpose()*(s*(VV2->estimate() - VV1->estimate()) - g*dt) - dV;
    const Eigen::Vector3d ep = VP1->estimate().R_wb.transpose()*(s*(VP2->estimate().t_wb - VP1->estimate().t_wb - VV1->estimate()*dt) - g*dt*dt/2) - dP;

    _error << er, ev, ep;
}

void EdgeInertialGS::linearizeOplus()
{
    const VertexPose* VP1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV1= static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG= static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA= static_cast<const VertexAccBias*>(_vertices[3]);
    const VertexPose* VP2 = static_cast<const VertexPose*>(_vertices[4]);
    const VertexVelocity* VV2 = static_cast<const VertexVelocity*>(_vertices[5]);
    const VertexGravityDirection* VGDir = static_cast<const VertexGravityDirection*>(_vertices[6]);
    const VertexScale* VS = static_cast<const VertexScale*>(_vertices[7]);
    const IMU::Bias b(VA->estimate()[0],VA->estimate()[1],VA->estimate()[2],VG->estimate()[0],VG->estimate()[1],VG->estimate()[2]);
    const IMU::Bias db = mpInt->getDeltaBias(b);

    Eigen::Vector3d dbg;
    dbg << db.wx, db.wy, db.wz;

    const Eigen::Matrix3d Rwb1 = VP1->estimate().R_wb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = VP2->estimate().R_wb;
    const Eigen::Matrix3d Rwg = VGDir->estimate().R_wg;
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3,2);
    Gm(0,1) = -IMU::kGravity;
    Gm(1,0) = IMU::kGravity;
    const double s = VS->estimate();
    const Eigen::MatrixXd dGdTheta = Rwg*Gm;
    const Eigen::Matrix3d dR = mpInt->getDeltaRotation(b).cast<double>();
    const Eigen::Matrix3d eR = dR.transpose()*Rbw1*Rwb2;
    const Eigen::Vector3d er = logSO3(eR);
    const Eigen::Matrix3d invJr = inverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
     // rotation
    _jacobianOplus[0].block<3,3>(0,0) = -invJr*Rwb2.transpose()*Rwb1;
    _jacobianOplus[0].block<3,3>(3,0) = Sophus::SO3d::hat(Rbw1*(s*(VV2->estimate() - VV1->estimate()) - g*dt));
    _jacobianOplus[0].block<3,3>(6,0) = Sophus::SO3d::hat(Rbw1*(s*(VP2->estimate().t_wb - VP1->estimate().t_wb
                                                   - VV1->estimate()*dt) - 0.5*g*dt*dt));
    // translation
    _jacobianOplus[0].block<3,3>(6,3) = Eigen::DiagonalMatrix<double,3>(-s,-s,-s);

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(3,0) = -s*Rbw1;
    _jacobianOplus[1].block<3,3>(6,0) = -s*Rbw1*dt;

    // Jacobians wrt Gyro bias
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(0,0) = -invJr*eR.transpose()*rightJacobianSO3(JRg*dbg)*JRg;
    _jacobianOplus[2].block<3,3>(3,0) = -JVg;
    _jacobianOplus[2].block<3,3>(6,0) = -JPg;

    // Jacobians wrt Accelerometer bias
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(3,0) = -JVa;
    _jacobianOplus[3].block<3,3>(6,0) = -JPa;

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3,3>(0,0) = invJr;
    // translation
    _jacobianOplus[4].block<3,3>(6,3) = s*Rbw1*Rwb2;

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3,3>(3,0) = s*Rbw1;

    // Jacobians wrt Gravity direction
    _jacobianOplus[6].setZero();
    _jacobianOplus[6].block<3,2>(3,0) = -Rbw1*dGdTheta*dt;
    _jacobianOplus[6].block<3,2>(6,0) = -0.5*Rbw1*dGdTheta*dt*dt;

    // Jacobians wrt scale factor
    _jacobianOplus[7].setZero();
    _jacobianOplus[7].block<3,1>(3,0) = Rbw1*(VV2->estimate()-VV1->estimate());
    _jacobianOplus[7].block<3,1>(6,0) = Rbw1*(VP2->estimate().t_wb-VP1->estimate().t_wb-VV1->estimate()*dt);
}

EdgePriorPoseImu::EdgePriorPoseImu(ConstraintPoseImu *c)
{
    resize(4);
    Rwb = c->Rwb;
    twb = c->twb;
    vwb = c->vwb;
    bg = c->bg;
    ba = c->ba;
    setInformation(c->H);
}

void EdgePriorPoseImu::computeError()
{
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[1]);
    const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[2]);
    const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[3]);

    const Eigen::Vector3d er = logSO3(Rwb.transpose()*VP->estimate().R_wb);
    const Eigen::Vector3d et = Rwb.transpose()*(VP->estimate().t_wb-twb);
    const Eigen::Vector3d ev = VV->estimate() - vwb;
    const Eigen::Vector3d ebg = VG->estimate() - bg;
    const Eigen::Vector3d eba = VA->estimate() - ba;

    _error << er, et, ev, ebg, eba;
}

void EdgePriorPoseImu::linearizeOplus()
{
    const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d er = logSO3(Rwb.transpose()*VP->estimate().R_wb);
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3,3>(0,0) = inverseRightJacobianSO3(er);
    _jacobianOplus[0].block<3,3>(3,3) = Rwb.transpose()*VP->estimate().R_wb;
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3,3>(6,0) = Eigen::Matrix3d::Identity();
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3,3>(9,0) = Eigen::Matrix3d::Identity();
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3,3>(12,0) = Eigen::Matrix3d::Identity();
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
