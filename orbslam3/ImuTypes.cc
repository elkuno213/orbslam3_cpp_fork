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
#include <Eigen/SVD>
// Local
#include "orbslam3/ImuTypes.h"

namespace ORB_SLAM3::IMU {

constexpr float kEpsilon = 1e-4;

// ────────────────────────────────────────────────────────────────────────── //
// Point

Point::Point(
  const float ax,
  const float ay,
  const float az,
  const float wx,
  const float wy,
  const float wz,
  const double t
)
  : acc(ax, ay, az), gyro(wx, wy, wz), t(t) {}

Point::Point(
  const Eigen::Vector3f& acc,
  const Eigen::Vector3f& gyro,
  const double t
)
  : acc(acc), gyro(gyro), t(t) {}

Point::Point(const cv::Point3f& acc, const cv::Point3f& gyro, const double t)
  : acc(acc.x, acc.y, acc.z), gyro(gyro.x, gyro.y, gyro.z), t(t) {}

// ────────────────────────────────────────────────────────────────────────── //
// IMU Bias

Bias::Bias(
  const float bias_ax,
  const float bias_ay,
  const float bias_az,
  const float bias_wx,
  const float bias_wy,
  const float bias_wz
)
  : ax(bias_ax)
  , ay(bias_ay)
  , az(bias_az)
  , wx(bias_wx)
  , wy(bias_wy)
  , wz(bias_wz) {}

Bias Bias::operator-(const Bias& other) const {
  return Bias(
    ax - other.ax,
    ay - other.ay,
    az - other.az,
    wx - other.wx,
    wy - other.wy,
    wz - other.wz
  );
}

void Bias::copyFrom(const Bias& other) {
  ax = other.ax;
  ay = other.ay;
  az = other.az;
  wx = other.wx;
  wy = other.wy;
  wz = other.wz;
}

std::ostream& operator<<(std::ostream& out, const Bias& bias) {
  if (bias.wx > 0) {
    out << " ";
  }
  out << bias.wx << ",";
  if (bias.wy > 0) {
    out << " ";
  }
  out << bias.wy << ",";
  if (bias.wz > 0) {
    out << " ";
  }
  out << bias.wz << ",";
  if (bias.ax > 0) {
    out << " ";
  }
  out << bias.ax << ",";
  if (bias.ay > 0) {
    out << " ";
  }
  out << bias.ay << ",";
  if (bias.az > 0) {
    out << " ";
  }
  out << bias.az;

  return out;
}

// ────────────────────────────────────────────────────────────────────────── //
// IMU Calibration

Calib::Calib() {
  is_set = false;

  T_bc = Sophus::SE3f();
  T_cb = Sophus::SE3f();
  noise_cov.setZero();
  walk_cov.setZero();
}

Calib::Calib(
  const Sophus::SE3f& camera_to_body,
  const float noise_gyro,
  const float noise_acc,
  const float walk_gyro,
  const float walk_acc
) {
  is_set = true;

  const float noise_gyro_squared = noise_gyro * noise_gyro;
  const float noise_acc_squared  =  noise_acc * noise_acc ;
  const float walk_gyro_squared  =  walk_gyro * walk_gyro ;
  const float walk_acc_squared   =   walk_acc * walk_acc  ;

  T_bc = camera_to_body;
  T_cb = T_bc.inverse();
  noise_cov.diagonal() << noise_gyro_squared, noise_gyro_squared, noise_gyro_squared,
                           noise_acc_squared,  noise_acc_squared,  noise_acc_squared;
  walk_cov.diagonal() << walk_gyro_squared, walk_gyro_squared, walk_gyro_squared,
                          walk_acc_squared,  walk_acc_squared,  walk_acc_squared;
}

// ────────────────────────────────────────────────────────────────────────── //
// Integrated Rotation

IntegratedRotation::IntegratedRotation(
  const Eigen::Vector3f& gyro,
  const Bias& bias,
  const double dt
) {
  if (dt <= 0.0) {
    LOG(ERROR) << "IntegratedRotation: dt must be positive.";
    throw std::runtime_error("IntegratedRotation: dt must be positive.");
  }

  // Calculate the angular displacement in the body frame.
  const float x = (gyro.x() - bias.wx) * dt;
  const float y = (gyro.y() - bias.wy) * dt;
  const float z = (gyro.z() - bias.wz) * dt;

  // Calculate the angular displacement and its orders.
  const float d_squared = x * x + y * y + z * z;
  const float d         = std::sqrt(d_squared);

  // Calculate the incremental rotation and right Jacobian.
  const Eigen::Vector3f v = (Eigen::Vector3f() << x, y, z).finished();
  const Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

  if (d < kEpsilon) {
    dR            = I + W;
    rightJacobian = I;
  } else {
    dR            = I
                  + W * std::sin(d) / d
                  + W * W * (1.f - std::cos(d)) / d_squared;
    rightJacobian = I
                  - W * (1.f - std::cos(d)) / d_squared
                  + W * W * (d   - std::sin(d)) / (d_squared * d);
  }
}

// ────────────────────────────────────────────────────────────────────────── //
// Preintegrated IMU Measurements

Preintegrated::Preintegrated() {
  noise_cov.setZero();
  walk_cov.setZero();
  initialize(Bias());
}

Preintegrated::Preintegrated(const Bias& bias, const Calib& calib) {
  noise_cov = calib.noise_cov;
  walk_cov  = calib.walk_cov;
  initialize(bias);
}

// Copy constructor
Preintegrated::Preintegrated(Preintegrated* other)
  : t(other->t)
  , C(other->C)
  , info(other->info)
  , noise_cov(other->noise_cov)
  , walk_cov(other->walk_cov)
  , bias(other->bias)
  , dR(other->dR)
  , dV(other->dV)
  , dP(other->dP)
  , JR_gyro(other->JR_gyro)
  , JV_gyro(other->JV_gyro)
  , JV_acc(other->JV_acc)
  , JP_gyro(other->JP_gyro)
  , JP_acc(other->JP_acc)
  , mean_acc(other->mean_acc)
  , mean_gyro(other->mean_gyro)
  , updated_bias_(other->updated_bias_)
  , bias_diff_(other->bias_diff_)
  , measurements_(other->measurements_) {}

void Preintegrated::copyFrom(Preintegrated* other) {
  t         = other->t;
  C         = other->C;
  info      = other->info;
  noise_cov = other->noise_cov;
  walk_cov  = other->walk_cov;
  bias.copyFrom(other->bias);
  dR        = other->dR;
  dV        = other->dV;
  dP        = other->dP;
  JR_gyro   = other->JR_gyro;
  JV_gyro   = other->JV_gyro;
  JV_acc    = other->JV_acc;
  JP_gyro   = other->JP_gyro;
  JP_acc    = other->JP_acc;
  mean_acc  = other->mean_acc;
  mean_gyro = other->mean_gyro;
  updated_bias_.copyFrom(other->updated_bias_);
  bias_diff_    = other->bias_diff_;
  measurements_ = other->measurements_;
}

void Preintegrated::initialize(const Bias& init_bias) {
  t = 0.0;
  C.setZero();
  info.setZero();
  bias = init_bias;
  dR.setIdentity();
  dV.setZero();
  dP.setZero();
  JR_gyro.setZero();
  JV_gyro.setZero();
  JV_acc.setZero();
  JP_gyro.setZero();
  JP_acc.setZero();
  mean_acc.setZero();
  mean_gyro.setZero();
  updated_bias_ = init_bias;
  bias_diff_ = Bias();
  measurements_.clear();
}

void Preintegrated::integrateNewMeasurement(
  const Eigen::Vector3f& new_acc,
  const Eigen::Vector3f& new_gyro,
  const double dt
) {
  if (dt <= 0.0) {
    LOG(ERROR) << "Preintegrated: dt must be positive.";
    throw std::runtime_error("Preintegrated: dt must be positive.");
    return;
  }

  // Store new measurement.
  measurements_.emplace_back(new_acc, new_gyro, dt);

  // Correct acceleration and gyroscope measurements with bias.
  Eigen::Vector3f acc, gyro;
  acc  << new_acc.x()  - bias.ax, new_acc.y()  - bias.ay, new_acc.z()  - bias.az;
  gyro << new_gyro.x() - bias.wx, new_gyro.y() - bias.wy, new_gyro.z() - bias.wz;

  const Eigen::Matrix3f acc_hat = Sophus::SO3f::hat(acc);

  // Update mean acceleration and gyroscope of measurements.
  mean_acc  = (t * mean_acc  + dR * acc * dt) / (t + dt);
  mean_gyro = (t * mean_gyro + gyro * dt    ) / (t + dt);

  // ──────────────────────────── //
  // Update in the following order:
  // - Position using previous velocity and rotation.
  // - Velocity using previous rotation.
  // - Rotation.

  // Initialize matrices A and B for computing the covariance.
  Eigen::Matrix<float, 9, 9> A = Eigen::Matrix<float, 9, 9>::Identity();
  Eigen::Matrix<float, 9, 6> B = Eigen::Matrix<float, 9, 6>::Zero();

  // Update delta position and velocity using previous rotation.
  dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
  dV = dV + dR * acc * dt;

  // Compute velocity and position parts of matrices A and B.
  A.block<3, 3>(3, 0) = -dR * dt * acc_hat;
  A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * acc_hat;
  A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);
  B.block<3, 3>(3, 3) = dR * dt;
  B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;

  // Update position and velocity jacobians.
  JP_acc  = JP_acc  + JV_acc  * dt - 0.5f * dR * dt * dt;
  JP_gyro = JP_gyro + JV_gyro * dt - 0.5f * dR * dt * dt * acc_hat * JR_gyro;
  JV_acc  = JV_acc  - dR * dt;
  JV_gyro = JV_gyro - dR * dt * acc_hat * JR_gyro;

  // Update delta rotation
  IntegratedRotation dRi(new_gyro, bias, dt);
  dR = normalizeRotation(dR * dRi.dR);

  // Compute rotation parts of matrices A and B?
  A.block<3, 3>(0, 0) = dRi.dR.transpose();
  B.block<3, 3>(0, 0) = dRi.rightJacobian * dt;

  // Update covariance.
  C.block<9, 9>(0, 0) = A * C.block<9, 9>(0, 0) * A.transpose() + B * noise_cov * B.transpose();
  C.block<6, 6>(9, 9) += walk_cov;

  // Update rotation jacobian.
  JR_gyro = dRi.dR.transpose() * JR_gyro - dRi.rightJacobian * dt;

  // Increment integrated time.
  t += dt;
}

void Preintegrated::reintegrate() {
  std::unique_lock<std::mutex> lock(mutex_);

  // Reset preintegration with updated bias.
  initialize(updated_bias_);
  // Reintegrate all measurements.
  for (const auto& m : measurements_) {
    integrateNewMeasurement(m.acc, m.gyro, m.t);
  }
}

void Preintegrated::mergePrevious(Preintegrated* previous) {
  if (previous == this) {
    // No merge if the same object.
    LOG(WARNING) << "Preintegrated: Cannot merge with itself.";
    return;
  }

  std::unique_lock<std::mutex> lock_one(mutex_);

  // Reset preintegration with updated bias.
  initialize(updated_bias_);

  // Merge all measurements of the other preintegration.
  {
    std::unique_lock<std::mutex> lock_other(previous->mutex_);
    for (const auto& m : previous->measurements_) {
      integrateNewMeasurement(m.acc, m.gyro, m.t);
    }
  }

  // Merge all measurements of this preintegration.
  for (const auto& m : measurements_) {
    integrateNewMeasurement(m.acc, m.gyro, m.t);
  }
}

void Preintegrated::setNewBias(const Bias& new_bias) {
  std::unique_lock<std::mutex> lock(mutex_);

  updated_bias_ = new_bias;
  bias_diff_    = new_bias - bias;
}

IMU::Bias Preintegrated::getDeltaBias(const Bias& other) {
  std::unique_lock<std::mutex> lock(mutex_);

  return other - bias;
}

Eigen::Matrix3f Preintegrated::getDeltaRotation(const Bias& other) {
  std::unique_lock<std::mutex> lock(mutex_);

  const Bias diff = other - bias;
  return normalizeRotation(dR * Sophus::SO3f::exp(JR_gyro * diff.gyro()).matrix());
}

Eigen::Vector3f Preintegrated::getDeltaVelocity(const Bias& other) {
  std::unique_lock<std::mutex> lock(mutex_);

  const Bias diff = other - bias;
  return dV + JV_gyro * diff.gyro() + JV_acc * diff.acc();
}

Eigen::Vector3f Preintegrated::getDeltaPosition(const Bias& other) {
  std::unique_lock<std::mutex> lock(mutex_);

  const Bias diff = other - bias;
  return dP + JP_gyro * diff.gyro() + JP_acc * diff.acc();
}

Eigen::Matrix3f Preintegrated::getUpdatedDeltaRotation() {
  std::unique_lock<std::mutex> lock(mutex_);

  return normalizeRotation(dR * Sophus::SO3f::exp(JR_gyro * bias_diff_.gyro()).matrix());
}

Eigen::Vector3f Preintegrated::getUpdatedDeltaVelocity() {
  std::unique_lock<std::mutex> lock(mutex_);

  return dV + JV_gyro * bias_diff_.acc() + JV_acc * bias_diff_.gyro();
}

Eigen::Vector3f Preintegrated::getUpdatedDeltaPosition() {
  std::unique_lock<std::mutex> lock(mutex_);

  return dP + JP_gyro * bias_diff_.acc() + JP_acc * bias_diff_.gyro();
}

Eigen::Matrix3f Preintegrated::getOriginalDeltaRotation() {
  std::unique_lock<std::mutex> lock(mutex_);

  return dR;
}

Eigen::Vector3f Preintegrated::getOriginalDeltaVelocity() {
  std::unique_lock<std::mutex> lock(mutex_);

  return dV;
}

Eigen::Vector3f Preintegrated::getOriginalDeltaPosition() {
  std::unique_lock<std::mutex> lock(mutex_);
  return dP;
}

Bias Preintegrated::getOriginalBias() {
  std::unique_lock<std::mutex> lock(mutex_);

  return bias;
}

Bias Preintegrated::getUpdatedBias() {
  std::unique_lock<std::mutex> lock(mutex_);

  return updated_bias_;
}

Bias Preintegrated::getBiasDifference() {
  std::unique_lock<std::mutex> lock(mutex_);

  return bias_diff_;
}

// ────────────────────────────────────────────────────────────────────────── //
// Lie Algebra Functions

Eigen::Matrix3f getRightJacobianSO3(const float x, const float y, const float z) {
  const float d_squared = x * x + y * y + z * z;
  const float d         = std::sqrt(d_squared);
  const Eigen::Vector3f v = (Eigen::Vector3f() << x, y, z).finished();
  const Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

  if (d < kEpsilon) {
    return I;
  } else {
    return I
           - W * (1.0f - std::cos(d)) / d_squared
           + W * W * (d - std::sin(d)) / (d_squared * d);
  }
}

Eigen::Matrix3f getRightJacobianSO3(const Eigen::Vector3f& vector) {
  return getRightJacobianSO3(vector.x(), vector.y(), vector.z());
}

Eigen::Matrix3f getInverseRightJacobianSO3(const float x, const float y, const float z) {
  const float d_squared   = x * x + y * y + z * z;
  const float d           = std::sqrt(d_squared);
  const Eigen::Vector3f v = (Eigen::Vector3f() << x, y, z).finished();
  const Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

  if (d < kEpsilon) {
    return I;
  } else {
    return I
           + W / 2.f
           + W * W * (1.f / d_squared - (1.f + std::cos(d)) / (2.f * d * std::sin(d)));
  }
}

Eigen::Matrix3f getInverseRightJacobianSO3(const Eigen::Vector3f& vector) {
  return getInverseRightJacobianSO3(vector.x(), vector.y(), vector.z());
}

Eigen::Matrix3f normalizeRotation(const Eigen::Matrix3f& R) {
  const Eigen::JacobiSVD<Eigen::Matrix3f> svd(
    R,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  return svd.matrixU() * svd.matrixV().transpose();
}

} // namespace ORB_SLAM3::IMU
