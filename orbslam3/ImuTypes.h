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

#ifndef IMUTYPES_H
#define IMUTYPES_H

// Standard
#include <mutex>
#include <vector>
// 3rdparty
#include <Eigen/Core>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core.hpp>
#include <orbslam3/external/Sophus/sophus/se3.hpp>
// Local
#include "orbslam3/SerializationUtils.h"

namespace ORB_SLAM3::IMU {

// Constants.
constexpr float kGravity = 9.81f;

// IMU measurement point.
struct Point {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3f acc  = Eigen::Vector3f::Zero(); // acceleration     [m/s^2]
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero(); // angular velocity [rad/s]
  double t = 0.0;                                 // timestamp        [s]

  Point() = default;
  Point(
    const float ax,
    const float ay,
    const float az,
    const float wx,
    const float wy,
    const float wz,
    const double t
  );
  Point(const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro, const double t);
  Point(const cv::Point3f& acc, const cv::Point3f& gyro, const double t);

private:
  // ──────────────────────────── //
  // Serialization

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int) {
    ar& boost::serialization::make_array(acc.data() , acc.size());
    ar& boost::serialization::make_array(gyro.data(), gyro.size());
    ar& t;
  }
};

// IMU bias (accelerometer and gyroscope)
struct Bias {
  float ax = 0.f, ay = 0.f, az = 0.f; // accelerometer bias [m/s^2]
  float wx = 0.f, wy = 0.f, wz = 0.f; // gyroscope bias     [rad/s]

  Bias() = default;
  Bias(
    const float bias_ax,
    const float bias_ay,
    const float bias_az,
    const float bias_wx,
    const float bias_wy,
    const float bias_wz
  );

  Bias operator-(const Bias& other) const;

  void copyFrom(const Bias& other);

  Eigen::Vector3f acc()  const { return Eigen::Vector3f(ax, ay, az); }
  Eigen::Vector3f gyro() const { return Eigen::Vector3f(wx, wy, wz); }

  friend std::ostream& operator<<(std::ostream& out, const Bias& bias);

private:
  // ──────────────────────────── //
  // Serialization

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int) {
    ar& ax;
    ar& ay;
    ar& az;

    ar& wx;
    ar& wy;
    ar& wz;
  }
};

// IMU calibration
struct Calib {
  Sophus::SE3f T_bc; // Transformation from camera to body-frame (IMU)
  Sophus::SE3f T_cb; // Transformation from body-frame (IMU) to camera
  Eigen::DiagonalMatrix<float, 6> noise_cov; // Noise covariance
  Eigen::DiagonalMatrix<float, 6> walk_cov;  // Random walk covariance
  bool is_set;

  Calib();
  Calib(
    const Sophus::SE3f& camera_to_body,
    const float noise_gyro,
    const float noise_acc,
    const float walk_gyro,
    const float walk_acc
  );

private:
  // ──────────────────────────── //
  // Serialization

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    serializeSophusSE3(ar, T_cb, version);
    serializeSophusSE3(ar, T_bc, version);

    ar& boost::serialization::make_array(
      noise_cov.diagonal().data(),
      noise_cov.diagonal().size()
    );
    ar& boost::serialization::make_array(
      walk_cov.diagonal().data(),
      walk_cov.diagonal().size()
    );

    ar& is_set;
  }
};

// Integrated rotation from gyroscope measurements.
struct IntegratedRotation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3f dR;
  Eigen::Matrix3f rightJacobian;

  IntegratedRotation() = default;
  IntegratedRotation(const Eigen::Vector3f& gyro, const Bias& bias, const double dt);
};

// Preintegration of IMU measurements
struct Preintegrated {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Total integrated time [s]
  double t;
  // Covariance matrix
  Eigen::Matrix<float, 15, 15> C;
  // Information matrix
  Eigen::Matrix<float, 15, 15> info;
  // Noise and random walk covariance
  Eigen::DiagonalMatrix<float, 6> noise_cov, walk_cov;
  // Original bias
  Bias bias;
  Eigen::Matrix3f dR; // Delta rotation matrix
  Eigen::Vector3f dV, dP; // Delta velocity and position
  // Jacobian matrices for rotation, velocity and position wrt gyro and acc
  Eigen::Matrix3f JR_gyro, JV_gyro, JV_acc, JP_gyro, JP_acc;
  // Mean acceleration and gyroscope
  Eigen::Vector3f mean_acc, mean_gyro;

  // ──────────────────────────── //
  // Constructors

  Preintegrated();
  Preintegrated(const Bias& bias, const Calib& calib);
  Preintegrated(Preintegrated* other);

  // ──────────────────────────── //
  // Public methods

  void copyFrom(Preintegrated* other);
  void initialize(const Bias& init_bias);
  void integrateNewMeasurement(
    const Eigen::Vector3f& acc,
    const Eigen::Vector3f& gyro,
    const double dt
  );
  void reintegrate();
  void mergePrevious(Preintegrated* previous);
  void setNewBias(const Bias& new_bias);

  IMU::Bias getDeltaBias(const Bias& other);

  Eigen::Matrix3f getDeltaRotation(const Bias& other);
  Eigen::Vector3f getDeltaVelocity(const Bias& other);
  Eigen::Vector3f getDeltaPosition(const Bias& other);

  Eigen::Matrix3f getUpdatedDeltaRotation();
  Eigen::Vector3f getUpdatedDeltaVelocity();
  Eigen::Vector3f getUpdatedDeltaPosition();

  Eigen::Matrix3f getOriginalDeltaRotation();
  Eigen::Vector3f getOriginalDeltaVelocity();
  Eigen::Vector3f getOriginalDeltaPosition();

  Bias getOriginalBias();
  Bias getUpdatedBias();
  Bias getBiasDifference();

private:
  // ──────────────────────────── //
  // Serialization

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int) {
    ar& t;

    ar& boost::serialization::make_array(C.data()   , C.size()   );
    ar& boost::serialization::make_array(info.data(), info.size());
    ar& boost::serialization::make_array(
      noise_cov.diagonal().data(),
      noise_cov.diagonal().size()
    );
    ar& boost::serialization::make_array(
      walk_cov.diagonal().data(),
      walk_cov.diagonal().size()
    );

    ar& bias;

    ar& boost::serialization::make_array(dR.data()       , dR.size()       );
    ar& boost::serialization::make_array(dV.data()       , dV.size()       );
    ar& boost::serialization::make_array(dP.data()       , dP.size()       );
    ar& boost::serialization::make_array(JR_gyro.data()  , JR_gyro.size()  );
    ar& boost::serialization::make_array(JV_gyro.data()  , JV_gyro.size()  );
    ar& boost::serialization::make_array(JV_acc.data()   , JV_acc.size()   );
    ar& boost::serialization::make_array(JP_gyro.data()  , JP_gyro.size()  );
    ar& boost::serialization::make_array(JP_acc.data()   , JP_acc.size()   );
    ar& boost::serialization::make_array(mean_acc.data() , mean_acc.size() );
    ar& boost::serialization::make_array(mean_gyro.data(), mean_gyro.size());

    ar& updated_bias_;
    ar& bias_diff_;
    ar& measurements_;
  }

private:
  // Updated bias.
  Bias updated_bias_;
  // Difference between the original and updated bias, which is used to compute
  // the updated values of the preintegration.
  Bias bias_diff_;
  // Measurements stored.
  std::vector<Point> measurements_;
  std::mutex mutex_;
};

// Lie Algebra Functions
Eigen::Matrix3f normalizeRotation(const Eigen::Matrix3f& R);

} // namespace ORB_SLAM3::IMU

#endif // IMUTYPES_H
