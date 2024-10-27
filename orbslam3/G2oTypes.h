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

#ifndef G2OTYPES_H
#define G2OTYPES_H

// Standard
#include <cmath>
// 3rdparty
#include <orbslam3/external/g2o/g2o/core/base_binary_edge.h>
#include <orbslam3/external/g2o/g2o/core/base_multi_edge.h>
#include <orbslam3/external/g2o/g2o/core/base_unary_edge.h>
#include <orbslam3/external/g2o/g2o/core/base_vertex.h>
#include <orbslam3/external/g2o/g2o/types/types_sba.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
// Local
#include "orbslam3/ImuTypes.h"

namespace ORB_SLAM3 {

class KeyFrame;
class Frame;
class GeometricCamera;

// ────────────────────────────────────────────────────────────────────────── //
// Aliases

using Vector6d  = Eigen::Matrix<double,  6,  1>;
using Vector9d  = Eigen::Matrix<double,  9,  1>;
using Vector12d = Eigen::Matrix<double, 12,  1>;
using Vector15d = Eigen::Matrix<double, 15,  1>;
using Matrix6d  = Eigen::Matrix<double,  6,  6>;
using Matrix9d  = Eigen::Matrix<double,  9,  9>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Matrix24d = Eigen::Matrix<double, 24, 24>;
using Matrix27d = Eigen::Matrix<double, 27, 27>;

// ────────────────────────────────────────────────────────────────────────── //
// Functions

// Skew symmetric matrix.
Eigen::Matrix3d skew(const Eigen::Vector3d& w);
// Normalize rotation matrix.
Eigen::Matrix3d normalizeRotation(const Eigen::Matrix3d& R);
// Exponential map for SO3 group that converts an angle-axis vector to a
// rotation matrix.
Eigen::Matrix3d expSO3(const Eigen::Vector3d& w);
// Logarithm map for SO3 group that converts a rotation matrix to an angle-axis
// vector.
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);
// Right Jacobian of SO3 group of an angle-axis vector.
Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& w);
// Inverse right Jacobian of SO3 group of an angle-axis vector.
Eigen::Matrix3d inverseRightJacobianSO3(const Eigen::Vector3d& w);

// ────────────────────────────────────────────────────────────────────────── //
// Classes

// Class handling the IMU and camera poses, where there are 3 principal frames:
// - World frame (w)
// - Body frame (b) for IMU
// - Camera frame (c)
class ImuCamPose {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Rotation/Translation from body frame to world frame.
  Eigen::Matrix3d R_wb;
  Eigen::Vector3d t_wb;
  // Sets of rotation/translation from world frame to camera frame.
  std::vector<Eigen::Matrix3d> R_cw;
  std::vector<Eigen::Vector3d> t_cw;
  // Sets of rotation/translation from body frame to camera frame.
  std::vector<Eigen::Matrix3d> R_cb;
  std::vector<Eigen::Vector3d> t_cb;
  // Sets of rotation/translation from camera frame to body frame.
  std::vector<Eigen::Matrix3d> R_bc;
  std::vector<Eigen::Vector3d> t_bc;
  // Set of camera models.
  std::vector<GeometricCamera*> cameras;
  // Multiplicative factor of baseline and focal length.
  double bf;

  // ──────────────────────────────────── //
  // Constructors

  ImuCamPose() {}
  ImuCamPose(const KeyFrame* keyframe);
  ImuCamPose(const Frame* frame);
  ImuCamPose(
    const Eigen::Matrix3d& R_wc,
    const Eigen::Vector3d& t_wc,
    const KeyFrame* keyframe
  );

  // ──────────────────────────────────── //
  // Public methods

  void setParameters(
    const std::vector<Eigen::Matrix3d>& _R_cw,
    const std::vector<Eigen::Vector3d>& _t_cw,
    const std::vector<Eigen::Matrix3d>& _R_bc,
    const std::vector<Eigen::Vector3d>& _t_bc,
    const double _bf
  );

  // Project a 3D point in the camera frame to the image plane for Monocular camera.
  Eigen::Vector2d projectMonocular(
    const Eigen::Vector3d& pt,
    const std::size_t cam_idx = 0
  ) const;
  // Project a 3D point in the camera frame to the image plane for Stereo camera.
  Eigen::Vector3d projectStereo(
    const Eigen::Vector3d& pt,
    const std::size_t cam_idx = 0
  ) const;

  // Check if the depth of a 3D point in the camera frame is positive.
  bool isDepthPositive(
    const Eigen::Vector3d& pt,
    const std::size_t cam_idx = 0
  ) const;

  // Update poses in the body frame and world frame from incremental update.
  // Please make sure the update has 6 elements: 3 for rotation and 3 for translation.
  void updateInBodyFrame (const double* update);
  void UpdateInWorldFrame(const double* update);

private:
  // Initial rotation matrix from body to world frame.
  Eigen::Matrix3d R0_wb_;
  // Incremental rotation matrix difference from body to world frame.
  Eigen::Matrix3d dR_;
  // Number of iterations for normalization.
  std::size_t iterations_;
};

// Optimization vertex for the IMU and camera poses.
class VertexPose : public g2o::BaseVertex<6, ImuCamPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexPose() = default;

  VertexPose(const KeyFrame* keyframe);

  VertexPose(const Frame* frame);

  virtual bool read(std::istream& is);

  virtual bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
  }

  virtual void oplusImpl(const double* update) {
    _estimate.updateInBodyFrame(update);
    updateCache();
  }
};

// Optimization vertex for the IMU and camera poses with 4 degrees of freedom:
// translation and yaw.
class VertexPose4DoF : public g2o::BaseVertex<4, ImuCamPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexPose4DoF() = default;

  VertexPose4DoF(const KeyFrame* keyframe);

  VertexPose4DoF(const Frame* frame);

  VertexPose4DoF(
    const Eigen::Matrix3d& R_wc,
    const Eigen::Vector3d& t_wc,
    const KeyFrame* keyframe
  );

  virtual bool read(std::istream& is) {
    return false;
  }
  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update_) {
    double update_6dof[6];
    update_6dof[0] = 0;
    update_6dof[1] = 0;
    update_6dof[2] = update_[0];
    update_6dof[3] = update_[1];
    update_6dof[4] = update_[2];
    update_6dof[5] = update_[3];
    _estimate.UpdateInWorldFrame(update_6dof);
    updateCache();
  }
};

// Optimization vertex for the velocity of the camera.
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexVelocity() = default;

  VertexVelocity(const KeyFrame* keyframe);

  VertexVelocity(const Frame* frame);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update) {
    const Eigen::Vector3d dv(update[0], update[1], update[2]);
    setEstimate(estimate() + dv);
  }
};

// Optimization vertex for the gyroscope bias.
class VertexGyroBias : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexGyroBias() = default;

  VertexGyroBias(const KeyFrame* keyframe);

  VertexGyroBias(const Frame* frame);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update) {
    const Eigen::Vector3d dbias(update[0], update[1], update[2]);
    setEstimate(estimate() + dbias);
  }
};

// Optimization vertex for the accelerometer bias.
class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexAccBias() = default;

  VertexAccBias(const KeyFrame* keyframe);

  VertexAccBias(const Frame* frame);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update) {
    const Eigen::Vector3d dbias(update[0], update[1], update[2]);
    setEstimate(estimate() + dbias);
  }
};

struct GravityDirection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3d R_wg;

  GravityDirection() = default;

  GravityDirection(Eigen::Matrix3d R_wg) : R_wg(R_wg) {}

  void update(const double* update) {
    R_wg = R_wg * expSO3(Eigen::Vector3d(update[0], update[1], 0.0));
  }
};

// Optimization vertex for the gravity direction.
class VertexGravityDirection : public g2o::BaseVertex<2, GravityDirection> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexGravityDirection() = default;

  VertexGravityDirection(const Eigen::Matrix3d& R_wg);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update) {
    _estimate.update(update);
    updateCache();
  }
};

// Optimization vertex for the scale factor.
class VertexScale : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexScale();

  VertexScale(const double scale);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {
    setEstimate(1.0);
  }

  virtual void oplusImpl(const double* update) {
    setEstimate(estimate() * std::exp(*update));
  }
};

// Class representing an inverse depth point in the host frame of a camera.
struct InverseDepthPoint {
  // Inverse depth value.
  double rho;
  // Observation in the host frame.
  double u, v;
  // Camera intrinsic parameters from the host frame.
  double fx, fy, cx, cy, bf;

  InverseDepthPoint() {}
  InverseDepthPoint(
    const double _rho,
    const double _u,
    const double _v,
    const KeyFrame* host_keyframe
  );

  void update(const double* update);
};

// Optimization vertex for the inverse depth point at the host frame.
class VertexInverseDepthPoint : public g2o::BaseVertex<1, InverseDepthPoint> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexInverseDepthPoint() = default;

  VertexInverseDepthPoint(
    const double rho,
    const double u,
    const double v,
    const KeyFrame* keyframe
  );

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update) {
    _estimate.update(update);
    updateCache();
  }
};

class EdgeMono
  : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeMono(const std::size_t cam_idx = 0);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  bool isDepthPositive();
  Matrix9d getHessian();

private:
  const std::size_t cam_idx_;
};

class EdgeMonoOnlyPose
  : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeMonoOnlyPose(const Eigen::Vector3f& x_w, const std::size_t cam_idx = 0);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  bool isDepthPositive();
  Matrix6d getHessian();

private:
  const Eigen::Vector3d x_w_;
  const std::size_t cam_idx_;
};

class EdgeStereo
  : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereo(const std::size_t cam_idx = 0);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  Matrix9d getHessian();

private:
  const std::size_t cam_idx_;
};

class EdgeStereoOnlyPose
  : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoOnlyPose(const Eigen::Vector3f& x_w, const std::size_t cam_idx = 0);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  Matrix6d getHessian();

private:
  const Eigen::Vector3d x_w_; // 3D point coordinates
  const std::size_t cam_idx_;
};

class EdgeInertial : public g2o::BaseMultiEdge<9, Vector9d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeInertial(IMU::Preintegrated* preintegrated);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  Matrix24d getHessian();
  Matrix9d getHessian2();

private:
  const Eigen::Matrix3d JR_gyro_, JV_gyro_, JP_gyro_;
  const Eigen::Matrix3d JV_acc_, JP_acc_;
  IMU::Preintegrated* preintegrated_;
  const double dt_;
  Eigen::Vector3d g_;
};

// Edge inertial where gravity is included as optimizable variable and it is not
// supposed to be pointing in -z axis, as well as scale.
class EdgeInertialGS : public g2o::BaseMultiEdge<9, Vector9d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeInertialGS(IMU::Preintegrated* preintegrated);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

private:
  const Eigen::Matrix3d JR_gyro_, JV_gyro_, JP_gyro_;
  const Eigen::Matrix3d JV_acc_, JP_acc_;
  IMU::Preintegrated* preintegrated_;
  const double dt;
  Eigen::Vector3d g_, g0_;
};

class EdgeGyroRW
  : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGyroRW() = default;

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  Matrix6d getHessian();
  Eigen::Matrix3d getHessian2();
};

class EdgeAccRW
  : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeAccRW() = default;

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

  Matrix6d getHessian();
  Eigen::Matrix3d getHessian2();
};

class ConstraintPoseImu {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConstraintPoseImu(
    const Eigen::Matrix3d& R_wb,
    const Eigen::Vector3d& t_wb,
    const Eigen::Vector3d& v_wb,
    const Eigen::Vector3d& bias_gyro,
    const Eigen::Vector3d& bias_acc,
    const Matrix15d& H
  );

  const Eigen::Matrix3d R_wb_;
  const Eigen::Vector3d t_wb_;
  const Eigen::Vector3d v_wb_;
  const Eigen::Vector3d bias_gyro_;
  const Eigen::Vector3d bias_acc_;
  const Matrix15d H_;

private:
  Matrix15d solveHessian(Matrix15d H);
};

class EdgePriorPoseImu : public g2o::BaseMultiEdge<15, Vector15d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePriorPoseImu(const ConstraintPoseImu* constraint);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  void computeError();

  virtual void linearizeOplus();

  Matrix15d getHessian();

private:
  Eigen::Matrix3d R_wb_;
  Eigen::Vector3d t_wb_;
  Eigen::Vector3d v_wb_;
  Eigen::Vector3d bias_gyro_;
  Eigen::Vector3d bias_acc_;
};

// Edge for the prior of the accelerometer bias.
class EdgePriorAcc
  : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePriorAcc(const Eigen::Vector3f& prior);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

private:
  const Eigen::Vector3d prior_;
};

class EdgePriorGyro
  : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyroBias> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePriorGyro(const Eigen::Vector3f& prior);

  virtual bool read(std::istream& is) {
    return false;
  }

  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void linearizeOplus();

  void computeError();

private:
  const Eigen::Vector3d prior_;
};

class Edge4DoF : public g2o::BaseBinaryEdge<6,Vector6d,VertexPose4DoF,VertexPose4DoF>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge4DoF(const Eigen::Matrix4d &deltaT){
        dTij = deltaT;
        dRij = deltaT.block<3,3>(0,0);
        dtij = deltaT.block<3,1>(0,3);
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose4DoF* VPi = static_cast<const VertexPose4DoF*>(_vertices[0]);
        const VertexPose4DoF* VPj = static_cast<const VertexPose4DoF*>(_vertices[1]);
        _error << logSO3(VPi->estimate().R_cw[0]*VPj->estimate().R_cw[0].transpose()*dRij.transpose()),
                 VPi->estimate().R_cw[0]*(-VPj->estimate().R_cw[0].transpose()*VPj->estimate().t_cw[0])+VPi->estimate().t_cw[0] - dtij;
    }

    // virtual void linearizeOplus(); // numerical implementation

    Eigen::Matrix4d dTij;
    Eigen::Matrix3d dRij;
    Eigen::Vector3d dtij;
};

} // namespace ORB_SLAM3

#endif // G2OTYPES_H
