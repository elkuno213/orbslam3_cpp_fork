// 3rdparty
#include <gtest/gtest.h>
// Local
#include "orbslam3/G2oTypes.h"

TEST(G2oTypes, NormalizeRotation) {
  {
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_normalized = ORB_SLAM3::normalizeRotation(R);
    EXPECT_TRUE(R.isApprox(R_normalized));
  }
  {
    const Eigen::Matrix3d R
      = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d R_multiplied = R * 2.0;
    const Eigen::Matrix3d R_normalized = ORB_SLAM3::normalizeRotation(R_multiplied);
    EXPECT_TRUE(R.isApprox(R_normalized));
  }
}

TEST(G2oTypes, ExpSO3) {
  {
    const Eigen::Vector3d w = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d R = ORB_SLAM3::expSO3(w);
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity()));
  }
  {
    const Eigen::Vector3d w(0.0, 0.0, M_PI_2);
    const Eigen::Matrix3d R = ORB_SLAM3::expSO3(w);
    const Eigen::Matrix3d R_expected = (Eigen::Matrix3d() <<
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0
    ).finished();
    EXPECT_TRUE(R.isApprox(R_expected));
  }
}

TEST(G2oTypes, LogSO3) {
  {
    const Eigen::Matrix3d R = Eigen::DiagonalMatrix<double, 3>(1.0, 2.0, 3.0);
    const Eigen::Vector3d w = ORB_SLAM3::logSO3(R);
    EXPECT_TRUE(w.isApprox(Eigen::Vector3d::Zero()));
  }
  {
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d w = ORB_SLAM3::logSO3(R);
    EXPECT_TRUE(w.isApprox(Eigen::Vector3d::Zero()));
  }
  {
    const Eigen::Matrix3d R
      = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d w = ORB_SLAM3::logSO3(R);
    const Eigen::Vector3d w_expected = Eigen::Vector3d::UnitZ() * M_PI_2;
    EXPECT_TRUE(w.isApprox(w_expected));
  }
}

TEST(G2oTypes, RightJacobianSO3) {
  {
    const Eigen::Vector3d w = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d J = ORB_SLAM3::rightJacobianSO3(w);
    EXPECT_TRUE(J.isApprox(Eigen::Matrix3d::Identity()));
  }
  {
    const Eigen::Vector3d w(0.0, 0.0, M_PI_2);
    const Eigen::Matrix3d J = ORB_SLAM3::rightJacobianSO3(w);
    const Eigen::Matrix3d J_expected = (Eigen::Matrix3d() <<
       0.63662, 0.63662, 0.0,
      -0.63662, 0.63662, 0.0,
           0.0,     0.0, 1.0
    ).finished();
    EXPECT_TRUE(J.isApprox(J_expected, 1e-6));
  }
}

TEST(G2oTypes, InverseRightJacobianSO3) {
  {
    const Eigen::Vector3d w = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d J = ORB_SLAM3::inverseRightJacobianSO3(w);
    EXPECT_TRUE(J.isApprox(Eigen::Matrix3d::Identity()));
  }
  {
    const Eigen::Vector3d w(0.0, 0.0, M_PI_2);
    const Eigen::Matrix3d J = ORB_SLAM3::inverseRightJacobianSO3(w);
    const Eigen::Matrix3d J_expected = (Eigen::Matrix3d() <<
       0.785398, -0.785398, 0.0,
       0.785398,  0.785398, 0.0,
            0.0,      0.0, 1.0
    ).finished();
    EXPECT_TRUE(J.isApprox(J_expected, 1e-6));
  }
}
