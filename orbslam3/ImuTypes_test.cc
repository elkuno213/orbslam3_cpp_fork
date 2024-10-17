// Standard
#include <sstream>
// 3rdparty
#include <gtest/gtest.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
// Local
#include "orbslam3/ImuTypes.h"

TEST(Point, Constructor) {
  const float ax = 0.1;
  const float ay = 0.2;
  const float az = 0.3;
  const float wx = 0.4;
  const float wy = 0.5;
  const float wz = 0.6;
  const cv::Point3f acc(ax, ay, az);
  const cv::Point3f gyro(wx, wy, wz);
  const double timestamp = 1.0;

  const ORB_SLAM3::IMU::Point point_1(ax, ay, az, wx, wy, wz, timestamp);
  EXPECT_EQ(point_1.a.x(), ax);
  EXPECT_EQ(point_1.a.y(), ay);
  EXPECT_EQ(point_1.a.z(), az);
  EXPECT_EQ(point_1.w.x(), wx);
  EXPECT_EQ(point_1.w.y(), wy);
  EXPECT_EQ(point_1.w.z(), wz);
  EXPECT_EQ(point_1.t, timestamp);

  const ORB_SLAM3::IMU::Point point_2(acc, gyro, timestamp);
  EXPECT_EQ(point_2.a.x(), ax);
  EXPECT_EQ(point_2.a.y(), ay);
  EXPECT_EQ(point_2.a.z(), az);
  EXPECT_EQ(point_2.w.x(), wx);
  EXPECT_EQ(point_2.w.y(), wy);
  EXPECT_EQ(point_2.w.z(), wz);
  EXPECT_EQ(point_2.t, timestamp);
}

TEST(Bias, Constructor) {
  const float bias_ax = 0.1;
  const float bias_ay = 0.2;
  const float bias_az = 0.3;
  const float bias_wx = 0.4;
  const float bias_wy = 0.5;
  const float bias_wz = 0.6;
  const ORB_SLAM3::IMU::Bias bias(bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz);

  EXPECT_EQ(bias.bax, bias_ax);
  EXPECT_EQ(bias.bay, bias_ay);
  EXPECT_EQ(bias.baz, bias_az);
  EXPECT_EQ(bias.bwx, bias_wx);
  EXPECT_EQ(bias.bwy, bias_wy);
  EXPECT_EQ(bias.bwz, bias_wz);
}

TEST(Bias, CopyFrom) {
  const float bias_ax = 0.1;
  const float bias_ay = 0.2;
  const float bias_az = 0.3;
  const float bias_wx = 0.4;
  const float bias_wy = 0.5;
  const float bias_wz = 0.6;

  ORB_SLAM3::IMU::Bias bias_1(bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz);
  ORB_SLAM3::IMU::Bias bias_2;
  bias_2.CopyFrom(bias_1);
  EXPECT_EQ(bias_2.bax, bias_ax);
  EXPECT_EQ(bias_2.bay, bias_ay);
  EXPECT_EQ(bias_2.baz, bias_az);
  EXPECT_EQ(bias_2.bwx, bias_wx);
  EXPECT_EQ(bias_2.bwy, bias_wy);
  EXPECT_EQ(bias_2.bwz, bias_wz);
}

TEST(Bias, Serialization) {
  const float bias_ax = 0.1;
  const float bias_ay = 0.2;
  const float bias_az = 0.3;
  const float bias_wx = 0.4;
  const float bias_wy = 0.5;
  const float bias_wz = 0.6;
  const ORB_SLAM3::IMU::Bias bias(bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz);

  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << bias;
  ORB_SLAM3::IMU::Bias loaded_bias;
  boost::archive::text_iarchive ia(ss);
  ia >> loaded_bias;

  EXPECT_EQ(loaded_bias.bax, bias_ax);
  EXPECT_EQ(loaded_bias.bay, bias_ay);
  EXPECT_EQ(loaded_bias.baz, bias_az);
  EXPECT_EQ(loaded_bias.bwx, bias_wx);
  EXPECT_EQ(loaded_bias.bwy, bias_wy);
  EXPECT_EQ(loaded_bias.bwz, bias_wz);
}

class CalibTest : public ::testing::Test {
protected:
  void SetUp() override {
    const Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
    Tbc = Sophus::SE3f(q, t);

    noise_gyro      = 0.1;
    noise_acc       = 0.2;
    noise_gyro_walk = 0.3;
    noise_acc_walk  = 0.4;

    calib = ORB_SLAM3::IMU::Calib(Tbc, noise_gyro, noise_acc, noise_gyro_walk, noise_acc_walk);
  }

  void TearDown() override {}

  void Archive(ORB_SLAM3::IMU::Calib& loaded) {
    std::stringstream ss;
    boost::archive::text_oarchive oa(ss);
    oa << calib;
    boost::archive::text_iarchive ia(ss);
    ia >> loaded;
  }

  ORB_SLAM3::IMU::Calib calib;
  Sophus::SE3f Tbc;
  float noise_gyro;
  float noise_acc;
  float noise_gyro_walk;
  float noise_acc_walk;
};

TEST_F(CalibTest, Constructor) {
  const float noise_gyro_squared      = noise_gyro      * noise_gyro     ;
  const float noise_acc_squared       = noise_acc       * noise_acc      ;
  const float noise_gyro_walk_squared = noise_gyro_walk * noise_gyro_walk;
  const float noise_acc_walk_squared  = noise_acc_walk  * noise_acc_walk ;

  EXPECT_EQ(calib.mTbc.matrix(), Tbc.matrix());
  EXPECT_EQ(calib.Cov.diagonal()(0), noise_gyro_squared);
  EXPECT_EQ(calib.Cov.diagonal()(1), noise_gyro_squared);
  EXPECT_EQ(calib.Cov.diagonal()(2), noise_gyro_squared);
  EXPECT_EQ(calib.Cov.diagonal()(3), noise_acc_squared);
  EXPECT_EQ(calib.Cov.diagonal()(4), noise_acc_squared);
  EXPECT_EQ(calib.Cov.diagonal()(5), noise_acc_squared);
  EXPECT_EQ(calib.CovWalk.diagonal()(0), noise_gyro_walk_squared);
  EXPECT_EQ(calib.CovWalk.diagonal()(1), noise_gyro_walk_squared);
  EXPECT_EQ(calib.CovWalk.diagonal()(2), noise_gyro_walk_squared);
  EXPECT_EQ(calib.CovWalk.diagonal()(3), noise_acc_walk_squared);
  EXPECT_EQ(calib.CovWalk.diagonal()(4), noise_acc_walk_squared);
  EXPECT_EQ(calib.CovWalk.diagonal()(5), noise_acc_walk_squared);
}

TEST_F(CalibTest, Serialization) {
  ORB_SLAM3::IMU::Calib loaded;
  Archive(loaded);

  const float noise_gyro_squared      = noise_gyro      * noise_gyro     ;
  const float noise_acc_squared       = noise_acc       * noise_acc      ;
  const float noise_gyro_walk_squared = noise_gyro_walk * noise_gyro_walk;
  const float noise_acc_walk_squared  = noise_acc_walk  * noise_acc_walk ;

  EXPECT_EQ(loaded.mTbc.matrix(), Tbc.matrix());
  EXPECT_EQ(loaded.Cov.diagonal()(0), noise_gyro_squared);
  EXPECT_EQ(loaded.Cov.diagonal()(1), noise_gyro_squared);
  EXPECT_EQ(loaded.Cov.diagonal()(2), noise_gyro_squared);
  EXPECT_EQ(loaded.Cov.diagonal()(3), noise_acc_squared);
  EXPECT_EQ(loaded.Cov.diagonal()(4), noise_acc_squared);
  EXPECT_EQ(loaded.Cov.diagonal()(5), noise_acc_squared);
  EXPECT_EQ(loaded.CovWalk.diagonal()(0), noise_gyro_walk_squared);
  EXPECT_EQ(loaded.CovWalk.diagonal()(1), noise_gyro_walk_squared);
  EXPECT_EQ(loaded.CovWalk.diagonal()(2), noise_gyro_walk_squared);
  EXPECT_EQ(loaded.CovWalk.diagonal()(3), noise_acc_walk_squared);
  EXPECT_EQ(loaded.CovWalk.diagonal()(4), noise_acc_walk_squared);
  EXPECT_EQ(loaded.CovWalk.diagonal()(5), noise_acc_walk_squared);
}

class IntegratedRotationTest : public ::testing::Test {
protected:
  void SetUp() override {}

  void TearDown() override {}

  void TooSmallEpsilon() {
    gyro = Eigen::Vector3f(0.1f, 0.2f, 0.3f);

    const float ax = 0.;
    const float ay = 0.;
    const float az = 0.;
    const float wx = 0.1 - 1e-5;
    const float wy = 0.2;
    const float wz = 0.3;
    bias = ORB_SLAM3::IMU::Bias(ax, ay, az, wx, wy, wz);

    time = 1.0;

    integrated_rotation = ORB_SLAM3::IMU::IntegratedRotation(gyro, bias, time);
  }

  void NormalCase() {
    gyro = Eigen::Vector3f(0.2f, 0.2f, 0.2f);

    const float ax = 0.;
    const float ay = 0.;
    const float az = 0.;
    const float wx = 0.1;
    const float wy = 0.1;
    const float wz = 0.1;
    bias = ORB_SLAM3::IMU::Bias(ax, ay, az, wx, wy, wz);

    time = 1.0;

    integrated_rotation = ORB_SLAM3::IMU::IntegratedRotation(gyro, bias, time);
  }

  ORB_SLAM3::IMU::IntegratedRotation integrated_rotation;
  Eigen::Vector3f gyro;
  ORB_SLAM3::IMU::Bias bias;
  float time;
};

TEST_F(IntegratedRotationTest, TooSmallEpsilon) {
  TooSmallEpsilon();

  const Eigen::Vector3f v = (Eigen::Vector3f() << 1e-5, 0.f, 0.f).finished();
  const Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  const Eigen::Matrix3f expected_deltaR = Eigen::Matrix3f::Identity() + W;
  const Eigen::Matrix3f expected_rightJ = Eigen::Matrix3f::Identity();

  EXPECT_TRUE(integrated_rotation.deltaR.isApprox(expected_deltaR));
  EXPECT_TRUE(integrated_rotation.rightJ.isApprox(expected_rightJ));
}

TEST_F(IntegratedRotationTest, NormalCase) {
  NormalCase();

  const float x = 0.2 - 0.1;
  const float y = 0.2 - 0.1;
  const float z = 0.2 - 0.1;
  const float d_squared = x * x + y * y + z * z;
  const float d = std::sqrt(d_squared);
  const Eigen::Vector3f v = (Eigen::Vector3f() << x, y, z).finished();
  const Eigen::Matrix3f W = Sophus::SO3f::hat(v);
  const Eigen::Matrix3f W_squared = W * W;
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f expected_deltaR = I + W * std::sin(d) / d + W_squared * (1.0f - std::cos(d)) / d_squared;
  const Eigen::Matrix3f expected_rightJ = I - W * (1.0f - std::cos(d)) / d_squared + W_squared * (d - std::sin(d)) / (d_squared * d);

  EXPECT_TRUE(integrated_rotation.deltaR.isApprox(expected_deltaR));
  EXPECT_TRUE(integrated_rotation.rightJ.isApprox(expected_rightJ));
}

class PreintegratedTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Bias
    const float bias_ax = 0.01;
    const float bias_ay = 0.01;
    const float bias_az = 0.01;
    const float bias_wx = 0.01;
    const float bias_wy = 0.01;
    const float bias_wz = 0.01;
    bias = ORB_SLAM3::IMU::Bias(bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz);

    // Calibration
    const Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f t(0.01, 0.01, 0.01);
    const Sophus::SE3f Tbc(q, t);
    const float noise_gyro      = 0.1;
    const float noise_acc       = 0.2;
    const float noise_gyro_walk = 0.3;
    const float noise_acc_walk  = 0.4;
    calib = ORB_SLAM3::IMU::Calib(Tbc, noise_gyro, noise_acc, noise_gyro_walk, noise_acc_walk);
  }

  void TearDown() override {}

  void Archive(ORB_SLAM3::IMU::Preintegrated& loaded) {
    const ORB_SLAM3::IMU::Preintegrated preintegrated(bias, calib);

    std::stringstream ss;
    boost::archive::text_oarchive oa(ss);
    oa << preintegrated;
    boost::archive::text_iarchive ia(ss);
    ia >> loaded;
  }

  ORB_SLAM3::IMU::Bias bias;
  ORB_SLAM3::IMU::Calib calib;
};

TEST_F(PreintegratedTest, Constructor) {
  // TODO: add == operator to Bias.

  ORB_SLAM3::IMU::Preintegrated preintegrated(bias, calib);

  EXPECT_EQ(preintegrated.b.bax, bias.bax);
  EXPECT_EQ(preintegrated.b.bay, bias.bay);
  EXPECT_EQ(preintegrated.b.baz, bias.baz);
  EXPECT_EQ(preintegrated.b.bwx, bias.bwx);
  EXPECT_EQ(preintegrated.b.bwy, bias.bwy);
  EXPECT_EQ(preintegrated.b.bwz, bias.bwz);
  EXPECT_EQ(preintegrated.Nga.diagonal(), calib.Cov.diagonal());
  EXPECT_EQ(preintegrated.NgaWalk.diagonal(), calib.CovWalk.diagonal());

  ORB_SLAM3::IMU::Preintegrated copied(&preintegrated);
  EXPECT_EQ(copied.b.bax, bias.bax);
  EXPECT_EQ(copied.b.bay, bias.bay);
  EXPECT_EQ(copied.b.baz, bias.baz);
  EXPECT_EQ(copied.b.bwx, bias.bwx);
  EXPECT_EQ(copied.b.bwy, bias.bwy);
  EXPECT_EQ(copied.b.bwz, bias.bwz);
  EXPECT_EQ(copied.Nga.diagonal(), calib.Cov.diagonal());
  EXPECT_EQ(copied.NgaWalk.diagonal(), calib.CovWalk.diagonal());
}

TEST_F(PreintegratedTest, Serialization) {
  ORB_SLAM3::IMU::Preintegrated loaded;
  Archive(loaded);

  EXPECT_EQ(loaded.b.bax, bias.bax);
  EXPECT_EQ(loaded.b.bay, bias.bay);
  EXPECT_EQ(loaded.b.baz, bias.baz);
  EXPECT_EQ(loaded.b.bwx, bias.bwx);
  EXPECT_EQ(loaded.b.bwy, bias.bwy);
  EXPECT_EQ(loaded.b.bwz, bias.bwz);
  EXPECT_EQ(loaded.Nga.diagonal(), calib.Cov.diagonal());
  EXPECT_EQ(loaded.NgaWalk.diagonal(), calib.CovWalk.diagonal());
}

TEST_F(PreintegratedTest, Integration) {
  ORB_SLAM3::IMU::Preintegrated preintegrated(bias, calib);

  // Integrate new measurement.
  {
    // Integrate.
    const Eigen::Vector3f acc(0.1, 0.0, 0.1);
    const Eigen::Vector3f gyro(0.1, 0.0, 0.1);
    const float dt = 0.1f;
    preintegrated.IntegrateNewMeasurement(acc, gyro, dt);

    // Expected values.
    Eigen::Matrix<float, 15, 15> expected_C = Eigen::Matrix<float, 15, 15>::Zero();
    expected_C.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 1e-4f;
    expected_C.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 4e-4f;
    expected_C.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 1e-6f;
    expected_C.block<3, 3>(9, 9) = Eigen::Matrix3f::Identity() * 0.09f;
    expected_C.block<3, 3>(12, 12) = Eigen::Matrix3f::Identity() * 0.16f;
    expected_C.block<3, 3>(3, 6) = Eigen::Matrix3f::Identity() * 2e-5f;
    expected_C.block<3, 3>(6, 3) = Eigen::Matrix3f::Identity() * 2e-5f;
    const Eigen::Matrix<float, 15, 15> expected_Info = Eigen::Matrix<float, 15, 15>::Zero();
    const Eigen::Matrix3f expected_dR = (Eigen::Matrix3f() <<
        0.999959f, -0.00900426f, -0.000959462f,
      0.00899518f,    0.999919f,  -0.00900426f,
      0.00104046f,  0.00899526f,     0.999959f
    ).finished();
    const Eigen::Vector3f expected_dV(0.009f, -0.001f, 0.009f);
    const Eigen::Vector3f expected_dP(0.00045f, -5e-5f, 0.00045f);
    const Eigen::Matrix3f expected_JRg = (Eigen::Matrix3f() <<
       -0.0999986f, -0.000449737f, -5.13359e-05f,
      0.000450037f,   -0.0999973f, -0.000449737f,
       4.8639e-05f,  0.000450037f,   -0.0999986f
    ).finished();
    const Eigen::Matrix3f expected_JVg = Eigen::Matrix3f::Zero();
    const Eigen::Matrix3f expected_JVa = Eigen::DiagonalMatrix<float, 3>(-0.1f, -0.1f, -0.1f);
    const Eigen::Matrix3f expected_JPg = Eigen::Matrix3f::Zero();
    const Eigen::Matrix3f expected_JPa = Eigen::DiagonalMatrix<float, 3>(-0.005f, -0.005f, -0.005f);
    const Eigen::Vector3f expected_avgA(0.09f, -0.01f, 0.09f);
    const Eigen::Vector3f expected_avgW(0.09f, -0.01f, 0.09f);

    // Check.
    EXPECT_NEAR(preintegrated.dT, 0.1f, 1e-6f);
    EXPECT_TRUE(preintegrated.C.isApprox(expected_C));
    EXPECT_TRUE(preintegrated.Info.isApprox(expected_Info));
    EXPECT_TRUE(preintegrated.dR.isApprox(expected_dR));
    EXPECT_TRUE(preintegrated.dV.isApprox(expected_dV));
    EXPECT_TRUE(preintegrated.dP.isApprox(expected_dP));
    EXPECT_TRUE(preintegrated.JRg.isApprox(expected_JRg));
    EXPECT_TRUE(preintegrated.JVg.isApprox(expected_JVg));
    EXPECT_TRUE(preintegrated.JVa.isApprox(expected_JVa));
    EXPECT_TRUE(preintegrated.JPg.isApprox(expected_JPg));
    EXPECT_TRUE(preintegrated.JPa.isApprox(expected_JPa));
    EXPECT_TRUE(preintegrated.avgA.isApprox(expected_avgA));
    EXPECT_TRUE(preintegrated.avgW.isApprox(expected_avgW));
  }
}
