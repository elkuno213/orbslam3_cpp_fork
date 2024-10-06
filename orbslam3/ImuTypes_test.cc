// Standard
#include <sstream>
// 3rdparty
#include <gtest/gtest.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
// Local
#include "orbslam3/ImuTypes.h"

TEST(Point, Constructor) {
  // Default constructor
  {
    ORB_SLAM3::IMU::Point point;
    EXPECT_EQ(point.acc.x() , 0.f);
    EXPECT_EQ(point.acc.y() , 0.f);
    EXPECT_EQ(point.acc.z() , 0.f);
    EXPECT_EQ(point.gyro.x(), 0.f);
    EXPECT_EQ(point.gyro.y(), 0.f);
    EXPECT_EQ(point.gyro.z(), 0.f);
    EXPECT_EQ(point.t, 0.0);
  }
  // Constructor with float values
  {
    const float ax = 0.1;
    const float ay = 0.2;
    const float az = 0.3;
    const float wx = 0.4;
    const float wy = 0.5;
    const float wz = 0.6;
    const double t = 1.0;
    const ORB_SLAM3::IMU::Point point(ax, ay, az, wx, wy, wz, t);
    EXPECT_EQ(point.acc.x() , ax);
    EXPECT_EQ(point.acc.y() , ay);
    EXPECT_EQ(point.acc.z() , az);
    EXPECT_EQ(point.gyro.x(), wx);
    EXPECT_EQ(point.gyro.y(), wy);
    EXPECT_EQ(point.gyro.z(), wz);
    EXPECT_EQ(point.t, t);
  }
  // Constructor with Eigen::Vector3f
  {
    const float ax = 0.1;
    const float ay = 0.2;
    const float az = 0.3;
    const float wx = 0.4;
    const float wy = 0.5;
    const float wz = 0.6;
    const Eigen::Vector3f acc(ax, ay, az);
    const Eigen::Vector3f gyro(wx, wy, wz);
    const double t = 1.0;
    const ORB_SLAM3::IMU::Point point(acc, gyro, t);
    EXPECT_EQ(point.acc.x() , ax);
    EXPECT_EQ(point.acc.y() , ay);
    EXPECT_EQ(point.acc.z() , az);
    EXPECT_EQ(point.gyro.x(), wx);
    EXPECT_EQ(point.gyro.y(), wy);
    EXPECT_EQ(point.gyro.z(), wz);
    EXPECT_EQ(point.t, t);
  }
  // Constructor with cv::Point3f
  {
    const float ax = 0.1;
    const float ay = 0.2;
    const float az = 0.3;
    const float wx = 0.4;
    const float wy = 0.5;
    const float wz = 0.6;
    const cv::Point3f acc(ax, ay, az);
    const cv::Point3f gyro(wx, wy, wz);
    const double t = 1.0;
    const ORB_SLAM3::IMU::Point point(acc, gyro, t);
    EXPECT_EQ(point.acc.x() , ax);
    EXPECT_EQ(point.acc.y() , ay);
    EXPECT_EQ(point.acc.z() , az);
    EXPECT_EQ(point.gyro.x(), wx);
    EXPECT_EQ(point.gyro.y(), wy);
    EXPECT_EQ(point.gyro.z(), wz);
    EXPECT_EQ(point.t, t);
  }
}

TEST(Point, Serialization) {
  const Eigen::Vector3f acc(0.1f, 0.2f, 0.3f);
  const Eigen::Vector3f gyro(0.4f, 0.5f, 0.6f);
  const double t = 1.0;
  const ORB_SLAM3::IMU::Point point(acc, gyro, t);

  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << point;
  ORB_SLAM3::IMU::Point loaded;
  boost::archive::text_iarchive ia(ss);
  ia >> loaded;

  EXPECT_EQ(loaded.acc.x() , acc.x());
  EXPECT_EQ(loaded.acc.y() , acc.y());
  EXPECT_EQ(loaded.acc.z() , acc.z());
  EXPECT_EQ(loaded.gyro.x(), gyro.x());
  EXPECT_EQ(loaded.gyro.y(), gyro.y());
  EXPECT_EQ(loaded.gyro.z(), gyro.z());
}

TEST(Bias, Constructor) {
  // Default constructor
  {
    ORB_SLAM3::IMU::Bias bias;
    EXPECT_EQ(bias.ax, 0.f);
    EXPECT_EQ(bias.ay, 0.f);
    EXPECT_EQ(bias.az, 0.f);
    EXPECT_EQ(bias.wx, 0.f);
    EXPECT_EQ(bias.wy, 0.f);
    EXPECT_EQ(bias.wz, 0.f);
  }
  // Constructor with float values
  {
    const float bias_ax = 0.1;
    const float bias_ay = 0.2;
    const float bias_az = 0.3;
    const float bias_wx = 0.4;
    const float bias_wy = 0.5;
    const float bias_wz = 0.6;
    const ORB_SLAM3::IMU::Bias bias(bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz);
    EXPECT_EQ(bias.ax, bias_ax);
    EXPECT_EQ(bias.ay, bias_ay);
    EXPECT_EQ(bias.az, bias_az);
    EXPECT_EQ(bias.wx, bias_wx);
    EXPECT_EQ(bias.wy, bias_wy);
    EXPECT_EQ(bias.wz, bias_wz);
  }
}

TEST(Bias, Subtraction) {
  const float bias_ax_1 = 0.1;
  const float bias_ay_1 = 0.2;
  const float bias_az_1 = 0.3;
  const float bias_wx_1 = 0.4;
  const float bias_wy_1 = 0.5;
  const float bias_wz_1 = 0.6;
  const ORB_SLAM3::IMU::Bias bias_1(bias_ax_1, bias_ay_1, bias_az_1, bias_wx_1, bias_wy_1, bias_wz_1);

  const float bias_ax_2 = 0.01;
  const float bias_ay_2 = 0.02;
  const float bias_az_2 = 0.03;
  const float bias_wx_2 = 0.04;
  const float bias_wy_2 = 0.05;
  const float bias_wz_2 = 0.06;
  const ORB_SLAM3::IMU::Bias bias_2(bias_ax_2, bias_ay_2, bias_az_2, bias_wx_2, bias_wy_2, bias_wz_2);

  const ORB_SLAM3::IMU::Bias bias_diff = bias_1 - bias_2;
  EXPECT_EQ(bias_diff.ax, bias_ax_1 - bias_ax_2);
  EXPECT_EQ(bias_diff.ay, bias_ay_1 - bias_ay_2);
  EXPECT_EQ(bias_diff.az, bias_az_1 - bias_az_2);
  EXPECT_EQ(bias_diff.wx, bias_wx_1 - bias_wx_2);
  EXPECT_EQ(bias_diff.wy, bias_wy_1 - bias_wy_2);
  EXPECT_EQ(bias_diff.wz, bias_wz_1 - bias_wz_2);
}

TEST(Bias, CopyFrom) {
  const float bias_ax = 0.1;
  const float bias_ay = 0.2;
  const float bias_az = 0.3;
  const float bias_wx = 0.4;
  const float bias_wy = 0.5;
  const float bias_wz = 0.6;

  ORB_SLAM3::IMU::Bias bias(bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz);
  ORB_SLAM3::IMU::Bias copied;
  copied.copyFrom(bias);

  EXPECT_EQ(copied.ax, bias_ax);
  EXPECT_EQ(copied.ay, bias_ay);
  EXPECT_EQ(copied.az, bias_az);
  EXPECT_EQ(copied.wx, bias_wx);
  EXPECT_EQ(copied.wy, bias_wy);
  EXPECT_EQ(copied.wz, bias_wz);
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
  ORB_SLAM3::IMU::Bias loaded;
  boost::archive::text_iarchive ia(ss);
  ia >> loaded;

  EXPECT_EQ(loaded.ax, bias_ax);
  EXPECT_EQ(loaded.ay, bias_ay);
  EXPECT_EQ(loaded.az, bias_az);
  EXPECT_EQ(loaded.wx, bias_wx);
  EXPECT_EQ(loaded.wy, bias_wy);
  EXPECT_EQ(loaded.wz, bias_wz);
}

class CalibTest : public ::testing::Test {
protected:
  void SetUp() override {
    const Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
    T_bc = Sophus::SE3f(q, t);

    noise_gyro       = 0.1;
    noise_acc        = 0.2;
    random_walk_gyro = 0.3;
    random_walk_acc  = 0.4;

    calib = ORB_SLAM3::IMU::Calib(T_bc, noise_gyro, noise_acc, random_walk_gyro, random_walk_acc);
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
  Sophus::SE3f T_bc;
  float noise_gyro;
  float noise_acc;
  float random_walk_gyro;
  float random_walk_acc;
};

TEST_F(CalibTest, Constructor) {
  // Default constructor
  {
    ORB_SLAM3::IMU::Calib calib;
    EXPECT_FALSE(calib.is_set);
    EXPECT_EQ(calib.T_bc.matrix(), Sophus::SE3f().matrix());
    EXPECT_EQ(calib.T_cb.matrix(), Sophus::SE3f().matrix());
    EXPECT_EQ(calib.noise_cov.diagonal().isZero(), true);
    EXPECT_EQ(calib.walk_cov.diagonal().isZero() , true);
  }

  // Constructor
  {
    const float noise_gyro_squared       = noise_gyro * noise_gyro;
    const float noise_acc_squared        = noise_acc * noise_acc;
    const float random_walk_gyro_squared = random_walk_gyro * random_walk_gyro;
    const float random_walk_acc_squared  = random_walk_acc * random_walk_acc;

    EXPECT_EQ(calib.T_bc.matrix(), T_bc.matrix());
    EXPECT_EQ(calib.noise_cov.diagonal()(0), noise_gyro_squared      );
    EXPECT_EQ(calib.noise_cov.diagonal()(1), noise_gyro_squared      );
    EXPECT_EQ(calib.noise_cov.diagonal()(2), noise_gyro_squared      );
    EXPECT_EQ(calib.noise_cov.diagonal()(3), noise_acc_squared       );
    EXPECT_EQ(calib.noise_cov.diagonal()(4), noise_acc_squared       );
    EXPECT_EQ(calib.noise_cov.diagonal()(5), noise_acc_squared       );
    EXPECT_EQ(calib.walk_cov.diagonal()(0) , random_walk_gyro_squared);
    EXPECT_EQ(calib.walk_cov.diagonal()(1) , random_walk_gyro_squared);
    EXPECT_EQ(calib.walk_cov.diagonal()(2) , random_walk_gyro_squared);
    EXPECT_EQ(calib.walk_cov.diagonal()(3) , random_walk_acc_squared );
    EXPECT_EQ(calib.walk_cov.diagonal()(4) , random_walk_acc_squared );
    EXPECT_EQ(calib.walk_cov.diagonal()(5) , random_walk_acc_squared );
  }
}

TEST_F(CalibTest, Serialization) {
  ORB_SLAM3::IMU::Calib loaded;
  Archive(loaded);

  const float noise_gyro_squared       = noise_gyro * noise_gyro;
  const float noise_acc_squared        = noise_acc * noise_acc;
  const float random_walk_gyro_squared = random_walk_gyro * random_walk_gyro;
  const float random_walk_acc_squared  = random_walk_acc * random_walk_acc;

  EXPECT_EQ(loaded.T_bc.matrix(), T_bc.matrix());
  EXPECT_EQ(loaded.noise_cov.diagonal()(0), noise_gyro_squared      );
  EXPECT_EQ(loaded.noise_cov.diagonal()(1), noise_gyro_squared      );
  EXPECT_EQ(loaded.noise_cov.diagonal()(2), noise_gyro_squared      );
  EXPECT_EQ(loaded.noise_cov.diagonal()(3), noise_acc_squared       );
  EXPECT_EQ(loaded.noise_cov.diagonal()(4), noise_acc_squared       );
  EXPECT_EQ(loaded.noise_cov.diagonal()(5), noise_acc_squared       );
  EXPECT_EQ(loaded.walk_cov.diagonal()(0) , random_walk_gyro_squared);
  EXPECT_EQ(loaded.walk_cov.diagonal()(1) , random_walk_gyro_squared);
  EXPECT_EQ(loaded.walk_cov.diagonal()(2) , random_walk_gyro_squared);
  EXPECT_EQ(loaded.walk_cov.diagonal()(3) , random_walk_acc_squared );
  EXPECT_EQ(loaded.walk_cov.diagonal()(4) , random_walk_acc_squared );
  EXPECT_EQ(loaded.walk_cov.diagonal()(5) , random_walk_acc_squared );
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

  EXPECT_TRUE(integrated_rotation.dR.isApprox(expected_deltaR));
  EXPECT_TRUE(integrated_rotation.rightJacobian.isApprox(expected_rightJ));
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
  const Eigen::Matrix3f expected_deltaR
    = I
    + W * std::sin(d) / d
    + W_squared * (1.0f - std::cos(d)) / d_squared;
  const Eigen::Matrix3f expected_rightJ
    = I
    - W * (1.0f - std::cos(d)) / d_squared
    + W_squared * (d - std::sin(d)) / (d_squared * d);

  EXPECT_TRUE(integrated_rotation.dR.isApprox(expected_deltaR));
  EXPECT_TRUE(integrated_rotation.rightJacobian.isApprox(expected_rightJ));
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
    const Sophus::SE3f T_bc(q, t);
    const float noise_gyro       = 0.1;
    const float noise_acc        = 0.2;
    const float random_walk_gyro = 0.3;
    const float random_walk_acc  = 0.4;
    calib = ORB_SLAM3::IMU::Calib(T_bc, noise_gyro, noise_acc, random_walk_gyro, random_walk_acc);
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
  ORB_SLAM3::IMU::Preintegrated preintegrated(bias, calib);

  EXPECT_EQ(preintegrated.bias.ax, bias.ax);
  EXPECT_EQ(preintegrated.bias.ay, bias.ay);
  EXPECT_EQ(preintegrated.bias.az, bias.az);
  EXPECT_EQ(preintegrated.bias.wx, bias.wx);
  EXPECT_EQ(preintegrated.bias.wy, bias.wy);
  EXPECT_EQ(preintegrated.bias.wz, bias.wz);
  EXPECT_EQ(preintegrated.noise_cov.diagonal(), calib.noise_cov.diagonal());
  EXPECT_EQ(preintegrated.walk_cov.diagonal() , calib.walk_cov.diagonal() );

  ORB_SLAM3::IMU::Preintegrated copied(&preintegrated);
  EXPECT_EQ(copied.bias.ax, bias.ax);
  EXPECT_EQ(copied.bias.ay, bias.ay);
  EXPECT_EQ(copied.bias.az, bias.az);
  EXPECT_EQ(copied.bias.wx, bias.wx);
  EXPECT_EQ(copied.bias.wy, bias.wy);
  EXPECT_EQ(copied.bias.wz, bias.wz);
  EXPECT_EQ(copied.noise_cov.diagonal(), calib.noise_cov.diagonal());
  EXPECT_EQ(copied.walk_cov.diagonal() , calib.walk_cov.diagonal() );
}

TEST_F(PreintegratedTest, Serialization) {
  ORB_SLAM3::IMU::Preintegrated loaded;
  Archive(loaded);

  EXPECT_EQ(loaded.bias.ax, bias.ax);
  EXPECT_EQ(loaded.bias.ay, bias.ay);
  EXPECT_EQ(loaded.bias.az, bias.az);
  EXPECT_EQ(loaded.bias.wx, bias.wx);
  EXPECT_EQ(loaded.bias.wy, bias.wy);
  EXPECT_EQ(loaded.bias.wz, bias.wz);
  EXPECT_EQ(loaded.noise_cov.diagonal(), calib.noise_cov.diagonal());
  EXPECT_EQ(loaded.walk_cov.diagonal() , calib.walk_cov.diagonal());
}

TEST_F(PreintegratedTest, Integration) {
  ORB_SLAM3::IMU::Preintegrated preintegrated(bias, calib);

  // Integrate new measurement.
  {
    // Integrate.
    const Eigen::Vector3f acc(0.1, 0.0, 0.1);
    const Eigen::Vector3f gyro(0.1, 0.0, 0.1);
    const float dt = 0.1f;
    preintegrated.integrateNewMeasurement(acc, gyro, dt);

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
    EXPECT_NEAR(preintegrated.t, 0.1f, 1e-6f);
    EXPECT_TRUE(preintegrated.C.isApprox(expected_C));
    EXPECT_TRUE(preintegrated.info.isApprox(expected_Info));
    EXPECT_TRUE(preintegrated.dR.isApprox(expected_dR));
    EXPECT_TRUE(preintegrated.dV.isApprox(expected_dV));
    EXPECT_TRUE(preintegrated.dP.isApprox(expected_dP));
    EXPECT_TRUE(preintegrated.JR_gyro.isApprox(expected_JRg));
    EXPECT_TRUE(preintegrated.JV_gyro.isApprox(expected_JVg));
    EXPECT_TRUE(preintegrated.JV_acc.isApprox(expected_JVa));
    EXPECT_TRUE(preintegrated.JP_gyro.isApprox(expected_JPg));
    EXPECT_TRUE(preintegrated.JP_acc.isApprox(expected_JPa));
    EXPECT_TRUE(preintegrated.mean_acc.isApprox(expected_avgA));
    EXPECT_TRUE(preintegrated.mean_gyro.isApprox(expected_avgW));
  }
}
