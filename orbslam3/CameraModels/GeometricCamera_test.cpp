// 3rdparty
#include <gtest/gtest.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
// Local
#include "orbslam3/CameraModels/GeometricCamera.h"

class MockGeometricCamera : public ORB_SLAM3::GeometricCamera {
public:
  MockGeometricCamera(const std::vector<float>& params)
    : ORB_SLAM3::GeometricCamera(params) {
    mnId   = 0;
    mnType = CAM_PINHOLE;
  }

  // Override all pure virtual methods.
  cv::Point2f project(const cv::Point3f& p3D) override {
    return cv::Point2f(0.0f, 0.0f);
  }
  Eigen::Vector2d project(const Eigen::Vector3d& v3D) override {
    return Eigen::Vector2d(0.0, 0.0);
  }
  Eigen::Vector2f project(const Eigen::Vector3f& v3D) override {
    return Eigen::Vector2f(0.0f, 0.0f);
  }
  Eigen::Vector2f projectMat(const cv::Point3f& p3D) override {
    return Eigen::Vector2f(0.0f, 0.0f);
  }
  float uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D) override {
    return 0.0f;
  }
  Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) override {
    return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  }
  cv::Point3f unproject(const cv::Point2f& p2D) override {
    return cv::Point3f(0.0f, 0.0f, 0.0f);
  }
  Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& v3D) override {
    return Eigen::Matrix<double, 2, 3>::Zero();
  }
  bool ReconstructWithTwoViews(
    const std::vector<cv::KeyPoint>& vKeys1,
    const std::vector<cv::KeyPoint>& vKeys2,
    const std::vector<int>& vMatches12,
    Sophus::SE3f& T21,
    std::vector<cv::Point3f>& vP3D,
    std::vector<bool>& vbTriangulated
  ) override {
    return false;
  }
  cv::Mat toK() override {
    return cv::Mat::zeros(3, 3, CV_32F);
  }
  Eigen::Matrix3f toK_() override {
    return Eigen::Matrix3f::Zero();
  }
  bool epipolarConstrain(
    GeometricCamera* otherCamera,
    const cv::KeyPoint& kp1,
    const cv::KeyPoint& kp2,
    const Eigen::Matrix3f& R12,
    const Eigen::Vector3f& t12,
    const float sigmaLevel,
    const float unc
  ) override {
    return false;
  }
  bool matchAndtriangulate(
    const cv::KeyPoint& kp1,
    const cv::KeyPoint& kp2,
    GeometricCamera* pOther,
    Sophus::SE3f& Tcw1,
    Sophus::SE3f& Tcw2,
    const float sigmaLevel1,
    const float sigmaLevel2,
    Eigen::Vector3f& x3Dtriangulated
  ) override {
    return false;
  }
};

TEST(GeometricCamera, Interface) {
  MockGeometricCamera camera({1.0f, 2.0f, 3.0f, 4.0f});
  EXPECT_EQ(camera.GetId(), 0);
  EXPECT_EQ(camera.GetType(), ORB_SLAM3::GeometricCamera::CAM_PINHOLE);
  EXPECT_EQ(camera.size(), 4);
  EXPECT_EQ(camera.getParameter(0), 1.0);
  EXPECT_EQ(camera.getParameter(1), 2.0);
  EXPECT_EQ(camera.getParameter(2), 3.0);
  EXPECT_EQ(camera.getParameter(3), 4.0);

  camera.setParameter(5.0f, 0);
  EXPECT_EQ(camera.getParameter(0), 5.0);
}

TEST(GeometricCamera, Serialization) {
  MockGeometricCamera camera({1.0f, 2.0f, 3.0f, 4.0f});

  std::stringstream ss;
  {
    boost::archive::text_oarchive oa(ss);
    oa << camera;
  }

  MockGeometricCamera deserialized_camera({});
  {
    boost::archive::text_iarchive ia(ss);
    ia >> deserialized_camera;
  }

  EXPECT_EQ(deserialized_camera.GetId(), 0);
  EXPECT_EQ(deserialized_camera.GetType(), static_cast<unsigned int>(ORB_SLAM3::GeometricCamera::CAM_PINHOLE));
  EXPECT_EQ(deserialized_camera.size(), 4);
  EXPECT_EQ(deserialized_camera.getParameter(0), 1.0);
  EXPECT_EQ(deserialized_camera.getParameter(1), 2.0);
  EXPECT_EQ(deserialized_camera.getParameter(2), 3.0);
  EXPECT_EQ(deserialized_camera.getParameter(3), 4.0);
}
