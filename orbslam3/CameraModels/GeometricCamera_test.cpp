// 3rdparty
#include <gtest/gtest.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
// Local
#include "orbslam3/CameraModels/GeometricCamera.h"

class MockGeometricCamera : public ORB_SLAM3::GeometricCamera {
public:
  MockGeometricCamera() : ORB_SLAM3::GeometricCamera() {}

  MockGeometricCamera(const std::vector<float>& params)
    : ORB_SLAM3::GeometricCamera(params) {
    id_   = 0;
    type_ = Type::Pinhole;
  }

  // Override all pure virtual methods.

  Eigen::Vector2f project(const Eigen::Vector3f& pt) const override {
    return Eigen::Vector2f();
  }
  cv::Point2f project(const cv::Point3f& pt) const override {
    return cv::Point2f();
  }
  Eigen::Vector3f unproject(const Eigen::Vector2f& pt) const override {
    return Eigen::Vector3f();
  }
  cv::Point3f unproject(const cv::Point2f& pt) const override {
    return cv::Point3f();
  }

  JacobianMatrix jacobian(const Eigen::Vector3f& pt) const override {
    return JacobianMatrix();
  }

  Eigen::Matrix3f K() const override {
    return Eigen::Matrix3f();
  }

  float uncertainty(const Eigen::Vector2f& pt) const override {
    return 0.0f;
  }

  bool reconstructFromTwoViews(
    // Inputs.
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2,
    const std::vector<int>& matches_12,
    // Outputs.
    Sophus::SE3f& T_21, // Transformation from camera 1 to camera 2
    std::vector<cv::Point3f>& points,
    std::vector<bool>& triangulated_flags // Corresponding to each point above
  ) override {
    return false;
  }

  bool checkEpipolarConstrain(
    const GeometricCamera& camera_2,
    const cv::KeyPoint& keypoint_1,
    const cv::KeyPoint& keypoint_2,
    const Eigen::Matrix3f& R_12, // Rotation from camera 1 to camera 2
    const Eigen::Vector3f& t_12, // Translation from camera 1 to camera 2
    const float sigma_level,
    const float uncertainty
  ) const override {
    return false;
  }

  bool triangulateKeyPoints(
    // Inputs.
    const GeometricCamera& camera_2,
    const cv::KeyPoint& keyPoint_1,
    const cv::KeyPoint& keypoint_2,
    const Eigen::Matrix3f& R_12, // Rotation from camera 1 to camera 2
    const Eigen::Vector3f& t_12, // Translation from camera 1 to camera 2
    const float sigma_level,
    const float uncertainty,
    // Outputs.
    Eigen::Vector3f& triangulated_point
  ) const override {
    return false;
  }
};

TEST(GeometricCamera, Interface) {
  MockGeometricCamera camera({1.0f, 2.0f, 3.0f, 4.0f});
  EXPECT_EQ(camera.id(), 0);
  EXPECT_EQ(camera.type(), ORB_SLAM3::GeometricCamera::Type::Pinhole);
  EXPECT_EQ(camera.getNumParams(), 4);
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

  MockGeometricCamera deserialized_camera;
  {
    boost::archive::text_iarchive ia(ss);
    ia >> deserialized_camera;
  }

  EXPECT_EQ(deserialized_camera.id(), 0);
  EXPECT_EQ(deserialized_camera.type(), ORB_SLAM3::GeometricCamera::Type::Pinhole);
  EXPECT_EQ(deserialized_camera.getNumParams(), 4);
  EXPECT_EQ(deserialized_camera.getParameter(0), 1.0);
  EXPECT_EQ(deserialized_camera.getParameter(1), 2.0);
  EXPECT_EQ(deserialized_camera.getParameter(2), 3.0);
  EXPECT_EQ(deserialized_camera.getParameter(3), 4.0);
}
