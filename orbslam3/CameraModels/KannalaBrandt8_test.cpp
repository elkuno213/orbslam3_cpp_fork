// 3rdparty
#include <gtest/gtest.h>
// Local
#include "orbslam3/CameraModels/KannalaBrandt8.h"

TEST(KannalaBrandt8, Interface) {
  const std::vector<float> params
    = {300.f, 300.f, 320.f, 240.f, 0.1f, 0.01f, 1e-3, 1e-4};

  ORB_SLAM3::KannalaBrandt8* kb = new ORB_SLAM3::KannalaBrandt8(params);

  // Setters and getters.
  EXPECT_EQ(kb->GetType(), static_cast<unsigned int>(ORB_SLAM3::GeometricCamera::CAM_FISHEYE));
  EXPECT_EQ(kb->size(), 8);
  EXPECT_EQ(kb->getParameter(0), 300.f);
  EXPECT_EQ(kb->getParameter(1), 300.f);
  EXPECT_EQ(kb->getParameter(2), 320.f);
  EXPECT_EQ(kb->getParameter(3), 240.f);
  EXPECT_EQ(kb->getParameter(4), 0.1f);
  EXPECT_EQ(kb->getParameter(5), 0.01f);
  EXPECT_EQ(kb->getParameter(6), 1e-3f);
  EXPECT_EQ(kb->getParameter(7), 1e-4f);

  kb->setParameter(5.0, 0);
  EXPECT_EQ(kb->getParameter(0), 5.0);
  kb->setParameter(300.f, 0);

  // Project and unproject.
  {
    cv::Point3f pt(1.f, 1.f, 1.f);
    cv::Point2f projected = kb->project(pt);
    EXPECT_NEAR(projected.x, 543.004f, 1e-3);
    EXPECT_NEAR(projected.y, 463.004f, 1e-3);
  }
  {
    cv::Point2f pt(543.004f, 463.004f);
    cv::Point3f unprojected = kb->unproject(pt);
    EXPECT_NEAR(unprojected.x, 1.f, 1e-3);
    EXPECT_NEAR(unprojected.y, 1.f, 1e-3);
    EXPECT_NEAR(unprojected.z, 1.f, 1e-3);
  }

  // Camera matrix K.
  {
    cv::Mat K = kb->toK();
    EXPECT_EQ(K.at<float>(0, 0), 300.f);
    EXPECT_EQ(K.at<float>(1, 1), 300.f);
    EXPECT_EQ(K.at<float>(0, 2), 320.f);
    EXPECT_EQ(K.at<float>(1, 2), 240.f);
    EXPECT_EQ(K.at<float>(2, 2), 1.f);
  }

  // Equality check.
  {
    ORB_SLAM3::KannalaBrandt8* other = new ORB_SLAM3::KannalaBrandt8(params);
    EXPECT_TRUE(kb->IsEqual(other));
    delete other;
  }

  // Triangulate.
  {
    ORB_SLAM3::KannalaBrandt8* camera_2 = new ORB_SLAM3::KannalaBrandt8(params);
    const Eigen::Matrix3f R_12
      = Eigen::AngleAxisf(-5.f * M_PI / 180.f, Eigen::Vector3f::UnitY()).toRotationMatrix();
    const Eigen::Vector3f t_12 = Eigen::Vector3f(0.5f, 0.f, 0.f);
    const cv::KeyPoint keypoint_1({346.f, 240.f}, 1.f);
    const cv::KeyPoint keypoint_2({320.f, 240.f}, 1.f);
    const float sigma_level = 1.f;
    const float uncertainty = 1.f;
    Eigen::Vector3f triangulated;
    const float z = kb->TriangulateMatches(
      camera_2,
      keypoint_1,
      keypoint_2,
      R_12,
      t_12,
      sigma_level,
      uncertainty,
      triangulated
    );
    EXPECT_NEAR(triangulated.x(), 0.249f, 1e-3);
    EXPECT_NEAR(triangulated.y(),    0.f, 1e-3);
    EXPECT_NEAR(triangulated.z(), 2.868f, 1e-3);
  }

  // Epipolar constrain.
  {
    ORB_SLAM3::KannalaBrandt8* camera_2 = new ORB_SLAM3::KannalaBrandt8(params);
    const Eigen::Matrix3f R_12
      = Eigen::AngleAxisf(-5.f * M_PI / 180.f, Eigen::Vector3f::UnitY()).toRotationMatrix();
    const Eigen::Vector3f t_12 = Eigen::Vector3f(0.5f, 0.f, 0.f);
    const cv::KeyPoint keypoint_1({346.f, 240.f}, 1.f);
    const cv::KeyPoint keypoint_2({320.f, 240.f}, 1.f);
    const float sigma_level = 1.f;
    const float uncertainty = 1.f;
    const bool success      = kb->epipolarConstrain(
      camera_2,
      keypoint_1,
      keypoint_2,
      R_12,
      t_12,
      sigma_level,
      uncertainty
    );
    EXPECT_TRUE(success);
  }

  // Serialization.
  {
    std::stringstream ss;
    ss << *kb;
    ORB_SLAM3::KannalaBrandt8* other = new ORB_SLAM3::KannalaBrandt8();
    ss >> *other;
    EXPECT_TRUE(kb->IsEqual(other));
    delete other;
  }
}
