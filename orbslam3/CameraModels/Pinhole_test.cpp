// 3rdparty
#include <gtest/gtest.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
// Local
#include "orbslam3/CameraModels/Pinhole.h"

TEST(Pinhole, Interface) {
  ORB_SLAM3::Pinhole ph({500.f, 500.f, 320.f, 240.f});

  // Setters and getters.
  EXPECT_EQ(ph.type(), ORB_SLAM3::GeometricCamera::Type::Pinhole);
  EXPECT_EQ(ph.getNumParams(), 4);
  EXPECT_EQ(ph.getParameter(0), 500.f);
  EXPECT_EQ(ph.getParameter(1), 500.f);
  EXPECT_EQ(ph.getParameter(2), 320.f);
  EXPECT_EQ(ph.getParameter(3), 240.f);

  ph.setParameter(5.0, 0);
  EXPECT_EQ(ph.getParameter(0), 5.0);
  ph.setParameter(500.f, 0);

  // Project and unproject.
  {
    cv::Point3f pt(1.f, 2.f, 1.f);
    cv::Point2f projected = ph.project(pt);
    EXPECT_EQ(projected.x, 820.f ); // fx * x / z + cx = 500 * 1 / 1 + 320 = 820
    EXPECT_EQ(projected.y, 1240.f); // fy * y / z + cy = 500 * 2 / 1 + 240 = 1240
  }
  {
    cv::Point2f pt(820.f, 1240.f);
    cv::Point3f unprojected = ph.unproject(pt);
    EXPECT_EQ(unprojected.x, 1.f); // (x - cx) / fx = (820 - 320) / 500 = 1
    EXPECT_EQ(unprojected.y, 2.f); // (y - cy) / fy = (1240 - 240) / 500 = 2
    EXPECT_EQ(unprojected.z, 1.f);
  }

  // Camera matrix K.
  {
    const Eigen::Matrix3f K = ph.K();
    EXPECT_EQ(K(0, 0), 500.f);
    EXPECT_EQ(K(1, 1), 500.f);
    EXPECT_EQ(K(0, 2), 320.f);
    EXPECT_EQ(K(1, 2), 240.f);
    EXPECT_EQ(K(2, 2), 1.f  );
  }

  // Equality check.
  {
    ORB_SLAM3::Pinhole other ({500.f, 500.f, 320.f, 240.f});
    EXPECT_TRUE(ph.isEqual(other));
  }

  // Epipolar constrain.
  {
    ORB_SLAM3::Pinhole camera_2 ({500.f, 500.f, 320.f, 240.f});
    const cv::KeyPoint keypoint_1({100.f, 200.f}, 1.f);
    const cv::KeyPoint keypoint_2({110.f, 210.f}, 1.f);
    const Eigen::Matrix3f R_12 = Eigen::Matrix3f::Identity(); // No rotation between cameras.
    const Eigen::Vector3f t_12 = Eigen::Vector3f(0.5f, 0.5f, 0.f);
    const float sigma_level = 1.f;
    const float uncertainty = 1.f;
    const bool success      = ph.checkEpipolarConstrain(
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
    boost::archive::text_oarchive oa(ss);
    oa << ph;

    ORB_SLAM3::Pinhole other;
    boost::archive::text_iarchive ia(ss);
    ia >> other;

    EXPECT_TRUE(ph.isEqual(other));
  }
}
