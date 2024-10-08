// 3rdparty
#include <gtest/gtest.h>
// Local
#include "orbslam3/GeometricTools.h"

using namespace ORB_SLAM3;

// Project a 3D point into a camera.
Eigen::Vector3f project(const Eigen::Vector3f& pt, const Eigen::Matrix<float, 3, 4>& P) {
  const Eigen::Vector4f x(pt.x(), pt.y(), pt.z(), 1.f); // Homogeneous coordinates
  const Eigen::Vector3f projected = P * x; // Project the point
  if (projected.z() == 0.f) {
    return Eigen::Vector3f(0.f, 0.f, 0.f); // Return the origin
  }
  return projected / projected.z(); // Normalize by the depth
}

TEST(GeometricTools, Triangulation) {
  // ──────────────────────────── //
  // Prepare the test.

  // Known 3D point.
  const Eigen::Vector3f point(1.f, 2.f, 5.f);
  // Camera 1 projection matrix.
  Eigen::Matrix<float, 3, 4> T_c1w;
  T_c1w.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // No rotation
  T_c1w.col(3)            = Eigen::Vector3f::Zero();     // No translation
  // Camera 2 projection matrix.
  Eigen::Matrix<float, 3, 4> T_c2w;
  T_c2w.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // No rotation
  T_c2w.col(3) = Eigen::Vector3f(-1.f, 0.f, 0.f); // Translation in x-axis
  // Project the 3D point into the cameras.
  const Eigen::Vector3f point_c1 = project(point, T_c1w);
  const Eigen::Vector3f point_c2 = project(point, T_c2w);
  EXPECT_NE(point_c1, Eigen::Vector3f(0.f, 0.f, 0.f));
  EXPECT_NE(point_c2, Eigen::Vector3f(0.f, 0.f, 0.f));

  // ──────────────────────────── //
  // Test triangulation.
  Eigen::Vector3f triangulated;
  const bool success = GeometricTools::triangulate(point_c1, point_c2, T_c1w, T_c2w, triangulated);

  // Expect the triangulation to be successful.
  EXPECT_TRUE(success);

  // Expect the triangulated point to be close to the original point.
  EXPECT_TRUE(triangulated.isApprox(point));
}
