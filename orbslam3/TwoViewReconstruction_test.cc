// Standard
#include <memory>
// 3rdparty
#include <gtest/gtest.h>
// Local
#include "orbslam3/TwoViewReconstruction.h"

namespace simulation {

std::vector<Eigen::Vector3f> constructPolygonVertices(
  const Eigen::Vector3f& centroid,
  const float side,
  const std::size_t num_sides
) {
  const float angle_increment = 2.f * M_PIf / static_cast<float>(num_sides);

  std::vector<Eigen::Vector3f> vertices;
  vertices.reserve(num_sides);
  for (std::size_t i = 0; i < num_sides; ++i) {
    const float angle = angle_increment * static_cast<float>(i);
    const float x = centroid.x() + side * std::cos(angle);
    const float y = centroid.y() + side * std::sin(angle);
    vertices.emplace_back(x, y, centroid.z());
  }
  return vertices;
}

std::vector<Eigen::Vector3f> projectPoints(
  const std::vector<Eigen::Vector3f>& points,
  const Eigen::Matrix3f& K,
  const Eigen::Matrix3f& R,
  const Eigen::Vector3f& t
) {
  std::vector<Eigen::Vector3f> projected_points;
  projected_points.reserve(points.size());
  for (const auto& point : points) {
    const Eigen::Vector3f projected = K * (R * point + t);
    if (projected.z() <= 0.f) {
      throw std::runtime_error("Point is behind the camera.");
    }
    const Eigen::Vector3f homogeneous = Eigen::Vector3f(
      projected.x() / projected.z(),
      projected.y() / projected.z(),
      1.f
    );
    projected_points.push_back(homogeneous);
  }
  return projected_points;
}

Sophus::SE3f computeTransformationMatrix(
  const Eigen::Matrix3f& R_1,
  const Eigen::Vector3f& t_1,
  const Eigen::Matrix3f& R_2,
  const Eigen::Vector3f& t_2
) {
  const Eigen::Matrix3f R_21 = R_2 * R_1.transpose();
  const Eigen::Vector3f t_21 = t_2 - R_21 * t_1;
  return Sophus::SE3f(R_21, t_21);
}

// Camera intrinsic parameters.
constexpr float f_x = 800.f; // Focal length in x
constexpr float f_y = 800.f; // Focal length in y
constexpr float c_x = 320.f; // Principal point x
constexpr float c_y = 240.f; // Principal point y
const Eigen::Matrix3f K
  = (Eigen::Matrix3f() << f_x, 0.f, c_x, 0.f, f_y, c_y, 0.f, 0.f, 1.f).finished();

// Camera extrinsic parameters for the 1st view.
const Eigen::Matrix3f R_1 = Eigen::Matrix3f::Identity();
const Eigen::Vector3f t_1 = Eigen::Vector3f::Zero();

// Camera extrinsic parameters for the 2nd view.
const Eigen::Matrix3f R_2 = Eigen::Matrix3f::Identity();
const Eigen::Vector3f t_2(1.f, 0.f, 0.f); // Translation along the x-axis by 1 meter

// Transformations from the 1st view to the 2nd view.
const Sophus::SE3f T_21 = computeTransformationMatrix(R_1, t_1, R_2, t_2);

// 3D polygon in the world frame.
constexpr float side = 1.f; // [m]
const Eigen::Vector3f centroid(0.f, 0.f, 1.f); // in the world frame

} // namespace simulation

class TwoViewReconstructionTest : public ::testing::Test {
protected:
  void SetUp() override {
    reconstructor_ = std::make_unique<ORB_SLAM3::TwoViewReconstruction>(
      simulation::K,
      1.f,
      200
    );
  }

  void TearDown() override {}

  std::unique_ptr<ORB_SLAM3::TwoViewReconstruction> reconstructor_;
};

TEST_F(TwoViewReconstructionTest, Success) {
  constexpr std::size_t num_sides = 100;

  // Construct a 3D polygon in the world frame.
  const auto vertices = simulation::constructPolygonVertices(
    simulation::centroid,
    simulation::side,
    num_sides
  );

  // Project the 3D polygon to the 1st view and 2nd view.
  const auto projected_points_1 = simulation::projectPoints(
    vertices,
    simulation::K,
    simulation::R_1,
    simulation::t_1
  );
  const auto projected_points_2 = simulation::projectPoints(
    vertices,
    simulation::K,
    simulation::R_2,
    simulation::t_2
  );

  // Initialize keypoints for the 1st and 2nd views.
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

  keypoints_1.reserve(projected_points_1.size());
  for (const auto& point : projected_points_1) {
    keypoints_1.emplace_back(point.x(), point.y(), 1.f);
  }

  keypoints_2.reserve(projected_points_2.size());
  for (const auto& point : projected_points_2) {
    keypoints_2.emplace_back(point.x(), point.y(), 1.f);
  }

  // Initialize matches between keypoints.
  std::vector<int> matches_12;
  matches_12.resize(keypoints_1.size());
  std::iota(matches_12.begin(), matches_12.end(), 0);

  // Reconstruct.
  Sophus::SE3f T_21;
  std::vector<cv::Point3f> points_3D;
  std::vector<bool> triangulated_flags;
  const bool success = reconstructor_->Reconstruct(
    keypoints_1,
    keypoints_2,
    matches_12,
    T_21,
    points_3D,
    triangulated_flags
  );

  // Check.
  EXPECT_TRUE(success);

  EXPECT_TRUE(T_21.matrix().isApprox(simulation::T_21.matrix(), 1e-6));
  EXPECT_EQ(points_3D.size(), num_sides);
  for (std::size_t i = 0; i < num_sides; ++i) {
    const auto& point  = points_3D[i];
    const auto& vertex = vertices[i];
    EXPECT_NEAR(point.x, vertex.x(), 1e-5);
    EXPECT_NEAR(point.y, vertex.y(), 1e-5);
    EXPECT_NEAR(point.z, vertex.z(), 1e-5);
  }

  EXPECT_EQ(triangulated_flags.size(), num_sides);
  for (const auto& flag : triangulated_flags) {
    EXPECT_TRUE(flag);
  }
}

TEST_F(TwoViewReconstructionTest, FailureEmptyKeypoints) {
  // TODO: handle edge case and implement the test.
}

TEST_F(TwoViewReconstructionTest, FailureEmptyMatches) {
  // TODO: handle edge case and implement the test.
}

TEST_F(TwoViewReconstructionTest, FailureInsufficientKeypoints) {
  // TODO: handle edge case and implement the test.
}

TEST_F(TwoViewReconstructionTest, FailureInsufficientMatches) {
  // TODO: handle edge case and implement the test.
}
