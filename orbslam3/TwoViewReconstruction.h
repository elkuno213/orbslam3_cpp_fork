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

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

// Standard
#include <utility>
#include <vector>
// 3rdparty
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <orbslam3/external/Sophus/sophus/se3.hpp>

namespace ORB_SLAM3 {

// Class handling the two-view reconstruction of the relative pose and 3D scene
// structure from two views. It uses the 8-point algorithm to estimate the
// fundamental/homography matrices and RANSAC to remove outliers.
class TwoViewReconstruction {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TwoViewReconstruction(
    const Eigen::Matrix3f& K,
    const float std_dev = 1.f,
    const std::size_t ransac_iterations = 200
  );

  // Computes in parallel a fundamental matrix and a homography.
  // Selects a model and tries to recover the motion and the structure from
  // motion.
  bool reconstruct(
    const std::vector<cv::KeyPoint>& keypoints_1, // keypoints from 1st view
    const std::vector<cv::KeyPoint>& keypoints_2, // keypoints from 2nd view
    const std::vector<int>& matches_12,           // matches from 2nd to 1st view
                                                  // (1st view index is key)
                                                  // (2nd view index is value)
    Sophus::SE3f& T_21,                           // transformation matrix from 1st to 2nd view
    std::vector<cv::Point3f>& points_3D,          // 3D points in the world frame
    std::vector<bool>& triangulated_flags         // corresponding flags for 3D points triangulated
  );

private:
  using Match   = std::pair<int, int>;
  using Matches = std::vector<Match>;

  // Estimate the homography matrix from the keypoints between two views using
  // the 8-point algorithm and RANSAC.
  void findHomography(
    std::vector<bool>& inliers, // inlier matches between 1st and 2nd views
    float& score,               // score of the homography matrix
    Eigen::Matrix3f& H_21       // homography matrix from 1st to 2nd view
  ) const;

  // Estimate the fundamental matrix from the keypoints between two views using
  // the 8-point algorithm and RANSAC.
  void findFundamental(
    std::vector<bool>& inliers, // inlier matches between 1st and 2nd views
    float& score,               // score of the fundamental matrix
    Eigen::Matrix3f& F_21       // fundamental matrix from 1st to 2nd view
  ) const;

  // Compute the homography matrix from the keypoints between two views.
  Eigen::Matrix3f computeH21(
    const std::vector<cv::Point2f>& points_1, // points in 1st view
    const std::vector<cv::Point2f>& points_2  // points in 2nd view
  ) const;

  // Using the 8-point algorithm to compute the fundamental matrix from the
  // keypoints between two views that satisfy the epipolar constraint:
  // x_2^T * F_21 * x_1 = 0.
  Eigen::Matrix3f computeF21(
    const std::vector<cv::Point2f>& points_1, // points in 1st view
    const std::vector<cv::Point2f>& points_2  // points in 2nd view
  ) const;

  // Evaluate how well the homography matrices fit the keypoints between two
  // views by computing the reprojection error and testing by chi-square test.
  float checkHomography(
    // Inputs.
    const Eigen::Matrix3f& H_21, // homography matrix from 1st to 2nd view
    const Eigen::Matrix3f& H_12, // homography matrix from 2nd to 1st view
    const float sigma,           // standard deviation to normalize the error
    // Outputs.
    std::vector<bool>& inliers   // inlier matches between 1st and 2nd views
  ) const;

  // Evaluate how well the fundamental matrix fits the keypoints between two
  // views by computing the reprojection error and testing by chi-square test.
  float checkFundamental(
    // Inputs.
    const Eigen::Matrix3f& F_21, // fundamental matrix from 1st to 2nd view
    const float sigma,           // standard deviation to normalize the error
    // Outputs.
    std::vector<bool>& inliers   // inlier matches between 1st and 2nd views
  ) const;

  // Reconstruct the transformation matrix from the homography matrix by
  // decomposing it into 8 possible motion hypotheses and evaluating them based
  // on the triangulation and parallax.
  bool reconstructH(
    // Inputs.
    const std::vector<bool>& inliers,     // inlier matches between 1st and 2nd views
    const Eigen::Matrix3f& H_21,          // homography matrix from 1st to 2nd view
    const Eigen::Matrix3f& K,             // camera intrinsic matrix
    const float min_parallax,             // min parallax to consider a valid reconstruction
    const int min_triangulated,           // min number of triangulated points for valid reconstruction
    // Outputs.
    Sophus::SE3f& T_21,                   // transformation matrix from 1st to 2nd view
    std::vector<cv::Point3f>& points_3D,  // 3D points in the world frame
    std::vector<bool>& triangulated_flags // corresponding flags for 3D points triangulated
  ) const;

  // Reconstruct the transformation matrix from the fundamental matrix by
  // computing the essential matrix, decomposing it into 4 possible motion
  // hypotheses and evaluating them based on the triangulation and parallax.
  bool reconstructF(
    // Inputs.
    const std::vector<bool>& inliers,     // inlier matches between 1st and 2nd views
    const Eigen::Matrix3f& F_21,          // fundamental matrix from 1st to 2nd view
    const Eigen::Matrix3f& K,             // camera intrinsic matrix
    const float min_parallax,             // min parallax to consider a valid reconstruction
    const int min_triangulated,           // min number of triangulated points for valid reconstruction
    // Outputs.
    Sophus::SE3f& T_21,                   // transformation matrix from 1st to 2nd view
    std::vector<cv::Point3f>& points_3D,  // 3D points in the world frame
    std::vector<bool>& triangulated_flags // corresponding flags for 3D points triangulated
  ) const;

  // Normalize the keypoints to reduce the effect of scaling and translation,
  // which helps to improve the numerical stability of computations.
  void normalize(
    // Inputs.
    const std::vector<cv::KeyPoint>& keypoints,  // keypoints to normalize
    // Outputs.
    std::vector<cv::Point2f>& normalized_points, // normalized keypoints
    Eigen::Matrix3f& T // transformation matrix applied to the keypoints
  ) const;

  // Check whether the rotation matrix and translation vector are valid by
  // verifying the parallax, depth and reprojection error.
  int checkRT(
    // Inputs.
    const Eigen::Matrix3f& R,                     // rotation matrix
    const Eigen::Vector3f& t,                     // translation vector
    const std::vector<cv::KeyPoint>& keypoints_1, // keypoints from 1st view
    const std::vector<cv::KeyPoint>& keypoints_2, // keypoints from 2nd view
    const Matches& matches_12,                    // corresponding matches between 1st and 2nd views
    const std::vector<bool>& inliers,             // inlier matches
    const Eigen::Matrix3f& K,                     // camera intrinsic matrix
    const float thresh_squared,                   // squared threshold to check reprojection error
    // Outputs.
    std::vector<cv::Point3f>& points_3D,          // 3D points in the world frame
    std::vector<bool>& good_flags,                // resulting flags for valid reconstructions
    float& parallax                               // resulting parallax
  ) const;

  // Decompose Essential Matrix E into rotation matrices R_1, R_2 and
  // translation vector t, using SVD.
  void decomposeE(
    // Inputs.
    const Eigen::Matrix3f& E, // essential matrix
    // Outputs.
    Eigen::Matrix3f& R_1,     // rotation matrix 1
    Eigen::Matrix3f& R_2,     // rotation matrix 2
    Eigen::Vector3f& t        // translation vector
  ) const;

private:
  // Keypoints from 1st view (Reference Frame).
  std::vector<cv::KeyPoint> keypoints_1_;
  // Keypoints from 2nd view (Current Frame).
  std::vector<cv::KeyPoint> keypoints_2_;
  // Current matches from Reference frame to Current frame.
  Matches matches_;
  // Camera intrinsic matrix.
  Eigen::Matrix3f K_;
  // Standard deviation and variance.
  float std_dev_, var_;
  // Ransac max iterations.
  std::size_t ransac_iterations_;
  // Sets for RANSAC iterations, each set contains 8 indices of matches.
  std::vector<std::vector<std::size_t>> ransac_sets_;
};

} // namespace ORB_SLAM3

#endif // TwoViewReconstruction_H
