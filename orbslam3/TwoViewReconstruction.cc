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

// Standard
#include <numeric>
#include <thread>
// 3rdparty
#include <glog/logging.h>
#include <orbslam3/external/DBoW2/DUtils/Random.h>
// Local
#include "orbslam3/Converter.h"
#include "orbslam3/GeometricTools.h"
#include "orbslam3/TwoViewReconstruction.h"

namespace ORB_SLAM3 {

// TODO: consider adding kMinMatches parameter.
// TODO: consider parameterizing these values.
// Constants.
constexpr float kMinParallaxCosine = 0.99998f;
constexpr float kMinParallax = 1.f; // [degrees]
constexpr int kMinNumTriangulated = 50; // minimum number of triangulated points
constexpr float kMinSingularValueRatio = 1.00001f; // minimum ratio between singular values

TwoViewReconstruction::TwoViewReconstruction(
  const Eigen::Matrix3f& K,
  const float std_dev,
  const std::size_t ransac_iterations
)
  : K_(K)
  , std_dev_(std_dev)
  , var_(std_dev * std_dev)
  , ransac_iterations_(ransac_iterations)
{}

bool TwoViewReconstruction::reconstruct(
  const std::vector<cv::KeyPoint>& keypoints_1,
  const std::vector<cv::KeyPoint>& keypoints_2,
  const std::vector<int>& matches_12,
  Sophus::SE3f& T_21,
  std::vector<cv::Point3f>& points_3D,
  std::vector<bool>& triangulated_flags
) {
  if (keypoints_1.empty() || keypoints_2.empty() || matches_12.empty()) {
    LOG(WARNING) << "Empty keypoints or matches, cannot reconstruct.";
    T_21 = Sophus::SE3f();
    points_3D.clear();
    triangulated_flags.clear();
    return false;
  }

  // ──────────────────────────── //
  // Prepare data

  // Initialize keypoints with the new ones.
  keypoints_1_ = keypoints_1;
  keypoints_2_ = keypoints_2;

  // Convert matches to a vector of pairs.
  matches_.clear();
  matches_.reserve(std::min({keypoints_1_.size(), keypoints_2_.size(), matches_12.size()}));
  for (std::size_t i = 0; i < matches_12.size(); i++) {
    const std::size_t i_1 = i;
    const std::size_t i_2 = matches_12[i];
    // Skip invalid matches.
    if (i_1 >= keypoints_1_.size() || i_2 >= keypoints_2_.size() || i_2 < 0) {
      continue;
    }
    // Add the valid match.
    matches_.push_back(std::make_pair(i_1, i_2));
  }

  if (matches_.empty()) {
    LOG(WARNING) << "Empty matches after filtering, cannot reconstruct.";
    T_21 = Sophus::SE3f();
    points_3D.clear();
    triangulated_flags.clear();
    return false;
  }

  const std::size_t num_matches = matches_.size();

  // ──────────────────────────── //
  // Prepare RANSAC

  // Indices for minimum set selection.
  std::vector<std::size_t> available_indices;
  std::vector<std::size_t> all_indices;
  all_indices.resize(num_matches);
  std::iota(all_indices.begin(), all_indices.end(), 0);

  // Prepare sets of 8 indices for RANSAC iterations.
  ransac_sets_ = std::vector<std::vector<std::size_t>>(
    ransac_iterations_,
    std::vector<std::size_t>(8, 0)
  );

  // Seed the random number generator.
  // TODO: why using DUtils::Random instead of std::random?
  DUtils::Random::SeedRandOnce(0);

  // Generate sets.
  for (std::size_t it = 0; it < ransac_iterations_; it++) {
    available_indices = all_indices;

    // Select a minimum set.
    for (std::size_t j = 0; j < 8; j++) {
      // Randomly select an index.
      const int rand = DUtils::Random::RandomInt(0, available_indices.size() - 1);
      const std::size_t indice = available_indices[rand];
      // Save the index.
      ransac_sets_[it][j] = indice;
      // Remove the index from the available indices by replacing it with the
      // last one and removing the last one.
      available_indices[rand] = available_indices.back();
      available_indices.pop_back();
    }
  }

  // ──────────────────────────── //
  // Parallel computation of the fundamental matrix and homography matrix

  // Launch threads to compute in parallel fundamental and homography matrices.
  std::vector<bool> inliers_H, inliers_F;
  float score_H, score_F;
  Eigen::Matrix3f H, F;

  std::thread thread_H(
    &TwoViewReconstruction::findHomography,
    this,
    std::ref(inliers_H),
    std::ref(score_H),
    std::ref(H)
  );
  std::thread thread_F(
    &TwoViewReconstruction::findFundamental,
    this,
    std::ref(inliers_F),
    std::ref(score_F),
    std::ref(F)
  );

  // Wait until both threads have finished.
  thread_H.join();
  thread_F.join();

  // ──────────────────────────── //
  // Evaluate scores and select the best model

  // Compute ratio of scores.
  if (score_H + score_F == 0.f) {
    LOG(WARNING) << "Both scores are zero, cannot reconstruct.";
    return false;
  }

  // Select the best model based on the ratio of scores.
  const float ratio_H = score_H / (score_H + score_F);
  if (ratio_H > 0.5f) {
    LOG(INFO) << "Reconstructing with homography matrix.";
    return reconstructH(
      inliers_H,
      H,
      K_,
      kMinParallax,
      kMinNumTriangulated,
      T_21,
      points_3D,
      triangulated_flags
    );
  } else {
    LOG(INFO) << "Reconstructing with fundamental matrix.";
    return reconstructF(
      inliers_F,
      F,
      K_,
      kMinParallax,
      kMinNumTriangulated,
      T_21,
      points_3D,
      triangulated_flags
    );
  }
}

void TwoViewReconstruction::findHomography(
  std::vector<bool>& inliers,
  float& score,
  Eigen::Matrix3f& H_21
) const {
  // Number of putative matches.
  const std::size_t num_matches = matches_.size();

  // Initialize.
  inliers = std::vector<bool>(num_matches, false);
  score   = 0.f;

  // Normalize keypoints to improve numerical stability.
  std::vector<cv::Point2f> normalized_points_1, normalized_points_2;
  Eigen::Matrix3f T_1, T_2;
  normalize(keypoints_1_, normalized_points_1, T_1);
  normalize(keypoints_2_, normalized_points_2, T_2);
  const Eigen::Matrix3f T_2_inv = T_2.inverse();

  // Iteration variables.
  std::vector<cv::Point2f> set_1(8);
  std::vector<cv::Point2f> set_2(8);
  std::vector<bool> current_inliers(num_matches, false);
  float current_score;
  Eigen::Matrix3f current_H_21;

  // Perform all RANSAC iterations and save the solution with highest score.
  for (std::size_t it = 0; it < ransac_iterations_; it++) {
    // Select a minimum set of 8 points.
    for (std::size_t j = 0; j < 8; j++) {
      const std::size_t indice = ransac_sets_[it][j];
      set_1[j] = normalized_points_1[matches_[indice].first ];
      set_2[j] = normalized_points_2[matches_[indice].second];
    }

    // Compute a candidate of the homography matrix.
    Eigen::Matrix3f H_normalized = computeH21(set_1, set_2);
    if (H_normalized.isZero()) {
      continue;
    }
    // Denormalize the homography matrix.
    current_H_21 = T_2_inv * H_normalized * T_1;

    // Check the quality of the homography matrix and update the best solution.
    current_score = checkHomography(
      current_H_21,
      current_H_21.inverse(),
      std_dev_,
      current_inliers
    );
    if (current_score > score) {
      inliers = current_inliers;
      score   = current_score;
      H_21    = current_H_21;
    }
  }
}

void TwoViewReconstruction::findFundamental(
  std::vector<bool>& inliers,
  float& score,
  Eigen::Matrix3f& F_21
) const {
  // Number of putative matches.
  const std::size_t num_matches = matches_.size();

  // Initialize.
  inliers = std::vector<bool>(num_matches, false);
  score   = 0.f;

  // Normalize keypoints to improve numerical stability.
  std::vector<cv::Point2f> normalized_points_1, normalized_points_2;
  Eigen::Matrix3f T_1, T_2;
  normalize(keypoints_1_, normalized_points_1, T_1);
  normalize(keypoints_2_, normalized_points_2, T_2);
  const Eigen::Matrix3f T_2_t = T_2.transpose();

  // Iteration variables.
  std::vector<cv::Point2f> set_1(8); // a set of 8 points in the 1st view
  std::vector<cv::Point2f> set_2(8); // a set of 8 points in the 2nd view
  std::vector<bool> current_inliers(num_matches, false);
  float current_score;
  Eigen::Matrix3f current_F_21;

  // Perform all RANSAC iterations and save the solution with highest score.
  for (std::size_t it = 0; it < ransac_iterations_; it++) {
    // Select a minimum set of 8 points.
    for (std::size_t j = 0; j < 8; j++) {
      const std::size_t indice = ransac_sets_[it][j];
      set_1[j] = normalized_points_1[matches_[indice].first ];
      set_2[j] = normalized_points_2[matches_[indice].second];
    }

    // Compute a candidate of the fundamental matrix.
    const Eigen::Matrix3f F_normalized = computeF21(set_1, set_2);
    if (F_normalized.isZero()) {
      continue;
    }
    // Denormalize the fundamental matrix.
    current_F_21 = T_2_t * F_normalized * T_1;

    // Check the quality of the fundamental matrix and update the best solution.
    current_score = checkFundamental(current_F_21, std_dev_, current_inliers);
    if (current_score > score) {
      inliers = current_inliers;
      score   = current_score;
      F_21    = current_F_21;
    }
  }
}

Eigen::Matrix3f TwoViewReconstruction::computeH21(
  const std::vector<cv::Point2f>& points_1,
  const std::vector<cv::Point2f>& points_2
) const {
  if (points_1.empty() || points_2.empty()) {
    LOG(WARNING) << "Empty points, cannot compute homography matrix.";
    return Eigen::Matrix3f::Zero();
  }
  if (points_1.size() != points_2.size()) {
    LOG(WARNING) << "Different number of points, cannot compute homography matrix.";
    return Eigen::Matrix3f::Zero();
  }
  if (points_1.size() != 8) {
    LOG(WARNING) << "Number of points is not 8, cannot compute homography matrix.";
    return Eigen::Matrix3f::Zero();
  }

  // Construct the matrix A from the corresponding points.
  Eigen::MatrixXf A(2 * points_1.size(), 9);
  for (std::size_t i = 0; i < points_1.size(); i++) {
    // Point in the 1st view.
    const float u_1 = points_1[i].x;
    const float v_1 = points_1[i].y;
    // Corresponding point in the 2nd view.
    const float u_2 = points_2[i].x;
    const float v_2 = points_2[i].y;

    // 1st equation.
    A(2 * i, 0) = 0.f;
    A(2 * i, 1) = 0.f;
    A(2 * i, 2) = 0.f;
    A(2 * i, 3) = -u_1;
    A(2 * i, 4) = -v_1;
    A(2 * i, 5) = -1.f;
    A(2 * i, 6) = v_2 * u_1;
    A(2 * i, 7) = v_2 * v_1;
    A(2 * i, 8) = v_2;
    // 2nd equation.
    A(2 * i + 1, 0) = u_1;
    A(2 * i + 1, 1) = v_1;
    A(2 * i + 1, 2) = 1.f;
    A(2 * i + 1, 3) = 0.f;
    A(2 * i + 1, 4) = 0.f;
    A(2 * i + 1, 5) = 0.f;
    A(2 * i + 1, 6) = -u_2 * u_1;
    A(2 * i + 1, 7) = -u_2 * v_1;
    A(2 * i + 1, 8) = -u_2;
  }

  // Solve linear system using SVD and retrieve the vector corresponding to the
  // smallest singular value.
  const Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
  const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H(svd.matrixV().col(8).data());

  return H;
}

Eigen::Matrix3f TwoViewReconstruction::computeF21(
  const std::vector<cv::Point2f>& points_1,
  const std::vector<cv::Point2f>& points_2
) const {
  if (points_1.empty() || points_2.empty()) {
    LOG(WARNING) << "Empty points, cannot compute fundamental matrix.";
    return Eigen::Matrix3f::Zero();
  }
  if (points_1.size() != points_2.size()) {
    LOG(WARNING) << "Different number of points, cannot compute fundamental matrix.";
    return Eigen::Matrix3f::Zero();
  }
  if (points_1.size() != 8) {
    LOG(WARNING) << "Number of points is not 8, cannot compute fundamental matrix.";
    return Eigen::Matrix3f::Zero();
  }

  // Construct the matrix A from the corresponding points.
  Eigen::MatrixXf A(points_1.size(), 9);
  for (std::size_t i = 0; i < points_1.size(); i++) {
    // Point in the 1st view.
    const float u_1 = points_1[i].x;
    const float v_1 = points_1[i].y;
    // Corresponding point in the 2nd view.
    const float u_2 = points_2[i].x;
    const float v_2 = points_2[i].y;

    // Construct the matrix A.
    A(i, 0) = u_2 * u_1;
    A(i, 1) = u_2 * v_1;
    A(i, 2) = u_2;
    A(i, 3) = v_2 * u_1;
    A(i, 4) = v_2 * v_1;
    A(i, 5) = v_2;
    A(i, 6) = u_1;
    A(i, 7) = v_1;
    A(i, 8) = 1.f;
  }

  // Solve linear system using SVD: A = U * S * V^T and retrieve the vector
  // corresponding to the smallest singular value in the last column of V.
  const Eigen::JacobiSVD<Eigen::MatrixXf> svd_decomposition(
    A,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  // Reshape the vector to a 3x3 matrix.
  const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> F_approx(
    svd_decomposition.matrixV().col(8).data()
  );

  // Enforce rank 2 constraint on the fundamental matrix by performing SVD again
  // on F and setting the smallest singular value to 0 to ensure the resulting
  // matrix has rank 2, then recomputing F.
  const Eigen::JacobiSVD<Eigen::Matrix3f> svd_rank_enforcement(
    F_approx,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  Eigen::Vector3f w = svd_rank_enforcement.singularValues();
  w(2)              = 0.f;

  const Eigen::Matrix3f F21 = svd_rank_enforcement.matrixU()
                            * Eigen::DiagonalMatrix<float, 3>(w)
                            * svd_rank_enforcement.matrixV().transpose();
  return F21;
}

float TwoViewReconstruction::checkHomography(
  const Eigen::Matrix3f& H_21,
  const Eigen::Matrix3f& H_12,
  const float sigma,
  std::vector<bool>& inliers
) const {
  // Initialize.
  constexpr float thresh_df2 = 5.991f; // 95% confidence interval - 2 degrees of freedom
  const float sigma_squared_inv = 1.f / (sigma * sigma);

  const std::size_t num_matches = matches_.size();
  inliers.resize(num_matches);

  // Lambda function to compute the chi-square test.
  auto calculateChiSquare
    = [&](
        // Source point in source view.
        const float u_src,
        const float v_src,
        // Destination point in destination view.
        const float u_dst,
        const float v_dst,
        // Homography matrix from source to destination view.
        const Eigen::Matrix3f& H
      ) -> float {
        // Project the source point to the destination image.
        const float w_proj =  H(2, 0) * u_src + H(2, 1) * v_src + H(2, 2);
        const float u_proj = (H(0, 0) * u_src + H(0, 1) * v_src + H(0, 2)) / w_proj;
        const float v_proj = (H(1, 0) * u_src + H(1, 1) * v_src + H(1, 2)) / w_proj;
        // Calculate the distance between the projected and destination points.
        const float dist_squared = (u_dst - u_proj) * (u_dst - u_proj)
                                 + (v_dst - v_proj) * (v_dst - v_proj);
        // Calculate the chi-square and return.
        const float chi_squared = dist_squared * sigma_squared_inv;
        return chi_squared;
      };

  // Loop over all matches to update the inliers and compute the score.
  float score = 0.f;
  for (std::size_t i = 0; i < num_matches; i++) {
    bool is_inlier = true;

    // Extract the keypoints.
    const cv::KeyPoint& keypoint_1 = keypoints_1_[matches_[i].first ];
    const cv::KeyPoint& keypoint_2 = keypoints_2_[matches_[i].second];

    const float u1 = keypoint_1.pt.x;
    const float v1 = keypoint_1.pt.y;
    const float u2 = keypoint_2.pt.x;
    const float v2 = keypoint_2.pt.y;

    // Reprojection error in 1st view : x2in1 = H12 * x2
    const float chi_squared_1 = calculateChiSquare(u2, v2, u1, v1, H_12);
    if (chi_squared_1 > thresh_df2) {
      is_inlier = false;
    } else {
      score += thresh_df2 - chi_squared_1;
    }

    // Reprojection error in 2nd view : x1in2 = H21 * x1
    const float chi_squared_2 = calculateChiSquare(u1, v1, u2, v2, H_21);
    if (chi_squared_2 > thresh_df2) {
      is_inlier = false;
    } else {
      score += thresh_df2 - chi_squared_2;
    }

    // Update the inliers.
    inliers[i] = is_inlier;
  }

  return score;
}

float TwoViewReconstruction::checkFundamental(
  const Eigen::Matrix3f& F_21,
  const float sigma,
  std::vector<bool>& inliers
) const {
  // Initialize.
  const float sigma_squared_inv = 1.f / (sigma * sigma);
  constexpr float thresh_df1    = 3.841f; // 95% confidence interval - 1 degree of freedom
  constexpr float thresh_df2    = 5.991f; // 95% confidence interval - 2 degrees of freedom

  const std::size_t num_matches = matches_.size();
  inliers.resize(num_matches);

  // Extract parameters from the fundamental matrix.
  const float f_11 = F_21(0, 0);
  const float f_12 = F_21(0, 1);
  const float f_13 = F_21(0, 2);
  const float f_21 = F_21(1, 0);
  const float f_22 = F_21(1, 1);
  const float f_23 = F_21(1, 2);
  const float f_31 = F_21(2, 0);
  const float f_32 = F_21(2, 1);
  const float f_33 = F_21(2, 2);

  // Lambda function to compute the chi-square test.
  auto calculateChiSquare =
    [&](const float a, const float b, const float c, const float u, const float v) -> float {
    const float num          = a * u + b * v + c;
    const float dist_squared = num * num / (a * a + b * b);
    const float chi_squared  = dist_squared * sigma_squared_inv;
    return chi_squared;
  };

  // Loop over all matches to update the inliers and compute the score.
  float score = 0.f;
  for (std::size_t i = 0; i < num_matches; i++) {
    bool is_inlier = true;

    // Extract the keypoints.
    const cv::KeyPoint& keypoint_1 = keypoints_1_[matches_[i].first];
    const cv::KeyPoint& keypoint_2 = keypoints_2_[matches_[i].second];

    const float u_1 = keypoint_1.pt.x;
    const float v_1 = keypoint_1.pt.y;
    const float u_2 = keypoint_2.pt.x;
    const float v_2 = keypoint_2.pt.y;

    // Reprojection error in 2nd view.
    // l2 = F21 x1 = (a_2, b_2, c_2)
    const float a_2 = f_11 * u_1 + f_12 * v_1 + f_13;
    const float b_2 = f_21 * u_1 + f_22 * v_1 + f_23;
    const float c_2 = f_31 * u_1 + f_32 * v_1 + f_33;
    // Calculate the chi-square and check.
    const float chi_squared_2 = calculateChiSquare(a_2, b_2, c_2, u_2, v_2);
    if (chi_squared_2 > thresh_df1) {
      is_inlier = false;
    } else {
      score += thresh_df2 - chi_squared_2;
    }

    // Reprojection error in 1st view.
    // l1 = x2t F21 = (a_1, b_1, c_1)
    const float a_1 = f_11 * u_2 + f_21 * v_2 + f_31;
    const float b_1 = f_12 * u_2 + f_22 * v_2 + f_32;
    const float c_1 = f_13 * u_2 + f_23 * v_2 + f_33;
    // Calculate the chi-square and check.
    const float chi_squared_1 = calculateChiSquare(a_1, b_1, c_1, u_1, v_1);
    if (chi_squared_1 > thresh_df1) {
      is_inlier = false;
    } else {
      score += thresh_df2 - chi_squared_1;
    }

    // Update the inliers.
    inliers[i] = is_inlier;
  }

  return score;
}

bool TwoViewReconstruction::reconstructH(
  const std::vector<bool>& inliers,
  const Eigen::Matrix3f& H_21,
  const Eigen::Matrix3f& K,
  const float min_parallax,
  const int min_triangulated,
  Sophus::SE3f& T_21,
  std::vector<cv::Point3f>& points_3D,
  std::vector<bool>& triangulated_flags
) const {
  // Count the number of inliers.
  const std::size_t num_inliers = std::count(inliers.begin(), inliers.end(), true);

  // ──────────────────────────── //
  // Normalize the homography matrix

  // Normalize the homography matrix using the intrinsic matrix K.
  const Eigen::Matrix3f A = K.inverse() * H_21 * K;

  // Decompose the normalized homography using SVD.
  const Eigen::JacobiSVD<Eigen::Matrix3f> svd(
    A,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  const Eigen::Matrix3f U   = svd.matrixU();
  const Eigen::Matrix3f V   = svd.matrixV();
  const Eigen::Matrix3f V_t = V.transpose();
  const Eigen::Vector3f w   = svd.singularValues();
  const float s = U.determinant() * V_t.determinant();

  // Check the singular value ratios.
  const float d_1 = w(0);
  const float d_2 = w(1);
  const float d_3 = w(2);
  if (d_1 / d_2 < kMinSingularValueRatio || d_2 / d_3 < kMinSingularValueRatio) {
    return false;
  }

  // ──────────────────────────── //
  // We decompose the homography to recover 8 motion hypotheses using the method
  // of Faugeras et al. Motion and structure from motion in a piecewise planar
  // environment. International Journal of Pattern Recognition and Artificial
  // Intelligence, 1988. There are 2 cases considered:
  // 1. The determinant is positive: d' =  d_2
  // 2. The determinant is negative: d' = -d_2

  std::vector<Eigen::Matrix3f> rotations;
  std::vector<Eigen::Vector3f> translations;
  rotations.reserve(8);
  translations.reserve(8);
  float poss; // possible value

  // n'=[x1 0 x3] 4 possibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
  // - 4 possibilities for x1
  poss = std::sqrt((d_1 * d_1 - d_2 * d_2) / (d_1 * d_1 - d_3 * d_3));
  const std::vector<float> x_1 = {poss, poss, -poss, -poss};
  // - 4 possibilities for x3
  poss = std::sqrt((d_2 * d_2 - d_3 * d_3) / (d_1 * d_1 - d_3 * d_3));
  const std::vector<float> x_3 = {poss, -poss, poss, -poss};

  // Case 1: d' = d_2
  poss = std::sqrt((d_1 * d_1 - d_2 * d_2) * (d_2 * d_2 - d_3 * d_3)) / ((d_1 + d_3) * d_2);
  const std::vector<float> s_theta = {poss, -poss, -poss, poss};

  const float c_theta = (d_2 * d_2 + d_1 * d_3) / ((d_1 + d_3) * d_2);
  for (std::size_t i = 0; i < 4; i++) {
    // Compute the rotation matrix.
    const Eigen::Matrix3f R_p = (Eigen::Matrix3f() <<
         c_theta, 0.f, -s_theta[i],
             0.f, 1.f,         0.f,
      s_theta[i], 0.f,     c_theta
    ).finished();
    const Eigen::Matrix3f R = s * U * R_p * V_t;
    rotations.push_back(R);

    // Compute the translation vector.
    const Eigen::Vector3f t_p
      = (Eigen::Vector3f() << x_1[i], 0.f, -x_3[i]).finished() * (d_1 - d_3);
    const Eigen::Vector3f t = U * t_p;
    translations.push_back(t / t.norm());
  }

  // Case 2: d' = -d_2
  poss = std::sqrt((d_1 * d_1 - d_2 * d_2) * (d_2 * d_2 - d_3 * d_3)) / ((d_1 - d_3) * d_2);
  const std::vector<float> s_phi = {poss, -poss, -poss, poss};

  const float c_phi = (d_1 * d_3 - d_2 * d_2) / ((d_1 - d_3) * d_2);
  for (std::size_t i = 0; i < 4; i++) {
    // Compute the rotation matrix.
    const Eigen::Matrix3f R_p = (Eigen::Matrix3f() <<
         c_phi,  0.f,  s_phi[i],
           0.f, -1.f,       0.f,
      s_phi[i],  0.f,    -c_phi
    ).finished();
    const Eigen::Matrix3f R = s * U * R_p * V_t;
    rotations.push_back(R);

    // Compute the translation vector.
    const Eigen::Vector3f t_p
      = (Eigen::Vector3f() << x_1[i], 0.f, x_3[i]).finished() * (d_1 + d_3);
    const Eigen::Vector3f t = U * t_p;
    translations.push_back(t / t.norm());
  }

  // ──────────────────────────── //
  // Evaluate the 8 motion hypotheses using the triangulation.

  int second_best_num_good = 0;
  int best_num_good        = 0;
  int best_solution_idx    = -1;
  float best_parallax      = -1.f;
  std::vector<cv::Point3f> best_points_3D;
  std::vector<bool> best_triangulated_flags;

  // Instead of applying the visibility constraints proposed in the Faugeras'
  // paper (which could fail for points seen with low parallax), we reconstruct
  // all hypotheses and check in terms of triangulated points and parallax.
  for (std::size_t i = 0; i < 8; i++) {
    std::vector<cv::Point3f> points_3D_i;
    std::vector<bool> triangulated_flags_i;
    float parallax_i;
    const int num_good = checkRT(
      rotations[i],
      translations[i],
      keypoints_1_,
      keypoints_2_,
      matches_,
      inliers,
      K,
      4.f * var_,
      points_3D_i,
      triangulated_flags_i,
      parallax_i
    );

    if (num_good > best_num_good) {
      second_best_num_good    = best_num_good;
      best_num_good           = num_good;
      best_solution_idx       = static_cast<int>(i);
      best_parallax           = parallax_i;
      best_points_3D          = points_3D_i;
      best_triangulated_flags = triangulated_flags_i;
    } else if (num_good > second_best_num_good) {
      second_best_num_good = num_good;
    }
  }

  // ──────────────────────────── //
  // Choose the best solution.
  // The best solution should satisfy the following conditions:
  // - 75% of the good triangulated points must be greater than the second best
  // - the parallax must be greater than or equal to the minimum parallax
  // - the number of good triangulated points must be greater than the minimum
  //   number of triangulated points
  // - the number of good triangulated points must be greater than 90% of the
  //   total number of inliers

  if (
    static_cast<int>(0.75f * best_num_good) >  second_best_num_good &&
    best_parallax                           >= min_parallax         &&
    best_num_good                           >  min_triangulated     &&
    best_num_good                           >  0.9f * num_inliers
  ) {
    T_21 = Sophus::SE3f(rotations[best_solution_idx], translations[best_solution_idx]);
    points_3D = best_points_3D;
    triangulated_flags = best_triangulated_flags;

    return true;
  }

  return false;
}

bool TwoViewReconstruction::reconstructF(
  const std::vector<bool>& inliers,
  const Eigen::Matrix3f& F_21,
  const Eigen::Matrix3f& K,
  const float min_parallax,
  const int min_triangulated,
  Sophus::SE3f& T_21,
  std::vector<cv::Point3f>& points_3D,
  std::vector<bool>& triangulated_flags
) const {
  // Count the number of inliers.
  const std::size_t num_inliers = std::count(inliers.begin(), inliers.end(), true);

  // ──────────────────────────── //
  // Compute Essential Matrix and recover the 4 motion hypotheses.

  // Compute Essential Matrix from Fundamental Matrix
  const Eigen::Matrix3f E_21 = K.transpose() * F_21 * K;

  // Recover the 4 motion hypotheses:
  // - (R_1, t_1)
  // - (R_2, t_1)
  // - (R_1, t_2)
  // - (R_2, t_2)

  Eigen::Matrix3f R_1, R_2;
  Eigen::Vector3f t;
  decomposeE(E_21, R_1, R_2, t);
  const Eigen::Vector3f t_1 =  t;
  const Eigen::Vector3f t_2 = -t;

  // Check the 4 motion hypotheses using triangulation and parallax.
  std::vector<cv::Point3f> points_3D_1, points_3D_2,
                           points_3D_3, points_3D_4;
  std::vector<bool> triangulated_flags_1, triangulated_flags_2,
                    triangulated_flags_3, triangulated_flags_4;
  float parallax_1, parallax_2, parallax_3, parallax_4;

  const int num_good_1 = checkRT(
    R_1,
    t_1,
    keypoints_1_,
    keypoints_2_,
    matches_,
    inliers,
    K,
    4.f * var_,
    points_3D_1,
    triangulated_flags_1,
    parallax_1
  );
  const int num_good_2 = checkRT(
    R_2,
    t_1,
    keypoints_1_,
    keypoints_2_,
    matches_,
    inliers,
    K,
    4.f * var_,
    points_3D_2,
    triangulated_flags_2,
    parallax_2
  );
  const int num_good_3 = checkRT(
    R_1,
    t_2,
    keypoints_1_,
    keypoints_2_,
    matches_,
    inliers,
    K,
    4.f * var_,
    points_3D_3,
    triangulated_flags_3,
    parallax_3
  );
  const int num_good_4 = checkRT(
    R_2,
    t_2,
    keypoints_1_,
    keypoints_2_,
    matches_,
    inliers,
    K,
    4.f * var_,
    points_3D_4,
    triangulated_flags_4,
    parallax_4
  );

  // ──────────────────────────── //
  // Select the best motion hypothesis if it has:
  // - highest number of triangulated points
  // - good parallax
  // - enough triangulated points (more than 90% of the total number of inliers)

  // Check if the best reconstruction has enough triangulated points.
  const int max_good = std::max({num_good_1, num_good_2, num_good_3, num_good_4});
  const int min_good = std::max(static_cast<int>(0.9f * num_inliers), min_triangulated);
  if (max_good < min_good) {
    LOG(WARNING) << "Insufficient triangulated points, reject reconstruction of F";
    return false;
  }

  // Check if there is a clear winner.
  const std::vector<int> num_goods = {num_good_1, num_good_2, num_good_3, num_good_4};
  const int num_winners = std::count_if(
    num_goods.begin(),
    num_goods.end(),
    [&](const int num_good) {
      return num_good > 0.7f * max_good;
    }
  );
  if (num_winners > 1) {
    LOG(WARNING) << "Not a clear winner due to multiple good hypotheses, reject reconstruction of F";
    return false;
  }

  // If best reconstruction has enough parallax, select it.
  if (max_good == num_good_1 && parallax_1 > min_parallax) {
    T_21               = Sophus::SE3f(R_1, t_1);
    points_3D          = points_3D_1;
    triangulated_flags = triangulated_flags_1;
    return true;
  } else if (max_good == num_good_2 && parallax_2 > min_parallax) {
    T_21               = Sophus::SE3f(R_2, t_1);
    points_3D          = points_3D_2;
    triangulated_flags = triangulated_flags_2;
    return true;
  } else if (max_good == num_good_3 && parallax_3 > min_parallax) {
    T_21               = Sophus::SE3f(R_1, t_2);
    points_3D          = points_3D_3;
    triangulated_flags = triangulated_flags_3;
    return true;
  } else if (max_good == num_good_4 && parallax_4 > min_parallax) {
    T_21               = Sophus::SE3f(R_2, t_2);
    points_3D          = points_3D_4;
    triangulated_flags = triangulated_flags_4;
    return true;
  }

  return false;
}

void TwoViewReconstruction::normalize(
  const std::vector<cv::KeyPoint>& keypoints,
  std::vector<cv::Point2f>& normalized_points,
  Eigen::Matrix3f& T
) const {
  if (keypoints.empty()) {
    LOG(WARNING) << "Empty keypoints, cannot normalize points";
    normalized_points.clear();
    T.setIdentity();
    return;
  }

  const std::size_t num_keypoints = keypoints.size();
  normalized_points.resize(num_keypoints);

  // Compute the mean of the points.
  float mean_x = 0.f;
  float mean_y = 0.f;
  for (std::size_t i = 0; i < num_keypoints; i++) {
    mean_x += keypoints[i].pt.x;
    mean_y += keypoints[i].pt.y;
  }
  mean_x /= static_cast<float>(num_keypoints);
  mean_y /= static_cast<float>(num_keypoints);

  // Center the points and compute the average absolute deviation.
  float mean_dev_x = 0.f;
  float mean_dev_y = 0.f;
  for (std::size_t i = 0; i < num_keypoints; i++) {
    normalized_points[i].x = keypoints[i].pt.x - mean_x;
    normalized_points[i].y = keypoints[i].pt.y - mean_y;

    mean_dev_x += std::abs(normalized_points[i].x);
    mean_dev_y += std::abs(normalized_points[i].y);
  }
  mean_dev_x /= static_cast<float>(num_keypoints);
  mean_dev_y /= static_cast<float>(num_keypoints);

  const float s_x = 1.f / mean_dev_x;
  const float s_y = 1.f / mean_dev_y;

  // Scale the points so that the average absolute deviation is 1.
  for (std::size_t i = 0; i < num_keypoints; i++) {
    normalized_points[i].x = normalized_points[i].x * s_x;
    normalized_points[i].y = normalized_points[i].y * s_y;
  }

  // Compute the transformation matrix.
  T.setZero();
  T(0, 0) = s_x;
  T(1, 1) = s_y;
  T(0, 2) = -mean_x * s_x;
  T(1, 2) = -mean_y * s_y;
  T(2, 2) = 1.f;
}

int TwoViewReconstruction::checkRT(
  const Eigen::Matrix3f& R,
  const Eigen::Vector3f& t,
  const std::vector<cv::KeyPoint>& keypoints_1,
  const std::vector<cv::KeyPoint>& keypoints_2,
  const Matches& matches_12,
  const std::vector<bool>& inliers,
  const Eigen::Matrix3f& K,
  const float thresh_squared,
  std::vector<cv::Point3f>& points_3D,
  std::vector<bool>& triangulated_flags,
  float& parallax
) const {
  // Extract intrinsic parameters from camera matrix K.
  const float f_x = K(0, 0);
  const float f_y = K(1, 1);
  const float c_x = K(0, 2);
  const float c_y = K(1, 2);

  // Initialize variables.
  triangulated_flags = std::vector<bool>(keypoints_1.size(), false);
  points_3D.resize(keypoints_1.size());

  std::vector<float> parallax_cosines;
  parallax_cosines.reserve(keypoints_1.size());

  // Setup projection matrices.
  // - Camera 1 projection matrix K[I|0]
  Eigen::Matrix<float, 3, 4> P_1 = Eigen::Matrix<float, 3, 4>::Zero();
  P_1.block<3, 3>(0, 0) = K;
  // - Camera 1 optical center
  const Eigen::Vector3f O_1 = Eigen::Vector3f::Zero();
  // - Camera 2 projection matrix K[R|t]
  Eigen::Matrix<float, 3, 4> P_2 = Eigen::Matrix<float, 3, 4>::Zero();
  P_2.block<3, 3>(0, 0) = R;
  P_2.block<3, 1>(0, 3) = t;
  P_2                   = K * P_2;
  // - Camera 2 optical center
  const Eigen::Vector3f O_2 = -R.transpose() * t;

  // Loop over all matches to check the reprojection error.
  int num_good = 0;
  for (std::size_t i = 0; i < matches_12.size(); i++) {
    // Skip if the match is not an inlier.
    if (!inliers[i]) {
      continue;
    }

    const auto& match = matches_12[i];

    // Triangulate the 3D point.
    const cv::KeyPoint& keypoint_1 = keypoints_1[match.first ];
    const cv::KeyPoint& keypoint_2 = keypoints_2[match.second];
    Eigen::Vector3f point_3D_1; // 3D point in 1st view frame.
    GeometricTools::triangulate(
      Eigen::Vector3f(keypoint_1.pt.x, keypoint_1.pt.y, 1.f),
      Eigen::Vector3f(keypoint_2.pt.x, keypoint_2.pt.y, 1.f),
      P_1,
      P_2,
      point_3D_1
    );

    // Check validity of the 3D point coordinates.
    if (
      !std::isfinite(point_3D_1.x()) ||
      !std::isfinite(point_3D_1.y()) ||
      !std::isfinite(point_3D_1.z())
    ) {
      // Skip if the point is
      triangulated_flags[match.first] = false;
      continue;
    }

    // Check parallax and depth validities.

    // Vectors from camera centers to the 3D point.
    const Eigen::Vector3f n_1 = point_3D_1 - O_1;
    const Eigen::Vector3f n_2 = point_3D_1 - O_2;
    // Cosine of the parallax angle.
    const float parallax_cosine = n_1.dot(n_2) / (n_1.norm() * n_2.norm());

    // Skip if the point is not in front of camera 1 and parallax is too small
    // as infinite points can easily go to negative depth.
    if (point_3D_1.z() <= 0.f && parallax_cosine < kMinParallaxCosine) {
      continue;
    }
    // Skip if the point is not in front of camera 2 and parallax is too small
    // as infinite points can easily go to negative depth.
    const Eigen::Vector3f point_3D_2 = R * point_3D_1 + t;
    if (point_3D_2.z() <= 0.f && parallax_cosine < kMinParallaxCosine) {
      continue;
    }

    // Lambda function to check reprojection error.
    auto checkReprojectionError
    = [&]
      (
        const Eigen::Vector3f& point_3D,
        const cv::KeyPoint& keypoint,
        const float thresh_squared
      ) -> bool {
        const float depth_inv = 1.f / point_3D.z();
        const float x_proj = f_x * point_3D.x() * depth_inv + c_x;
        const float y_proj = f_y * point_3D.y() * depth_inv + c_y;
        const float error_squared
          = (x_proj - keypoint.pt.x) * (x_proj - keypoint.pt.x)
          + (y_proj - keypoint.pt.y) * (y_proj - keypoint.pt.y);
        return error_squared <= thresh_squared;
      };
    // Check reprojection error in 1st view.
    if (!checkReprojectionError(point_3D_1, keypoint_1, thresh_squared)) {
      continue;
    }
    // Check reprojection error in 2nd view.
    if (!checkReprojectionError(point_3D_2, keypoint_2, thresh_squared)) {
      continue;
    }

    // Store the parallax and corresponding 3D point.
    parallax_cosines.push_back(parallax_cosine);
    points_3D[match.first] = Converter::toCvPoint3f(point_3D_1);
    num_good++;

    // Mark the point as triangulated if it has sufficient parallax.
    if (parallax_cosine < kMinParallaxCosine) {
      triangulated_flags[match.first] = true;
    }
  }

  // Compute the parallax angle in degrees.
  if (num_good > 0) {
    std::sort(parallax_cosines.begin(), parallax_cosines.end());

    const std::size_t idx = std::min(
      static_cast<std::size_t>(kMinNumTriangulated),
      parallax_cosines.size() - 1
    );
    parallax = std::acos(parallax_cosines[idx]) * 180.f / M_PI;
  } else {
    parallax = 0.f;
  }

  return num_good;
}

void TwoViewReconstruction::decomposeE(
  const Eigen::Matrix3f& E,
  Eigen::Matrix3f& R_1,
  Eigen::Matrix3f& R_2,
  Eigen::Vector3f& t
) const {
  // Construct SVD of E and retrieve orthogonal matrices U and V_t.
  const Eigen::JacobiSVD<Eigen::Matrix3f> svd(
    E,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  const Eigen::Matrix3f U   = svd.matrixU();
  const Eigen::Matrix3f V_t = svd.matrixV().transpose();

  // Extract the translation vector.
  t = U.col(2);
  t = t / t.norm();

  // Extract the rotation matrices.
  Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
  W(0, 1) = -1.f;
  W(1, 0) =  1.f;
  W(2, 2) =  1.f;

  R_1 = U * W * V_t;
  if (R_1.determinant() < 0.f) {
    R_1 = -R_1;
  }

  R_2 = U * W.transpose() * V_t;
  if (R_2.determinant() < 0.f) {
    R_2 = -R_2;
  }
}

} // namespace ORB_SLAM3
