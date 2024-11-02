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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

// Standard
#include <cmath>
#include <set>
#include <vector>
// 3rdparty
#include <Eigen/Core>
#include <orbslam3/external/g2o/g2o/types/sim3.h>
// Local
#include <orbslam3/LoopClosing.h>

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class Map;
class MapPoint;

class Optimizer {
public:
  void static BundleAdjustment(
    const std::vector<KeyFrame*>& key_frames,
    const std::vector<MapPoint*>& map_points,
    int num_iterations           = 5,
    bool* stop_flag              = NULL,
    const unsigned long loop_id  = 0,
    const bool use_robust_kernel = true
  );

  void static GlobalBundleAdjustemnt(
    Map* map,
    int num_iterations           = 5,
    bool* stop_flag              = NULL,
    const unsigned long loop_id  = 0,
    const bool use_robust_kernel = true
  );

  void static FullInertialBA(
    Map* map,
    int num_iterations,
    const bool fix_local        = false,
    const unsigned long loop_id = 0,
    bool* stop_flag             = NULL,
    bool initialized            = false,
    float prior_gyro            = 1e2,
    float prior_acc             = 1e6,
    Eigen::VectorXd* singular_values = NULL, // TODO: currently not used, remove it
    bool* hessian = NULL // TODO: currently not used, remove it
  );

  void static LocalBundleAdjustment(
    KeyFrame* key_frames,
    bool* stop_flag,
    Map* map,
    int& num_fixed_key_frames,
    int& num_opt_key_frames,
    int& num_map_points, // TODO: currently not used, remove it
    int& num_edges
  );

  int static PoseOptimization(Frame* frame);

  int static PoseInertialOptimizationLastKeyFrame(
    Frame* frame,
    bool bRecInit = false // TODO: rename to inverted state: use_recovery
  );

  int static PoseInertialOptimizationLastFrame(
    Frame* frame,
    bool bRecInit = false
  );

  // If fix_scale is true, optimize 6DoF (stereo, rgbd), otherwise 7DoF (mono).
  void static OptimizeEssentialGraph(
    Map* map,
    KeyFrame* loop_key_frame,
    KeyFrame* curr_key_frame,
    const LoopClosing::KeyFrameAndPose& uncorrected_poses,
    const LoopClosing::KeyFrameAndPose& corrected_poses,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& loop_connections,
    const bool& fix_scale
  );

  void static OptimizeEssentialGraph(
    KeyFrame* curr_key_frame,
    std::vector<KeyFrame*>& fixed_key_frames,
    std::vector<KeyFrame*>& fixed_corrected_key_frames,
    std::vector<KeyFrame*>& non_fixed_key_frames,
    std::vector<MapPoint*>& uncorrected_map_points
  );

  // For inertial loopclosing.
  void static OptimizeEssentialGraph4DoF(
    Map* map,
    KeyFrame* loop_key_frame,
    KeyFrame* curr_key_frame,
    const LoopClosing::KeyFrameAndPose& uncorrected_poses,
    const LoopClosing::KeyFrameAndPose& corrected_poses,
    const std::map<KeyFrame*, std::set<KeyFrame*>>& loop_connections
  );

  // If fix_scale is true, optimize SE3 (stereo, rgbd), otherwise Sim3 (mono).
  static int OptimizeSim3(
    KeyFrame* key_frame_1,
    KeyFrame* key_frame_2,
    std::vector<MapPoint*>& corr_map_points_2, // Corresponding map points in key_frame_2
    g2o::Sim3& guess,
    const float thresh_squared,
    const bool fix_scale,
    Eigen::Matrix<double, 7, 7>& H_accum, // accumulated Hessian
    const bool remove_all_points = false // TODO(VuHoi): only false is currently used
  );

  // For inertial systems.
  void static LocalInertialBA(
    KeyFrame* key_frame,
    bool* stop_flag,
    Map* map,
    int& num_fixed_key_frames,
    int& num_opt_key_frames,
    int& num_map_points,
    int& num_edges,
    bool large_optimization = false,
    bool bRecInit = false // TODO: rename to inverted state: use_recovery
  );

  void static MergeInertialBA(
    KeyFrame* curr_key_frame,
    KeyFrame* merge_key_frame,
    bool* stop_flag,
    Map* map,
    LoopClosing::KeyFrameAndPose& corr_poses
  );

  // Local BA in welding area when two maps are merged.
  void static LocalBundleAdjustment(
    KeyFrame* main_key_frame,
    std::vector<KeyFrame*> adjust_key_frames,
    std::vector<KeyFrame*> fixed_key_frames,
    bool* stop_flag
  );

  // Marginalize block element (start:end, start:end). Perform Schur complement.
  // Marginalized elements are filled with zeros.
  static Eigen::MatrixXd Marginalize(
    const Eigen::MatrixXd& H,
    const int& start,
    const int& end
  );

  // Inertial pose-graph.
  void static InertialOptimization(
    Map* map,
    Eigen::Matrix3d& R_wg,
    double& scale,
    Eigen::Vector3d& bias_gyro,
    Eigen::Vector3d& bias_acc,
    bool is_mono,
    Eigen::MatrixXd& covInertial, // TODO(VuHoi): currently not used, remove it
    bool fixed_velocity = false,
    bool bGauss         = false, // TODO(VuHoi): currently not used, remove it
    float prior_gyro    = 1e2,
    float prior_acc     = 1e6
  );

  void static InertialOptimization(
    Map* map,
    Eigen::Vector3d& bias_gyro,
    Eigen::Vector3d& bias_acc,
    float prior_gyro = 1e2,
    float prior_acc = 1e6
  );

  void static InertialOptimization(
    Map* map,
    Eigen::Matrix3d& R_wg,
    double& scale
  );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} // namespace ORB_SLAM3

#endif // OPTIMIZER_H
