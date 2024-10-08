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

// 3rdparty
#include <Eigen/SVD>
// Local
#include "orbslam3/GeometricTools.h"

namespace ORB_SLAM3 {

bool GeometricTools::triangulate(
  const Eigen::Vector3f& point_c1,
  const Eigen::Vector3f& point_c2,
  const Eigen::Matrix<float, 3, 4>& T_c1w,
  const Eigen::Matrix<float, 3, 4>& T_c2w,
  Eigen::Vector3f& triangulated
) {
  // TODO: add edge case that 2 transformations are the same.

  // Construct the matrix A for the linear system Ax = 0.
  Eigen::Matrix4f A;
  A.row(0) = point_c1.x() * T_c1w.row(2) - T_c1w.row(0);
  A.row(1) = point_c1.y() * T_c1w.row(2) - T_c1w.row(1);
  A.row(2) = point_c2.x() * T_c2w.row(2) - T_c2w.row(0);
  A.row(3) = point_c2.y() * T_c2w.row(2) - T_c2w.row(1);

  // Solve the linear system using SVD.
  const Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4f x = svd.matrixV().col(3); // Homogeneous coordinates

  // Check if the point has valid scale.
  if (x(3) == 0.f) {
    return false;
  }

  // Convert the homogeneous coordinates to Euclidean coordinates.
  triangulated = x.head(3) / x(3);

  return true;
}

} // namespace ORB_SLAM3
