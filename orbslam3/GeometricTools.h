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

#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

// 3rdparty
#include <Eigen/Core>

namespace ORB_SLAM3 {

class GeometricTools {
public:
  // Triangulate point from two camera poses. Returns false if invalid scale,
  // true otherwise.
  static bool triangulate(
    const Eigen::Vector3f& point_c1, // Homogeneous coordinates of the point in camera 1
    const Eigen::Vector3f& point_c2, // Homogeneous coordinates of the point in camera 2
    const Eigen::Matrix<float, 3, 4>& T_c1w, // Transformation from world to camera 1
    const Eigen::Matrix<float, 3, 4>& T_c2w, // Transformation from world to camera 2
    Eigen::Vector3f& triangulated // Triangulated point
  );
};

} // namespace ORB_SLAM3

#endif // GEOMETRIC_TOOLS_H
