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

#ifndef CONVERTER_H
#define CONVERTER_H

// 3rdparty
#include <orbslam3/external/g2o/g2o/types/se3quat.h>
#include <orbslam3/external/g2o/g2o/types/sim3.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <orbslam3/external/Sophus/sophus/se3.hpp>
#include <orbslam3/external/Sophus/sophus/sim3.hpp>

namespace ORB_SLAM3::Converter {

// ──────────────────────────── //
// Conversion to OpenCV

// TODO templetize these functions
cv::Point2f toCvPoint2f(const Eigen::Vector2f& vector);
cv::Point3f toCvPoint3f(const Eigen::Vector3f& vector);
cv::Mat toCvMat(const Eigen::Matrix3f& matrix);
cv::Mat toCvMat(const Eigen::Matrix4f& matrix);

// ──────────────────────────── //
// Conversion to Eigen

Eigen::Vector2f toEigenVector2f(const cv::Point2f& pt);
Eigen::Vector3f toEigenVector3f(const cv::Point3f& pt);
Eigen::Vector3f toEigenVector3f(const cv::Mat& mat);
Eigen::Matrix3f toEigenMatrix3f(const cv::Mat& mat);

// ──────────────────────────── //
// Conversion to Sophus

// TODO: Sophus migration, to be deleted in the future
Sophus::SE3f  toSophus(const cv::Mat&   mat );
Sophus::Sim3f toSophus(const g2o::Sim3& sim3);

// ──────────────────────────── //
// Conversion to std

std::vector<float> toQuaternion(const cv::Mat& mat);
std::vector<cv::Mat> toDescriptorVector(const cv::Mat& mat);

} // namespace ORB_SLAM3::Converter

#endif // CONVERTER_H
