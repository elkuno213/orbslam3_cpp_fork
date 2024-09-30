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

// Local
#include "orbslam3/Converter.h"

namespace ORB_SLAM3::Converter {

// ──────────────────────────── //
// Conversion to OpenCV

cv::Point2f toCvPoint2f(const Eigen::Vector2f& vector) {
  return {vector.x(), vector.y()};
}

cv::Point3f toCvPoint3f(const Eigen::Vector3f& vector) {
  return {vector.x(), vector.y(), vector.z()};
}

cv::Mat toCvMat(const Eigen::Matrix3f& matrix) {
  cv::Mat mat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat.at<float>(i, j) = matrix(i, j);
    }
  }
  return mat.clone();
}

cv::Mat toCvMat(const Eigen::Matrix4f& matrix) {
  cv::Mat mat(4, 4, CV_32F);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mat.at<float>(i, j) = matrix(i, j);
    }
  }
  return mat.clone();
}

// ──────────────────────────── //
// Conversion to Eigen

Eigen::Vector2f toEigenVector2f(const cv::Point2f& pt) {
  return {pt.x, pt.y};
}

Eigen::Vector3f toEigenVector3f(const cv::Point3f& pt) {
  return {pt.x, pt.y, pt.z};
}

Eigen::Vector3f toEigenVector3f(const cv::Mat& mat) {
  if (mat.rows != 3 || mat.cols != 1) {
    throw std::runtime_error("toEigenVector3f: input matrix must be 3x1");
  }
  Eigen::Vector3f matrix;
  matrix << mat.at<float>(0), mat.at<float>(1), mat.at<float>(2);
  return matrix;
}

Eigen::Matrix3f toEigenMatrix3f(const cv::Mat& mat) {
  if (mat.rows != 3 || mat.cols != 3) {
    throw std::runtime_error("toEigenMatrix3f: input matrix must be 3x3");
  }
  Eigen::Matrix3f matrix;
  matrix << mat.at<float>(0, 0), mat.at<float>(0, 1), mat.at<float>(0, 2),
            mat.at<float>(1, 0), mat.at<float>(1, 1), mat.at<float>(1, 2),
            mat.at<float>(2, 0), mat.at<float>(2, 1), mat.at<float>(2, 2);
  return matrix;
}

// ──────────────────────────── //
// Conversion to Sophus

Sophus::SE3f toSophus(const cv::Mat& mat) {
  if (mat.rows != 4 || mat.cols != 4) {
    throw std::runtime_error("toSophus: input matrix must be 4x4");
  }
  const Eigen::Matrix3f R = toEigenMatrix3f(mat.rowRange(0, 3).colRange(0, 3));
  const Eigen::Quaternionf quaternion(R);
  const Eigen::Vector3f t = toEigenVector3f(mat.rowRange(0, 3).col(3));
  return Sophus::SE3f(quaternion, t);
}

Sophus::Sim3f toSophus(const g2o::Sim3& sim3) {
  return Sophus::Sim3f(
    Sophus::RxSO3d(sim3.scale(), sim3.rotation().matrix()).cast<float>(),
    sim3.translation().cast<float>()
  );
}

// ──────────────────────────── //
// Conversion to std

std::vector<float> toQuaternion(const cv::Mat& mat) {
  if (mat.rows != 3 || mat.cols != 3) {
    throw std::runtime_error("toQuaternion: input matrix must be 3x3");
  }
  const Eigen::Matrix3f matrix = toEigenMatrix3f(mat);
  const Eigen::Quaternionf quaternion(matrix);
  return {quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
}

std::vector<cv::Mat> toDescriptorVector(const cv::Mat& mat) {
  std::vector<cv::Mat> descriptors;
  descriptors.reserve(mat.rows);
  for (int r = 0; r < mat.rows; r++) {
    descriptors.push_back(mat.row(r));
  }
  return descriptors;
}

} // namespace ORB_SLAM3::Converter
