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

#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

// Standard
#include <vector>
// 3rdparty
#include <Eigen/Core>
#include <boost/serialization/array.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core.hpp>
#include <orbslam3/external/Sophus/sophus/se3.hpp>

namespace ORB_SLAM3 {

template <class Archive>
void serializeSophusSE3(Archive& ar, Sophus::SE3f& T, const unsigned int) {
  // Data to serialize/deserialize.
  Eigen::Vector4f quaternion;
  Eigen::Vector3f translation;
  // If serializing, calculate the quaternion and translation from the SE3.
  if (Archive::is_saving::value) {
    // TODO: Why w, x, y, z?
    const Eigen::Quaternionf q = T.unit_quaternion();
    quaternion << q.w(), q.x(), q.y(), q.z();
    translation = T.translation();
  }
  // Serialize/deserialize the quaternion and translation.
  ar& boost::serialization::make_array( quaternion.data(),  quaternion.size());
  ar& boost::serialization::make_array(translation.data(), translation.size());
  // If deserializing, calculate the SE3 from the quaternion and translation.
  if (Archive::is_loading::value) {
    const Eigen::Quaternionf q(
      quaternion[0], // w
      quaternion[1], // x
      quaternion[2], // y
      quaternion[3]  // z
    );
    T = Sophus::SE3f(q, translation);
  }
}

template <class Archive>
void serializeMatrix(Archive& ar, cv::Mat& mat, const unsigned int) {
  // Serialize/deserialize the matrix metadata.
  int cols, rows, type;
  bool continuous;
  if (Archive::is_saving::value) {
    cols       = mat.cols;
    rows       = mat.rows;
    type       = mat.type();
    continuous = mat.isContinuous();
  }
  ar& cols& rows& type& continuous;
  if (Archive::is_loading::value) {
    mat.create(rows, cols, type);
  }

  // Serialize/deserialize the matrix data based on the continuity.
  if (continuous) {
    const std::size_t data_size = rows * cols * mat.elemSize();
    ar& boost::serialization::make_array(mat.ptr(), data_size);
  } else {
    const std::size_t row_size = cols * mat.elemSize();
    for (int i = 0; i < rows; i++) {
      ar& boost::serialization::make_array(mat.ptr(i), row_size);
    }
  }
}

// An overload function to handle const cv::Mat. Since the matrix is const, it
// cannot be modified during deserialization. Therefore, a temporary matrix is
// used to store the deserialized data and then assigned to the const matrix.
// This method can be used thanks to the mechanism of cv::Mat which shares the
// underlying data when the assignment operator is used.
template <class Archive>
void serializeMatrix(Archive& ar, const cv::Mat& mat, const unsigned int version) {
  // Use const_cast to remove the const qualifier.
  cv::Mat& mat_ref = const_cast<cv::Mat&>(mat);
  // Call the non-const version of the function.
  serializeMatrix(ar, mat_ref, version);
}

template <class Archive>
void serializeKeyPoints(
  Archive& ar,
  const std::vector<cv::KeyPoint>& keypoints,
  const unsigned int
) {
  std::vector<cv::KeyPoint>& non_const = const_cast<std::vector<cv::KeyPoint>&>(keypoints);

  // Serialize/deserialize the number of keypoints.
  std::size_t num_keypoints;
  if (Archive::is_saving::value) {
    num_keypoints = keypoints.size();
  }
  ar& num_keypoints;
  if (Archive::is_loading::value) {
    non_const.clear();
    non_const.reserve(num_keypoints);
  }

  // Serialize/deserialize each keypoint.
  for (std::size_t i = 0; i < num_keypoints; ++i) {
    cv::KeyPoint keypoint;

    if (Archive::is_saving::value) {
      keypoint = keypoints[i];
    }
    ar& keypoint.angle;
    ar& keypoint.response;
    ar& keypoint.size;
    ar& keypoint.pt.x;
    ar& keypoint.pt.y;
    ar& keypoint.class_id;
    ar& keypoint.octave;
    if (Archive::is_loading::value) {
      non_const.push_back(keypoint);
    }
  }
}

} // namespace ORB_SLAM3

#endif // SERIALIZATION_UTILS_H
