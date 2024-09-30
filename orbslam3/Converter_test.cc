// 3rdparty
#include <gtest/gtest.h>
// Local
#include "orbslam3/Converter.h"

// Some default values to test.
// Eigen.
const Eigen::Quaternionf kQuaternionEigen(
  // Order: ZYX
    Eigen::AngleAxisf( M_PI / 3., Eigen::Vector3f::UnitZ()) // 60° around z-axis
  * Eigen::AngleAxisf(-M_PI / 6., Eigen::Vector3f::UnitY()) // -30° around y-axis
  * Eigen::AngleAxisf( M_PI / 2., Eigen::Vector3f::UnitX()) // 90° around x-axis
);
const Eigen::Matrix3f kRotationEigen = kQuaternionEigen.toRotationMatrix();
const Eigen::Vector3f kTranslationEigen(1.f, 2.f, 3.f);
const Eigen::Matrix4f kTransformationEigen = (
  Eigen::Matrix4f() << 0.4330127f,      -0.25f,  0.8660254f, 1.f,
                            0.75f, -0.4330127f,       -0.5f, 2.f,
                             0.5f,  0.8660254f,         0.f, 3.f,
                              0.f,         0.f,         0.f, 1.f
).finished();
// OpenCV.
const cv::Mat kRotationCv = (
  cv::Mat_<float>(3, 3) << 0.4330127f,      -0.25f, 0.8660254f,
                                0.75f, -0.4330127f,      -0.5f,
                                 0.5f,  0.8660254f,        0.f
);
const cv::Mat kTranslationCv = (cv::Mat_<float>(3, 1) << 1.f, 2.f, 3.f);
const cv::Mat kTransformationCv = (
  cv::Mat_<float>(4, 4) << 0.4330127f,      -0.25f, 0.8660254f, 1.f,
                                0.75f, -0.4330127f,      -0.5f, 2.f,
                                 0.5f,  0.8660254f,        0.f, 3.f,
                                  0.f,         0.f,        0.f, 1.f
);
// Std.
const std::vector<float> kEulerStd = {M_PI / 2., -M_PI / 6., M_PI / 3.}; // x, y, z

inline bool isEqual(const cv::Mat& one, const cv::Mat& other) {
  if (one.rows != other.rows || one.cols != other.cols) {
    return false;
  }
  for (int i = 0; i < one.rows; i++) {
    for (int j = 0; j < one.cols; j++) {
      if (std::abs(one.at<float>(i, j) - other.at<float>(i, j)) > 1e-6) {
        return false;
      }
    }
  }
  return true;
}

inline bool isEqual(
  const std::vector<float>& one,
  const std::vector<float>& other
) {
  if (one.size() != other.size()) {
    return false;
  }
  for (std::size_t i = 0; i < one.size(); i++) {
    if (std::abs(one[i] - other[i]) > 1e-6) {
      return false;
    }
  }
  return true;
}

inline bool isEqual(const Eigen::VectorXd& one, const Eigen::VectorXd& other) {
  if (one.size() != other.size()) {
    return false;
  }
  for (int i = 0; i < one.size(); i++) {
    if (std::abs(one(i) - other(i)) > 1e-6) {
      return false;
    }
  }
  return true;
}

TEST(Converter, toOpenCV) {
  // Eigen::Vector2f to cv::Point2f
  {
    const Eigen::Vector2f vector(1.f, 2.f);
    const cv::Point2f point = ORB_SLAM3::Converter::toCvPoint2f(vector);
    const cv::Point2f expected(1.f, 2.f);
    EXPECT_EQ(point, expected);
  }
  // Eigen::Vector3f to cv::Point3f
  {
    const Eigen::Vector3f vector(1.f, 2.f, 3.f);
    const cv::Point3f point = ORB_SLAM3::Converter::toCvPoint3f(vector);
    const cv::Point3f expected(1.f, 2.f, 3.f);
    EXPECT_EQ(point, expected);
  }
  // Eigen::Matrix3f to cv::Mat
  {
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(kRotationEigen);
    EXPECT_TRUE(isEqual(mat, kRotationCv));
  }
  // Eigen::Matrix4f to cv::Mat
  {
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(kTransformationEigen);
    EXPECT_TRUE(isEqual(mat, kTransformationCv));
  }
}

TEST(Converter, toEigen) {
  // cv::Point2f to Eigen::Vector2f
  {
    const cv::Point2f point(1.f, 2.f);
    const Eigen::Vector2f vector = ORB_SLAM3::Converter::toEigenVector2f(point);
    const Eigen::Vector2f expected(1.f, 2.f);
    EXPECT_TRUE(vector.isApprox(expected));
  }
  // cv::Point3f to Eigen::Vector3f
  {
    const cv::Point3f point(1.f, 2.f, 3.f);
    const Eigen::Vector3f vector = ORB_SLAM3::Converter::toEigenVector3f(point);
    const Eigen::Vector3f expected(1.f, 2.f, 3.f);
    EXPECT_TRUE(vector.isApprox(expected));
  }
  // cv::Mat to Eigen::Vector3f
  {
    const Eigen::Vector3f matrix = ORB_SLAM3::Converter::toEigenVector3f(kTranslationCv);
    const Eigen::Vector3f expected = kTranslationEigen;
    EXPECT_TRUE(matrix.isApprox(expected));
  }
  {
    const cv::Mat mat = cv::Mat::zeros(2, 2, CV_32F);
    EXPECT_THROW(ORB_SLAM3::Converter::toEigenVector3f(mat), std::runtime_error);
  }
  // cv::Mat to Eigen::Matrix3f
  {
    const Eigen::Matrix3f matrix = ORB_SLAM3::Converter::toEigenMatrix3f(kRotationCv);
    EXPECT_TRUE(matrix.isApprox(kRotationEigen));
  }
  {
    const cv::Mat mat = cv::Mat::zeros(2, 2, CV_32F);
    EXPECT_THROW(ORB_SLAM3::Converter::toEigenMatrix3f(mat), std::runtime_error);
  }
}

TEST(Converter, toStd) {
  // quaternion from cv::Mat to std::vector<float>
  {
    const std::vector<float> vector = ORB_SLAM3::Converter::toQuaternion(kRotationCv);
    const std::vector<float> expected
      = {kQuaternionEigen.x(), kQuaternionEigen.y(), kQuaternionEigen.z(), kQuaternionEigen.w()};
    EXPECT_TRUE(isEqual(vector, expected));
  }
}

TEST(Converter, toSophus) {
  // cv::Mat to Sophus::SE3f
  {
    const Sophus::SE3f se3 = ORB_SLAM3::Converter::toSophus(kTransformationCv);
    const Sophus::SE3f expected(kQuaternionEigen, kTranslationEigen);
    EXPECT_TRUE(se3.matrix().isApprox(expected.matrix()));
  }
  {
    const cv::Mat mat = cv::Mat::zeros(2, 2, CV_32F);
    EXPECT_THROW(ORB_SLAM3::Converter::toSophus(mat), std::runtime_error);
  }
  // g2o::Sim3 to Sophus::Sim3f
  {
    const double scale = 2.0;
    const g2o::Sim3 sim3_g2o(
      kQuaternionEigen.cast<double>(),
      kTranslationEigen.cast<double>(),
      scale
    );
    const Sophus::Sim3f sim3_sophus = ORB_SLAM3::Converter::toSophus(sim3_g2o);
    Eigen::Matrix4f expected = Eigen::Matrix4f::Zero();
    expected.block<3, 3>(0, 0) = kRotationEigen * scale;
    expected.block<3, 1>(0, 3) = kTranslationEigen;
    expected(3, 3) = 1.0f;
    EXPECT_TRUE(sim3_sophus.matrix().isApprox(expected));
  }
}

TEST(Converter, Other) {
  // cv::Mat to std::vector<float> for quaternion
  {
    const std::vector<float> vec = ORB_SLAM3::Converter::toQuaternion(kRotationCv);
    const std::vector<float> expected = {
      kQuaternionEigen.x(),
      kQuaternionEigen.y(),
      kQuaternionEigen.z(),
      kQuaternionEigen.w()
    };
    EXPECT_TRUE(isEqual(vec, expected));
  }
  {
    const cv::Mat mat = cv::Mat::zeros(2, 2, CV_32F);
    EXPECT_THROW(ORB_SLAM3::Converter::toQuaternion(mat), std::runtime_error);
  }
  // cv::Mat to std::vector<cv::Mat> for descriptors
  {
    const cv::Mat mat = (
      cv::Mat_<float>(3, 3) << 1.f, 2.f, 3.f,
                               4.f, 5.f, 6.f,
                               7.f, 8.f, 9.f
    );
    const std::vector<cv::Mat> vec = ORB_SLAM3::Converter::toDescriptorVector(mat);
    const std::vector<cv::Mat> expected = {
      (cv::Mat_<float>(1, 3) << 1.f, 2.f, 3.f),
      (cv::Mat_<float>(1, 3) << 4.f, 5.f, 6.f),
      (cv::Mat_<float>(1, 3) << 7.f, 8.f, 9.f)
    };
    EXPECT_EQ(vec.size(), expected.size());
    for (std::size_t i = 0; i < vec.size(); i++) {
      EXPECT_TRUE(isEqual(vec[i], expected[i]));
    }
  }
}
