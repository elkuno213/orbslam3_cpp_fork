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

inline bool isEqual(const std::vector<float>& one, const std::vector<float>& other) {
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
  // g2o::SE3Quat to cv::Mat
  {
    const g2o::SE3Quat se3(kQuaternionEigen.cast<double>(), kTranslationEigen.cast<double>());
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(se3);
    EXPECT_TRUE(isEqual(mat, kTransformationCv));
  }
  // g2o::Sim3 to cv::Mat
  {
    const double scale = 2.0;
    g2o::Sim3 sim3(kQuaternionEigen.cast<double>(), kTranslationEigen.cast<double>(), scale);
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(sim3);
    cv::Mat expected = cv::Mat::zeros(4, 4, CV_32F);
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        expected.at<float>(r, c) = kRotationCv.at<float>(r, c) * scale;
      }
      expected.at<float>(r, 3) = kTranslationEigen(r);
    }
    expected.at<float>(3, 3) = 1.f;
    EXPECT_TRUE(isEqual(mat, expected));
  }
  // Eigen::Matrix<float, 4, 4> to cv::Mat
  {
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(kTransformationEigen);
    EXPECT_TRUE(isEqual(mat, kTransformationCv));
  }
  // Eigen::Matrix<float, 3, 4> to cv::Mat
  {
    Eigen::Matrix<float, 3, 4> matrix = (
      Eigen::Matrix<float, 3, 4>() << 1.f,  2.f,  3.f,  4.f,
                                      5.f,  6.f,  7.f,  8.f,
                                      9.f, 10.f, 11.f, 12.f
    ).finished();
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(matrix);
    const cv::Mat expected = (
      cv::Mat_<float>(3, 4) << 1.f,  2.f,  3.f,  4.f,
                               5.f,  6.f,  7.f,  8.f,
                               9.f, 10.f, 11.f, 12.f
    );
    EXPECT_TRUE(isEqual(mat, expected));
  }
  // Eigen::Matrix3f to cv::Mat
  {
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(kRotationEigen);
    EXPECT_TRUE(isEqual(mat, kRotationCv));
  }
  // Eigen::MatrixXf to cv::Mat
  {
    const std::size_t rows = 2;
    const std::size_t cols = 4;
    const Eigen::MatrixXf matrix = (
      Eigen::MatrixXf(rows, cols) << 1.f, 2.f, 3.f, 4.f,
                                     5.f, 6.f, 7.f, 8.f
    ).finished();
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(matrix);
    const cv::Mat expected = (
      cv::Mat_<float>(rows, cols) << 1.f, 2.f, 3.f, 4.f,
                                     5.f, 6.f, 7.f, 8.f
    );
    EXPECT_TRUE(isEqual(mat, expected));
  }
  // Eigen::Matrix<float, 3, 1> to cv::Mat
  {
    const Eigen::Matrix<float, 3, 1> matrix = kTranslationEigen;
    const cv::Mat mat = ORB_SLAM3::Converter::toCvMat(matrix);
    EXPECT_TRUE(isEqual(mat, kTranslationCv));
  }
  // Eigen::Matrix<double, 3, 3> R and Eigen::Matrix<double, 3, 1> t to cv::Mat
  {
    const cv::Mat mat = ORB_SLAM3::Converter::toCvSE3(
      kRotationEigen.cast<double>(),
      kTranslationEigen.cast<double>()
    );
    EXPECT_TRUE(isEqual(mat, kTransformationCv));
  }
}

TEST(Converter, toEigen) {
  // cv::Mat to Eigen::Matrix<float, 3, 1>
  {
    const Eigen::Matrix<float, 3, 1> matrix = ORB_SLAM3::Converter::toVector3f(kTranslationCv);
    const Eigen::Matrix<float, 3, 1> expected = kTranslationEigen;
    EXPECT_TRUE(matrix.isApprox(expected));
  }
  // cv::Point3f to Eigen::Matrix<double, 3, 1>
  {
    const Eigen::Matrix<double, 3, 1> matrix = ORB_SLAM3::Converter::toVector3d(kTranslationCv);
    const Eigen::Matrix<double, 3, 1> expected = kTranslationEigen.cast<double>();
    EXPECT_TRUE(matrix.isApprox(expected));
  }
  // cv::Mat to Eigen::Matrix<float, 3, 3>
  {
    const Eigen::Matrix<float, 3, 3> matrix = ORB_SLAM3::Converter::toMatrix3f(kRotationCv);
    EXPECT_TRUE(matrix.isApprox(kRotationEigen));
  }
  // cv::Mat to Eigen::Matrix<float, 4, 4>
  {
    const Eigen::Matrix<float, 4, 4> matrix = ORB_SLAM3::Converter::toMatrix4f(kTransformationCv);
    EXPECT_TRUE(matrix.isApprox(kTransformationEigen));
  }
}

TEST(Converter, toStd) {
  // quaternion from cv::Mat to std::vector
  {
    const std::vector<float> vector = ORB_SLAM3::Converter::toQuaternion(kRotationCv);
    const std::vector<float> expected
      = {kQuaternionEigen.x(), kQuaternionEigen.y(), kQuaternionEigen.z(), kQuaternionEigen.w()};
    EXPECT_TRUE(isEqual(vector, expected));
  }
  // euler angles from cv::Mat to std::vector
  {
    const std::vector<float> vector = ORB_SLAM3::Converter::toEuler(kRotationCv);
    EXPECT_TRUE(isEqual(vector, kEulerStd));
  }
}

TEST(Converter, toSophus) {
  // cv::Mat to Sophus::SE3<float>
  {
    const Sophus::SE3<float> se3 = ORB_SLAM3::Converter::toSophus(kTransformationCv);
    const Sophus::SE3<float> expected(kQuaternionEigen, kTranslationEigen);
    EXPECT_TRUE(se3.matrix().isApprox(expected.matrix()));
  }
  // g2o::Sim3 to Sophus::Sim3f
  {
    const double scale = 2.0;
    g2o::Sim3 sim3(kQuaternionEigen.cast<double>(), kTranslationEigen.cast<double>(), scale);
    const Sophus::Sim3f sim3f = ORB_SLAM3::Converter::toSophus(sim3);
    Eigen::Matrix4f expected = Eigen::Matrix4f::Zero();
    expected.block<3, 3>(0, 0) = kRotationEigen * scale;
    expected.block<3, 1>(0, 3) = kTranslationEigen;
    expected(3, 3) = 1.0f;
    EXPECT_TRUE(sim3f.matrix().isApprox(expected));
  }
}

TEST(Converter, toG2O) {
  // cv::Mat to g2o::SE3Quat
  {
    const g2o::SE3Quat se3 = ORB_SLAM3::Converter::toSE3Quat(kTransformationCv);
    const g2o::SE3Quat expected(kQuaternionEigen.cast<double>(), kTranslationEigen.cast<double>());
    EXPECT_TRUE(isEqual(se3.toVector(), expected.toVector()));
  }
  // Sophus::SE3f to g2o::SE3Quat
  {
    const Sophus::SE3f se3(kQuaternionEigen, kTranslationEigen);
    const g2o::SE3Quat g2o_se3 = ORB_SLAM3::Converter::toSE3Quat(se3);
    const g2o::SE3Quat expected(kQuaternionEigen.cast<double>(), kTranslationEigen.cast<double>());
    EXPECT_TRUE(isEqual(g2o_se3.toVector(), expected.toVector()));
  }
}

TEST(Converter, Other) {
  // Check rotation matrix
  {
    EXPECT_TRUE(ORB_SLAM3::Converter::isRotationMatrix(kRotationCv));
  }
  // vector (cv::Mat) to skew matrix (cv::Mat)
  {
    const cv::Mat vector = (cv::Mat_<float>(3, 1) << 1.f, 2.f, 3.f);
    const cv::Mat mat = ORB_SLAM3::Converter::tocvSkewMatrix(vector);
    const cv::Mat expected = (
      cv::Mat_<float>(3, 3) << 0.f, -3.f, 2.f,
                               3.f,  0.f, -1.f,
                              -2.f,  1.f, 0.f
    );
    EXPECT_TRUE(isEqual(mat, expected));
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
