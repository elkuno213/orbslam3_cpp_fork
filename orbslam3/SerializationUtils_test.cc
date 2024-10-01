// Standard
#include <sstream>
// 3rdparty
#include <gtest/gtest.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
// Local
#include "orbslam3/SerializationUtils.h"

class SerializeSophusSE3Test : public ::testing::Test {
protected:
  void SetUp() override {
    // quaternion = [0.6830127, 0.1830127, 0.5, 0.5]
    const Eigen::Quaternionf quaternion
      = Eigen::AngleAxisf( M_PI / 3., Eigen::Vector3f::UnitZ())  // 60° around z-axis
      * Eigen::AngleAxisf(-M_PI / 6., Eigen::Vector3f::UnitY())  // -30° around y-axis
      * Eigen::AngleAxisf( M_PI / 2., Eigen::Vector3f::UnitX()); // 90° around x-axis
    const Eigen::Vector3f translation = Eigen::Vector3f(1.f, 2.f, 3.f);
    original = Sophus::SE3f(quaternion, translation);
  }

  void TearDown() override {}

  void Archive(Sophus::SE3f& loaded) {
    std::stringstream ss;
    {
      boost::archive::text_oarchive oa(ss);
      ORB_SLAM3::serializeSophusSE3(oa, original, 0);
    }
    {
      boost::archive::text_iarchive ia(ss);
      ORB_SLAM3::serializeSophusSE3(ia, loaded, 0);
    }
  }

  Sophus::SE3f original;
};

TEST_F(SerializeSophusSE3Test, Archive) {
  Sophus::SE3f loaded;
  Archive(loaded);

  EXPECT_EQ(original.rotationMatrix(), loaded.rotationMatrix());
  EXPECT_EQ(original.translation()   , loaded.translation()   );
}

class SerializeMatrixTest : public ::testing::Test {
protected:
  void SetUp() override {
    original = (cv::Mat_<float>(2, 3) << 1.f, 2.f, 3.f, 4.f, 5.f, 6.f);
  }

  void TearDown() override {}

  void ArchiveReference(cv::Mat& loaded) {
    std::stringstream ss;
    {
      boost::archive::text_oarchive oa(ss);
      ORB_SLAM3::serializeMatrix(oa, original, 0);
    }
    {
      boost::archive::text_iarchive ia(ss);
      ORB_SLAM3::serializeMatrix(ia, loaded, 0);
    }
  }

  void ArchiveConstReference(const cv::Mat& loaded) {
    std::stringstream ss;
    {
      boost::archive::text_oarchive oa(ss);
      ORB_SLAM3::serializeMatrix(oa, original, 0);
    }
    {
      boost::archive::text_iarchive ia(ss);
      ORB_SLAM3::serializeMatrix(ia, loaded, 0);
    }
  }

  cv::Mat original;
};

TEST_F(SerializeMatrixTest, ArchiveReference) {
  cv::Mat loaded = cv::Mat::zeros(1, 1, CV_32F);
  ArchiveReference(loaded);

  EXPECT_EQ(original.cols          , loaded.cols          );
  EXPECT_EQ(original.rows          , loaded.rows          );
  EXPECT_EQ(original.type()        , loaded.type()        );
  EXPECT_EQ(original.isContinuous(), loaded.isContinuous());
  EXPECT_EQ(original.elemSize()    , loaded.elemSize()    );

  EXPECT_EQ(original.at<float>(0, 0), loaded.at<float>(0, 0));
  EXPECT_EQ(original.at<float>(0, 1), loaded.at<float>(0, 1));
  EXPECT_EQ(original.at<float>(0, 2), loaded.at<float>(0, 2));
  EXPECT_EQ(original.at<float>(1, 0), loaded.at<float>(1, 0));
  EXPECT_EQ(original.at<float>(1, 1), loaded.at<float>(1, 1));
  EXPECT_EQ(original.at<float>(1, 2), loaded.at<float>(1, 2));
}

TEST_F(SerializeMatrixTest, ArchiveConstReference) {
  cv::Mat loaded = cv::Mat::zeros(1, 1, CV_32F);
  ArchiveConstReference(loaded);

  EXPECT_EQ(original.cols          , loaded.cols          );
  EXPECT_EQ(original.rows          , loaded.rows          );
  EXPECT_EQ(original.type()        , loaded.type()        );
  EXPECT_EQ(original.isContinuous(), loaded.isContinuous());
  EXPECT_EQ(original.elemSize()    , loaded.elemSize()    );

  EXPECT_EQ(original.at<float>(0, 0), loaded.at<float>(0, 0));
  EXPECT_EQ(original.at<float>(0, 1), loaded.at<float>(0, 1));
  EXPECT_EQ(original.at<float>(0, 2), loaded.at<float>(0, 2));
  EXPECT_EQ(original.at<float>(1, 0), loaded.at<float>(1, 0));
  EXPECT_EQ(original.at<float>(1, 1), loaded.at<float>(1, 1));
  EXPECT_EQ(original.at<float>(1, 2), loaded.at<float>(1, 2));
}

class SerializeKeyPointsTest : public ::testing::Test {
protected:
  void SetUp() override {
    original = {
      cv::KeyPoint{cv::Point2f(1.f, 2.f), 3.f, 4.f, 5.f, 6, 7},
      cv::KeyPoint{cv::Point2f(8.f, 9.f), 10.f, 11.f, 12.f, 13, 14},
      cv::KeyPoint{cv::Point2f(15.f, 16.f), 17.f, 18.f, 19.f, 20, 21}
    };
  }

  void TearDown() override {}

  void Archive(const std::vector<cv::KeyPoint>& loaded) {
    std::stringstream ss;
    {
      boost::archive::text_oarchive oa(ss);
      ORB_SLAM3::serializeKeyPoints(oa, original, 0);
    }
    {
      boost::archive::text_iarchive ia(ss);
      ORB_SLAM3::serializeKeyPoints(ia, loaded, 0);
    }
  }

  std::vector<cv::KeyPoint> original;
};

TEST_F(SerializeKeyPointsTest, Archive) {
  std::vector<cv::KeyPoint> loaded = {
    cv::KeyPoint{cv::Point2f(0.f, 0.f), 0.f, 0.f, 0.f, 0, 0},
    cv::KeyPoint{cv::Point2f(0.f, 0.f), 0.f, 0.f, 0.f, 0, 0},
    cv::KeyPoint{cv::Point2f(0.f, 0.f), 0.f, 0.f, 0.f, 0, 0},
    cv::KeyPoint{cv::Point2f(0.f, 0.f), 0.f, 0.f, 0.f, 0, 0}
  };
  Archive(loaded);

  EXPECT_EQ(original.size(), loaded.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    EXPECT_EQ(original[i].pt      , loaded[i].pt      );
    EXPECT_EQ(original[i].size    , loaded[i].size    );
    EXPECT_EQ(original[i].angle   , loaded[i].angle   );
    EXPECT_EQ(original[i].response, loaded[i].response);
    EXPECT_EQ(original[i].octave  , loaded[i].octave  );
    EXPECT_EQ(original[i].class_id, loaded[i].class_id);
  }
}
