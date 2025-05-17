#include "Common/Common.h"
#include <gtest/gtest.h>

using namespace ORB_SLAM3;

TEST(CommonTest, IsInertialBased) {
  EXPECT_TRUE(IsInertialBased(Sensor::InertialMonocular));
  EXPECT_TRUE(IsInertialBased(Sensor::InertialStereo));
  EXPECT_TRUE(IsInertialBased(Sensor::InertialRGBD));

  EXPECT_FALSE(IsInertialBased(Sensor::Monocular));
  EXPECT_FALSE(IsInertialBased(Sensor::Stereo));
  EXPECT_FALSE(IsInertialBased(Sensor::RGBD));
}
