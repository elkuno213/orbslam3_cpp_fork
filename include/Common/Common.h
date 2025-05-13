#pragma once

namespace ORB_SLAM3 {

// TODO(VuHoi): group with System::eSensor
enum class Sensor {
  Monocular         = 0,
  Stereo            = 1,
  RGBD              = 2,
  InertialMonocular = 3,
  InertialStereo    = 4,
  InertialRGBD      = 5,
};

} // namespace ORB_SLAM3
