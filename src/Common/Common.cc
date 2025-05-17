#include "Common/Common.h"

namespace ORB_SLAM3 {

bool IsInertialBased(Sensor sensor) {
  // clang-format off
  return sensor == Sensor::InertialMonocular
      || sensor == Sensor::InertialStereo
      || sensor == Sensor::InertialRGBD;
  // clang-format on
}

} // namespace ORB_SLAM3
