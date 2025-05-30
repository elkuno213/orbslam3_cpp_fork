#pragma once

#include <cstdint>
#include <Thirdparty/DBoW2/DBoW2/FORB.h>
#include <Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h>
#include <fmt/core.h>
#include <fmt/format.h>

namespace ORB_SLAM3 {

enum class TrackingState {
  NotReady       = -1,
  NoImagesYet    = 0,
  NotInitialized = 1,
  Tracking       = 2,
  RecentlyLost   = 3,
  Lost           = 4,
};

// Sensor types.
enum class Sensor {
  Monocular         = 0, // Monocular camera
  Stereo            = 1, // Stereo camera
  RGBD              = 2, // RGB-D camera
  InertialMonocular = 3, // IMU + Monocular camera
  InertialStereo    = 4, // IMU + Stereo camera
  InertialRGBD      = 5, // IMU + RGB-D camera
};

// Checks if sensor has IMU.
bool IsInertialBased(Sensor sensor);

// Check if sensor has monocular camera.
bool IsMonocularBased(Sensor sensor);

// Check if sensor has stereo camera.
bool IsStereoBased(Sensor sensor);

// Check if sensor has RGB-D camera.
bool IsRGBDBased(Sensor sensor);

// ID types.
using CameraID   = std::uint16_t;
using FrameID    = std::uint64_t;
using KeyFrameID = std::uint32_t;
using MapPointID = std::uint32_t;
using MapID      = std::uint16_t;

// Vocabulary type from DBoW2.
using ORBVocabulary = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;

} // namespace ORB_SLAM3

template <>
struct fmt::formatter<ORB_SLAM3::TrackingState> : fmt::formatter<std::string_view> {
  auto format(ORB_SLAM3::TrackingState s, format_context& ctx) const {
    std::string_view name = "Unknown";
    switch (s) {
      case ORB_SLAM3::TrackingState::NotReady:
        name = "NotReady";
        break;
      case ORB_SLAM3::TrackingState::NoImagesYet:
        name = "NoImagesYet";
        break;
      case ORB_SLAM3::TrackingState::NotInitialized:
        name = "NotInitialized";
        break;
      case ORB_SLAM3::TrackingState::Tracking:
        name = "Tracking";
        break;
      case ORB_SLAM3::TrackingState::RecentlyLost:
        name = "RecentlyLost";
        break;
      case ORB_SLAM3::TrackingState::Lost:
        name = "Lost";
        break;
    }
    return formatter<std::string_view>::format(name, ctx);
  }
};
