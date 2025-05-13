#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace ORB_SLAM3::KITTI {

void LoadMonocularImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
);

void LoadStereoImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimestamps
);

} // namespace ORB_SLAM3::KITTI
