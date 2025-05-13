#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace ORB_SLAM3::TUM {

void LoadMonocularImages(
  const std::string&        strFile,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
);

void LoadRGBDImages(
  const std::string&        strAssociationFilename,
  std::vector<std::string>& vstrImageFilenamesRGB,
  std::vector<std::string>& vstrImageFilenamesD,
  std::vector<double>&      vTimestamps
);

} // namespace ORB_SLAM3::TUM
