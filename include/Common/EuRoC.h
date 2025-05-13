#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace ORB_SLAM3::EuRoC {

void LoadMonocularImages(
  const std::string&        strImagePath,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImages,
  std::vector<double>&      vTimeStamps
);

void LoadStereoImages(
  const std::string&        strPathLeft,
  const std::string&        strPathRight,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimeStamps
);

void LoadIMU(
  const std::string&        strImuPath,
  std::vector<double>&      vTimeStamps,
  std::vector<cv::Point3f>& vAcc,
  std::vector<cv::Point3f>& vGyro
);

} // namespace ORB_SLAM3::EuRoC
