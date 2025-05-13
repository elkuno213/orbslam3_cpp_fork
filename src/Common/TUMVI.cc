#include "Common/TUMVI.h"
#include <fstream>

namespace ORB_SLAM3::TUMVI {

void LoadMonocularImages(
  const std::string&        strImagePath,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImages,
  std::vector<double>&      vTimeStamps
) {
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);

    if (!s.empty()) {
      if (s[0] == '#') {
        continue;
      }

      int         pos  = s.find(' ');
      std::string item = s.substr(0, pos);

      vstrImages.push_back(strImagePath + "/" + item + ".png");
      double t = std::stod(item);
      vTimeStamps.push_back(t * 1e-9);
    }
  }
}

void LoadStereoImages(
  const std::string&        strPathLeft,
  const std::string&        strPathRight,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimeStamps
) {
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);

    if (!s.empty()) {
      if (s[0] == '#') {
        continue;
      }

      int         pos  = s.find(' ');
      std::string item = s.substr(0, pos);

      vstrImageLeft.push_back(strPathLeft + "/" + item + ".png");
      vstrImageRight.push_back(strPathRight + "/" + item + ".png");

      double t = std::stod(item);
      vTimeStamps.push_back(t * 1e-9);
    }
  }
}

void LoadIMU(
  const std::string&        strImuPath,
  std::vector<double>&      vTimeStamps,
  std::vector<cv::Point3f>& vAcc,
  std::vector<cv::Point3f>& vGyro
) {
  std::ifstream fImu;
  fImu.open(strImuPath.c_str());
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);

  while (!fImu.eof()) {
    std::string s;
    std::getline(fImu, s);
    if (s[0] == '#') {
      continue;
    }

    if (!s.empty()) {
      std::string item;
      std::size_t pos = 0;
      double      data[7];
      int         count = 0;
      while ((pos = s.find(',')) != std::string::npos) {
        item          = s.substr(0, pos);
        data[count++] = std::stod(item);
        s.erase(0, pos + 1);
      }
      item    = s.substr(0, pos);
      data[6] = std::stod(item);

      vTimeStamps.push_back(data[0] * 1e-9);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}

} // namespace ORB_SLAM3::TUMVI
