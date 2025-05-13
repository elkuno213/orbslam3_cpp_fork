#include "Common/KITTI.h"
#include <fstream>
#include <iomanip>

namespace ORB_SLAM3::KITTI {

void LoadMonocularImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fTimes;
  std::string   strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  std::string strPrefixLeft = strPathToSequence + "/image_0/";

  const int nTimes = vTimestamps.size();
  vstrImageFilenames.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
  }
}

void LoadStereoImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fTimes;
  std::string   strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  std::string strPrefixLeft  = strPathToSequence + "/image_0/";
  std::string strPrefixRight = strPathToSequence + "/image_1/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
  }
}

} // namespace ORB_SLAM3::KITTI
