#include "Common/TUM.h"
#include <fstream>

namespace ORB_SLAM3::TUM {

void LoadMonocularImages(
  const std::string&        strFile,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
) {
  std::ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  std::string s0;
  std::getline(f, s0);
  std::getline(f, s0);
  std::getline(f, s0);

  while (!f.eof()) {
    std::string s;
    std::getline(f, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double      t;
      std::string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenames.push_back(sRGB);
    }
  }
}

void LoadRGBDImages(
  const std::string&        strAssociationFilename,
  std::vector<std::string>& vstrImageFilenamesRGB,
  std::vector<std::string>& vstrImageFilenamesD,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  while (!fAssociation.eof()) {
    std::string s;
    std::getline(fAssociation, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double      t;
      std::string sRGB, sD;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
  }
}

} // namespace ORB_SLAM3::TUM
