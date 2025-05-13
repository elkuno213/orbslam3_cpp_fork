/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel
 * and Juan D. Tardós, University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M.
 * Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "Common/TUM.h"
#include "System.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  // Parse arguments.
  std::string vocabulary_file, settings_file, sequence_dir, association_file, output_dir;

  const bool args_ok = ORB_SLAM3::TUM::ParseArguments(
    argc,
    argv,
    vocabulary_file,
    settings_file,
    sequence_dir,
    association_file,
    output_dir
  );
  if (!args_ok) {
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageFilenamesRGB;
  std::vector<std::string> vstrImageFilenamesD;
  std::vector<double>      vTimestamps;
  ORB_SLAM3::TUM::LoadRGBDImages(
    association_file,
    vstrImageFilenamesRGB,
    vstrImageFilenamesD,
    vTimestamps
  );

  // Check consistency in the number of images and depthmaps
  int nImages = vstrImageFilenamesRGB.size();
  if (vstrImageFilenamesRGB.empty()) {
    std::cerr << std::endl << "No images found in provided path." << std::endl;
    return 1;
  } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
    std::cerr << std::endl << "Different number of images for rgb and depth." << std::endl;
    return 1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocabulary_file, settings_file, ORB_SLAM3::System::RGBD, true);
  float             imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  // Main loop
  cv::Mat imRGB, imD;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image and depthmap from file
    imRGB = cv::imread(
      sequence_dir + "/" + vstrImageFilenamesRGB[ni],
      cv::IMREAD_UNCHANGED
    ); //,cv::IMREAD_UNCHANGED);
    imD = cv::imread(
      sequence_dir + "/" + vstrImageFilenamesD[ni],
      cv::IMREAD_UNCHANGED
    ); //,cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (imRGB.empty()) {
      std::cerr << std::endl
                << "Failed to load image at: " << sequence_dir << "/" << vstrImageFilenamesRGB[ni]
                << std::endl;
      return 1;
    }

    if (imageScale != 1.f) {
      int width  = imRGB.cols * imageScale;
      int height = imRGB.rows * imageScale;
      cv::resize(imRGB, imRGB, cv::Size(width, height));
      cv::resize(imD, imD, cv::Size(width, height));
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the image to the SLAM system
    SLAM.TrackRGBD(imRGB, imD, tframe);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1) {
      T = vTimestamps[ni + 1] - tframe;
    } else if (ni > 0) {
      T = tframe - vTimestamps[ni - 1];
    }

    if (ttrack < T) {
      usleep((T - ttrack) * 1e6);
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  std::sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  std::cout << "-------" << std::endl << std::endl;
  std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << std::endl;
  std::cout << "mean tracking time: " << totaltime / nImages << std::endl;

  // Save camera trajectory
  fs::path output_file_path;
  output_file_path = fs::path(output_dir) / "CameraTrajectory.txt";
  SLAM.SaveTrajectoryTUM(output_file_path.string());
  output_file_path = fs::path(output_dir) / "KeyFrameTrajectory.txt";
  SLAM.SaveKeyFrameTrajectoryTUM(output_file_path.string());

  return 0;
}
