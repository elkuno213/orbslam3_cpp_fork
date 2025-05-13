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
  std::string vocabulary_file, settings_file, sequence_dir, output_dir;

  const bool args_ok = ORB_SLAM3::TUM::ParseArguments(
    argc,
    argv,
    vocabulary_file,
    settings_file,
    sequence_dir,
    output_dir
  );
  if (!args_ok) {
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageFilenames;
  std::vector<double>      vTimestamps;
  std::string              strFile = sequence_dir + "/rgb.txt";
  ORB_SLAM3::TUM::LoadMonocularImages(strFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocabulary_file, settings_file, ORB_SLAM3::System::MONOCULAR, true);
  float             imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  double t_resize = 0.f;
  double t_track  = 0.f;

  // Main loop
  cv::Mat im;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image from file
    im = cv::imread(
      sequence_dir + "/" + vstrImageFilenames[ni],
      cv::IMREAD_UNCHANGED
    ); //,cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (im.empty()) {
      std::cerr << std::endl
                << "Failed to load image at: " << sequence_dir << "/"
                << vstrImageFilenames[ni] << std::endl;
      return 1;
    }

    if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
      int width  = im.cols * imageScale;
      int height = im.rows * imageScale;
      cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
      t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                   t_End_Resize - t_Start_Resize
      )
                   .count();
      SLAM.InsertResizeTime(t_resize);
#endif
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im, tframe);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
    t_track
      = t_resize
      + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
    SLAM.InsertTrackTime(t_track);
#endif

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
  const fs::path output_file_path = fs::path(output_dir) / "KeyFrameTrajectory.txt";
  SLAM.SaveKeyFrameTrajectoryTUM(output_file_path.string());

  return 0;
}
