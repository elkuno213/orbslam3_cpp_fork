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
#include "Common/EuRoC.h"
#include "System.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  // Parse arguments.
  std::string              vocabulary_file, settings_file, output_dir;
  std::vector<std::string> sequences;

  const bool args_ok = ORB_SLAM3::EuRoC::ParseArguments(
    argc,
    argv,
    vocabulary_file,
    settings_file,
    sequences,
    output_dir
  );
  if (!args_ok) {
    return 1;
  }

  const int num_seq = sequences.size() / 2;

  // Load all sequences:
  int                              seq;
  std::vector<vector<std::string>> vstrImageLeft;
  std::vector<vector<std::string>> vstrImageRight;
  std::vector<vector<double>>      vTimestampsCam;
  std::vector<int>                 nImages;

  vstrImageLeft.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    std::cout << "Loading images for sequence " << seq << "...";

    std::string pathSeq        = sequences[2 * seq];
    std::string pathTimeStamps = sequences[2 * seq + 1];

    std::string pathCam0 = pathSeq + "/mav0/cam0/data";
    std::string pathCam1 = pathSeq + "/mav0/cam1/data";

    ORB_SLAM3::EuRoC::LoadStereoImages(
      pathCam0,
      pathCam1,
      pathTimeStamps,
      vstrImageLeft[seq],
      vstrImageRight[seq],
      vTimestampsCam[seq]
    );
    std::cout << "LOADED!" << std::endl;

    nImages[seq] = vstrImageLeft[seq].size();
    tot_images   += nImages[seq];
  }

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  std::cout << std::endl << "-------" << std::endl;
  std::cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocabulary_file, settings_file, ORB_SLAM3::System::STEREO, true);

  cv::Mat imLeft, imRight;
  for (seq = 0; seq < num_seq; seq++) {
    // Seq loop
    double t_resize = 0;
    double t_rect   = 0;
    double t_track  = 0;
    int    num_rect = 0;
    int    proccIm  = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read left and right images from file
      imLeft  = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
      imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);

      if (imLeft.empty()) {
        std::cerr << std::endl
                  << "Failed to load image at: " << std::string(vstrImageLeft[seq][ni])
                  << std::endl;
        return 1;
      }

      if (imRight.empty()) {
        std::cerr << std::endl
                  << "Failed to load image at: " << std::string(vstrImageRight[seq][ni])
                  << std::endl;
        return 1;
      }

      double tframe = vTimestampsCam[seq][ni];

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      // Pass the images to the SLAM system
      SLAM.TrackStereo(
        imLeft,
        imRight,
        tframe,
        std::vector<ORB_SLAM3::IMU::Point>(),
        vstrImageLeft[seq][ni]
      );

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
      t_track
        = t_resize + t_rect
        + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[seq] - 1) {
        T = vTimestampsCam[seq][ni + 1] - tframe;
      } else if (ni > 0) {
        T = tframe - vTimestampsCam[seq][ni - 1];
      }

      if (ttrack < T) {
        usleep((T - ttrack) * 1e6); // 1e6
      }
    }

    if (seq < num_seq - 1) {
      std::cout << "Changing the dataset" << std::endl;

      SLAM.ChangeDataset();
    }
  }
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  fs::path output_file_path;
  output_file_path = fs::path(output_dir) / "CameraTrajectory.txt";
  SLAM.SaveTrajectoryEuRoC(output_file_path.string());
  output_file_path = fs::path(output_dir) / "KeyFrameTrajectory.txt";
  SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

  return 0;
}
