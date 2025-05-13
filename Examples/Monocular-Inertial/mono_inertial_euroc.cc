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
#include "Common/EuRoC.h"
#include "ImuTypes.h"
#include "System.h"

namespace fs = std::filesystem;

double ttrack_tot = 0;

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
  std::vector<vector<std::string>> vstrImageFilenames;
  std::vector<vector<double>>      vTimestampsCam;
  std::vector<vector<cv::Point3f>> vAcc, vGyro;
  std::vector<vector<double>>      vTimestampsImu;
  std::vector<int>                 nImages;
  std::vector<int>                 nImu;
  std::vector<int>                 first_imu(num_seq, 0);

  vstrImageFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);
  vTimestampsImu.resize(num_seq);
  nImages.resize(num_seq);
  nImu.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    std::cout << "Loading images for sequence " << seq << "...";

    std::string pathSeq        = sequences[2 * seq];
    std::string pathTimeStamps = sequences[2 * seq + 1];

    std::string pathCam0 = pathSeq + "/mav0/cam0/data";
    std::string pathImu  = pathSeq + "/mav0/imu0/data.csv";

    ORB_SLAM3::EuRoC::LoadMonocularImages(
      pathCam0,
      pathTimeStamps,
      vstrImageFilenames[seq],
      vTimestampsCam[seq]
    );
    std::cout << "LOADED!" << std::endl;

    std::cout << "Loading IMU for sequence " << seq << "...";
    ORB_SLAM3::EuRoC::LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
    std::cout << "LOADED!" << std::endl;

    nImages[seq] = vstrImageFilenames[seq].size();
    tot_images   += nImages[seq];
    nImu[seq]    = vTimestampsImu[seq].size();

    if ((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
      std::cerr << "ERROR: Failed to load images or IMU for sequence" << seq << std::endl;
      return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first

    while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0]) {
      first_imu[seq]++;
    }
    first_imu[seq]--; // first imu measurement to be considered
  }

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  std::cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocabulary_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true);
  float             imageScale = SLAM.GetImageScale();

  double t_resize = 0.f;
  double t_track  = 0.f;

  int proccIm = 0;
  for (seq = 0; seq < num_seq; seq++) {
    // Main loop
    cv::Mat                            im;
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read image from file
      im = cv::imread(
        vstrImageFilenames[seq][ni],
        cv::IMREAD_UNCHANGED
      ); // CV_LOAD_IMAGE_UNCHANGED);

      double tframe = vTimestampsCam[seq][ni];

      if (im.empty()) {
        std::cerr << std::endl
                  << "Failed to load image at: " << vstrImageFilenames[seq][ni] << std::endl;
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

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0) {
        // std::cout << "t_cam " << tframe << std::endl;

        while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]) {
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(
            vAcc[seq][first_imu[seq]].x,
            vAcc[seq][first_imu[seq]].y,
            vAcc[seq][first_imu[seq]].z,
            vGyro[seq][first_imu[seq]].x,
            vGyro[seq][first_imu[seq]].y,
            vGyro[seq][first_imu[seq]].z,
            vTimestampsImu[seq][first_imu[seq]]
          ));
          first_imu[seq]++;
        }
      }

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      // Pass the image to the SLAM system
      // std::cout << "tframe = " << tframe << std::endl;
      SLAM.TrackMonocular(im, tframe, vImuMeas); // TODO change to monocular_inertial

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
      t_track
        = t_resize
        + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      ttrack_tot    += ttrack;
      // std::cout << "ttrack: " << ttrack << std::endl;

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
