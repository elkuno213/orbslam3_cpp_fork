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
#include <string>
#include <fmt/core.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "ImuTypes.h"
#include "LoggingUtils.h"
#include "System.h"

void LoadImages(
  const std::string&        strImagePath,
  const std::string&        strPathTimes,
  std::vector<std::string>& vstrImages,
  std::vector<double>&      vTimeStamps
);

void LoadIMU(
  const std::string&        strImuPath,
  std::vector<double>&      vTimeStamps,
  std::vector<cv::Point3f>& vAcc,
  std::vector<cv::Point3f>& vGyro
);

double ttrack_tot = 0;

int main(int argc, char* argv[]) {
  // Load env vars and args.
  spdlog::cfg::load_env_levels();
  spdlog::cfg::load_argv_levels(argc, argv);
  // Initialize application logger.
  ORB_SLAM3::logging::InitializeAppLogger("ORB-SLAM3", false);
  // Add file sink to the application logger.
  const std::string basename  = std::filesystem::path(argv[0]).stem().string();
  const std::string logfile   = fmt::format("/tmp/{}.log", basename);
  auto              file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile);
  spdlog::default_logger()->sinks().push_back(file_sink);

  if (argc < 5) {
    spdlog::critical("Invalid arguments provided.");
    spdlog::info(R"(
    Usage:
    ./mono_inertial_euroc <path-to-vocabulary-txt>
                          <path-to-settings-yaml>
                          <path-to-sequence-folder-1> <path-to-times-file-1>
                          [<path-to-sequence-folder-2> <path-to-times-file-2> ...]

    Arguments:
      <path-to-vocabulary-txt>    Path to the ORB vocabulary file (e.g., ORBvoc.txt).
      <path-to-settings-yaml>     Path to the settings file (e.g., EuRoC.yaml).
      <path-to-sequence-folder-N> Path to the sequence folder for dataset N.
      <path-to-times-file-N>      Path to the timestamps file for dataset N.

    Example:
      ./mono_inertial_euroc ORBvoc.txt EuRoC.yaml MH_01_easy mav0/cam0/times.txt
    )");
    return 1;
  }

  const int num_seq = (argc - 3) / 2;
  spdlog::info("Number of sequences: {}", num_seq);

  bool        bFileName = (((argc - 3) % 2) == 1);
  std::string file_name;
  if (bFileName) {
    file_name = std::string(argv[argc - 1]);
    spdlog::info("filename = {}", file_name);
  }

  // Load all sequences:
  int                               seq;
  std::vector<vector<std::string> > vstrImageFilenames;
  std::vector<vector<double> >      vTimestampsCam;
  std::vector<vector<cv::Point3f> > vAcc, vGyro;
  std::vector<vector<double> >      vTimestampsImu;
  std::vector<int>                  nImages;
  std::vector<int>                  nImu;
  std::vector<int>                  first_imu(num_seq, 0);

  vstrImageFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);
  vTimestampsImu.resize(num_seq);
  nImages.resize(num_seq);
  nImu.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    spdlog::info("Loading images for sequence {}...", seq);

    std::string pathSeq(argv[(2 * seq) + 3]);
    std::string pathTimeStamps(argv[(2 * seq) + 4]);

    std::string pathCam0 = pathSeq + "/mav0/cam0/data";
    std::string pathImu  = pathSeq + "/mav0/imu0/data.csv";

    LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
    spdlog::info("Images loaded!");

    spdlog::info("Loading IMU for sequence {}...", seq);
    LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
    spdlog::info("IMU data loaded!");

    nImages[seq] = vstrImageFilenames[seq].size();
    tot_images   += nImages[seq];
    nImu[seq]    = vTimestampsImu[seq].size();

    if ((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
      spdlog::critical("Failed to load images or IMU for sequence {}", seq);
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

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
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
        spdlog::critical("Failed to load image at: {}", vstrImageFilenames[seq][ni]);
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
        t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(
                     t_End_Resize - t_Start_Resize
        )
                     .count();
        SLAM.InsertResizeTime(t_resize);
#endif
      }

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0) {
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
      SLAM.TrackMonocular(im, tframe, vImuMeas); // TODO change to monocular_inertial

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
      t_track
        = t_resize
        + std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(t2 - t1).count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      ttrack_tot    += ttrack;

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
      spdlog::info("Changing the dataset...");
      SLAM.ChangeDataset();
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (bFileName) {
    const std::string kf_file = "kf_" + std::string(argv[argc - 1]) + ".txt";
    const std::string f_file  = "f_" + std::string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  }

  spdlog::drop_all();
  return 0;
}

void LoadImages(
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
      std::stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e9);
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

      vTimeStamps.push_back(data[0] / 1e9);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}
