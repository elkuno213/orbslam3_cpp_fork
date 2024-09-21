// 3rdparty
#include <glog/logging.h>
#include <orbslam3/G2oTypes.h>

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Hello, I am a consumer of the orbslam3 library!";
  ORB_SLAM3::Vector6d v6d;
  v6d << 1, 2, 3, 4, 5, 6;
  LOG(INFO) << "v6d";
  LOG(INFO) << v6d;

  return 0;
}
