#include <iostream>

#include <orbslam3/G2oTypes.h>

int main() {
  std::cout << "Hello, I am a consumer of the orbslam3 library!\n";

  ORB_SLAM3::Vector6d v6d;
  v6d << 1, 2, 3, 4, 5, 6;
  std::cout << "v6d: \n" << v6d << std::endl;

  return 0;
}
