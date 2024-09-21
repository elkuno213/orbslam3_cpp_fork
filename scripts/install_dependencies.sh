#!/bin/bash


sudo apt-get install -y \
  stow \
  libeigen3-dev \
  libopencv-dev \
  libpython3-dev \
  libunwind-dev \
  libgoogle-glog-dev \
  libboost-serialization-dev


# Optional: install librealsense2
# Register the server's public key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
# Add the server to the list of repositories
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
# Install the libraries
sudo apt-get install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev
