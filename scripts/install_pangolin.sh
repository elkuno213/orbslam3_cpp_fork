#!/bin/bash

# Install the Pangolin.
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin
cd /tmp/Pangolin
yes | ./scripts/install_prerequisites.sh recommended
cmake -B build \
      -S . \
      -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
      -DPython_EXECUTABLE=`which python3` \
      -DBUILD_TESTS=ON
ninja -C build
ninja -C build test
sudo ninja -C build install
