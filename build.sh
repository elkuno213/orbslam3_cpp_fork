#!/bin/bash

echo "Installing dependencies ..."
./scripts/install_dependencies.sh
./scripts/install_pangolin.sh

echo "Uncompressing vocabulary ..."
tar -xzvf config/vocabulary.txt.tar.gz \
    -C config \
    --one-top-level=vocabulary.txt \
    --strip-components=1

echo "Configuring and building orbslam3 ..."
cmake -B build \
      -S . \
      -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=1
ninja -C build
sudo ninja -C build install
