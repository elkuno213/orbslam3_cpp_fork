# ──────────────────────────────────────────────────────────────────────────── #
# Project                                                                      #

cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(consumer LANGUAGES CXX)

# ──────────────────────────────────────────────────────────────────────────── #
# Configuration                                                                #

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ORBSLAM3_DIR "" CACHE STRING "Path to the orbslam3 installation directory")

# ──────────────────────────────────────────────────────────────────────────── #
# Dependencies                                                                 #

# Find orbslam3.
find_package(orbslam3 REQUIRED HINTS ${ORBSLAM3_DIR})

# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

add_executable(consumer consumer.cpp)
target_link_libraries(consumer PRIVATE orbslam3::orbslam3)
