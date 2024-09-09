# ──────────────────────────────────────────────────────────────────────────── #
# Project                                                                      #

cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(orbslam3 LANGUAGES CXX VERSION 1.0.0)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# ──────────────────────────────────────────────────────────────────────────── #
# Configuration                                                                #

# Set the build type.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set C/C++ flags.
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -march=native") # Optimize for the current CPU
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++17 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# Add options.
option(BUILD_EXAMPLES "Build examples" OFF)

# ──────────────────────────────────────────────────────────────────────────── #
# Dependencies                                                                 #

find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

add_subdirectory(orbslam3/external/DBoW2)
add_subdirectory(orbslam3/external/g2o)
add_subdirectory(orbslam3/external/Sophus)

# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

add_subdirectory(orbslam3)

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# ──────────────────────────────────────────────────────────────────────────── #
# Install and export                                                           #

install(
  TARGETS orbslam3
  EXPORT orbslam3Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/orbslam3ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(
  EXPORT orbslam3Targets
  FILE orbslam3Targets.cmake
  NAMESPACE orbslam3::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orbslam3
)

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/orbslam3Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/orbslam3Config.cmake
  @ONLY
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/orbslam3Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/orbslam3ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/orbslam3
)

install(
  DIRECTORY orbslam3/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/orbslam3
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
