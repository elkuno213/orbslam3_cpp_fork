# ──────────────────────────────────────────────────────────────────────────── #
# Project                                                                      #

cmake_minimum_required(VERSION 2.8)
project(DBoW2)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# ──────────────────────────────────────────────────────────────────────────── #
# Configuration                                                                #

# Default built type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags.
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# ──────────────────────────────────────────────────────────────────────────── #
# Dependencies                                                                 #

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
  endif()
endif()

find_package(Boost REQUIRED COMPONENTS serialization)

# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

file(GLOB DUTILS_SRCS "DUtils/*.cpp")
file(GLOB DBOW2_SRCS  "DBoW2/*.cpp" )
add_library(DBoW2 SHARED)
target_sources(DBoW2 PRIVATE ${DBOW2_SRCS} ${DUTILS_SRCS})
target_include_directories(DBoW2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(DBoW2 PUBLIC
  ${OpenCV_LIBRARIES}
  Boost::serialization
)
set_target_properties(DBoW2 PROPERTIES
  PREFIX        "lib"
  OUTPUT_NAME   "orbslam3_external_dbow2"
  INSTALL_RPATH "$ORIGIN"
)

# ──────────────────────────────────────────────────────────────────────────── #
# Install and export                                                           #

install(
  TARGETS DBoW2
  EXPORT orbslam3Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
