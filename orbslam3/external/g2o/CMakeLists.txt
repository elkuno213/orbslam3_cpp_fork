# ──────────────────────────────────────────────────────────────────────────── #
# Project                                                                      #

cmake_minimum_required(VERSION 2.6)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
project(g2o)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# ──────────────────────────────────────────────────────────────────────────── #
# Configuration                                                                #

# Default built type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Flags.
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -W")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W")
set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -O3 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")

# Generate config.h
configure_file(config.h.in ${CMAKE_CURRENT_BINARY_DIR}/g2o/config.h)

# Add options.
option(BUILD_TESTS "Build tests" OFF)

# ──────────────────────────────────────────────────────────────────────────── #
# Dependencies                                                                 #

find_package(Eigen3 3.1.0 REQUIRED)

# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

# Prepare the sources for the library and tests.
file(GLOB_RECURSE G2O_TEST_SRCS "g2o/*_test.cpp" "g2o/*_test.cc" "g2o/*_test.c")
file(GLOB_RECURSE G2O_SRCS      "g2o/*.cpp"      "g2o/*.cc"      "g2o/*.c"     )
list(REMOVE_ITEM G2O_SRCS ${G2O_TEST_SRCS})

# Add the library.
add_library(g2o SHARED)
target_sources(g2o PRIVATE ${G2O_SRCS})
target_include_directories(g2o PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for config.h
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/orbslam3/external/g2o>
)
target_link_libraries(g2o PUBLIC
  Eigen3::Eigen
)
set_target_properties(g2o PROPERTIES
  PREFIX        "lib"
  OUTPUT_NAME   "orbslam3_external_g2o"
  INSTALL_RPATH "$ORIGIN"
)
if(UNIX)
  target_compile_options(g2o PRIVATE -DUNIX)
endif()

# Add tests.
if(BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  include(GoogleTest)

  foreach(test_src ${G2O_TEST_SRCS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name})
    target_sources(${test_name} PRIVATE ${test_src})
    target_link_libraries(${test_name} PRIVATE GTest::gtest GTest::gtest_main g2o)
    gtest_discover_tests(${test_name})
  endforeach()
endif()

# ──────────────────────────────────────────────────────────────────────────── #
# Install and export                                                           #

install(
  TARGETS g2o
  EXPORT orbslam3Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  FILES       ${CMAKE_CURRENT_BINARY_DIR}/g2o/config.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/orbslam3/external/g2o/g2o
)
