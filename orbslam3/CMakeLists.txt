# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

# Prepare sources for the library and tests.

file(GLOB_RECURSE ORBSLAM3_EXTERNAL_SRC
  "${PROJECT_SOURCE_DIR}/orbslam3/external/*.cc"
  "${PROJECT_SOURCE_DIR}/orbslam3/external/*.cpp"
  "${PROJECT_SOURCE_DIR}/orbslam3/external/*.c"
)

file(GLOB_RECURSE ORBSLAM3_TEST_SRC
  "${PROJECT_SOURCE_DIR}/orbslam3/*_test.cc"
  "${PROJECT_SOURCE_DIR}/orbslam3/*_test.cpp"
  "${PROJECT_SOURCE_DIR}/orbslam3/*_test.c"
)
list(REMOVE_ITEM ORBSLAM3_TEST_SRC ${ORBSLAM3_EXTERNAL_SRC}) # Remove external files from tests

file(GLOB_RECURSE ORBSLAM3_SRC
  "${PROJECT_SOURCE_DIR}/orbslam3/*.cc"
  "${PROJECT_SOURCE_DIR}/orbslam3/*.cpp"
  "${PROJECT_SOURCE_DIR}/orbslam3/*.c"
)
list(REMOVE_ITEM ORBSLAM3_SRC ${ORBSLAM3_TEST_SRC}) # Remove test files from sources
list(REMOVE_ITEM ORBSLAM3_SRC ${ORBSLAM3_EXTERNAL_SRC}) # Remove external files from sources

# Add the library target.
add_library(orbslam3 SHARED)
target_sources(orbslam3 PRIVATE ${ORBSLAM3_SRC})
target_include_directories(orbslam3
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(orbslam3 PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  Boost::serialization
  OpenSSL::Crypto
  ${Pangolin_LIBRARIES}
  glog::glog
  DBoW2
  g2o
  sophus
)
set_target_properties(orbslam3 PROPERTIES
  PREFIX        "lib"
  OUTPUT_NAME   "orbslam3"
  INSTALL_RPATH "$ORIGIN"
)

# Add the tests.
if(BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  include(GoogleTest)

  foreach(test_src ${ORBSLAM3_TEST_SRC})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name})
    target_sources(${test_name} PRIVATE ${test_src})
    target_link_libraries(${test_name} PRIVATE GTest::gtest GTest::gtest_main orbslam3)
    add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()
