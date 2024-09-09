# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

file(GLOB_RECURSE ORBSLAM3_SRC
  "${PROJECT_SOURCE_DIR}/orbslam3/*.cc"
  "${PROJECT_SOURCE_DIR}/orbslam3/*.cpp"
)

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
  DBoW2
  g2o
  sophus
)
set_target_properties(orbslam3 PROPERTIES
  PREFIX        "lib"
  OUTPUT_NAME   "orbslam3"
  INSTALL_RPATH "$ORIGIN"
)
