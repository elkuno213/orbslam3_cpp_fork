# ──────────────────────────────────────────────────────────────────────────── #
# Dependencies                                                                 #

find_package(realsense2)
include_directories(
  ${realsense_INCLUDE_DIR}
)

# ──────────────────────────────────────────────────────────────────────────── #
# Targets                                                                      #

# rgbd_tum
add_executable(rgbd_tum "RGB-D/rgbd_tum.cc")
target_link_libraries(rgbd_tum PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# stereo_kitti
add_executable(stereo_kitti Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# stereo_euroc
add_executable(stereo_euroc Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# stereo_tum_vi
add_executable(stereo_tum_vi Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# mono_tum
add_executable(mono_tum Monocular/mono_tum.cc)
target_link_libraries(mono_tum PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# mono_kitti
add_executable(mono_kitti Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# mono_euroc
add_executable(mono_euroc Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# mono_tum_vi
add_executable(mono_tum_vi Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# mono_inertial_euroc
add_executable(mono_inertial_euroc Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# mono_inertial_tum_vi
add_executable(mono_inertial_tum_vi Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# stereo_inertial_euroc
add_executable(stereo_inertial_euroc Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

# stereo_inertial_tum_vi
add_executable(stereo_inertial_tum_vi Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

if(realsense2_FOUND)
  # rgbd_realsense_D435i
  add_executable(rgbd_realsense_D435i RGB-D/rgbd_realsense_D435i.cc)
  target_link_libraries(rgbd_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # rgbd_inertial_realsense_D435i
  add_executable(rgbd_inertial_realsense_D435i RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
  target_link_libraries(rgbd_inertial_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # stereo_realsense_t265
  add_executable(stereo_realsense_t265 Stereo/stereo_realsense_t265.cc)
  target_link_libraries(stereo_realsense_t265 PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # stereo_realsense_D435i
  add_executable(stereo_realsense_D435i Stereo/stereo_realsense_D435i.cc)
  target_link_libraries(stereo_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # mono_realsense_t265
  add_executable(mono_realsense_t265 Monocular/mono_realsense_t265.cc)
  target_link_libraries(mono_realsense_t265 PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # mono_realsense_D435i
  add_executable(mono_realsense_D435i Monocular/mono_realsense_D435i.cc)
  target_link_libraries(mono_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # mono_inertial_realsense_t265
  add_executable(mono_inertial_realsense_t265 Monocular-Inertial/mono_inertial_realsense_t265.cc)
  target_link_libraries(mono_inertial_realsense_t265 PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # mono_inertial_realsense_D435i
  add_executable(mono_inertial_realsense_D435i Monocular-Inertial/mono_inertial_realsense_D435i.cc)
  target_link_libraries(mono_inertial_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # stereo_inertial_realsense_t265
  add_executable(stereo_inertial_realsense_t265 Stereo-Inertial/stereo_inertial_realsense_t265.cc)
  target_link_libraries(stereo_inertial_realsense_t265 PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # stereo_inertial_realsense_D435i
  add_executable(stereo_inertial_realsense_D435i Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
  target_link_libraries(stereo_inertial_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # recorder_realsense_D435i
  add_executable(recorder_realsense_D435i Calibration/recorder_realsense_D435i.cc)
  target_link_libraries(recorder_realsense_D435i PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})

  # recorder_realsense_T265
  add_executable(recorder_realsense_T265 Calibration/recorder_realsense_T265.cc)
  target_link_libraries(recorder_realsense_T265 PRIVATE ${PROJECT_NAME} ${realsense2_LIBRARY})
endif()
