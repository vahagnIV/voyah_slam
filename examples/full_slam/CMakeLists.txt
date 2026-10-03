add_executable(monocular_full_slam
    monocular_full_slam.cpp
)
target_link_libraries(monocular_full_slam
    PRIVATE voyahslam_dataset_reader
)

add_executable(binocular_full_slam
    binocular_full_slam.cpp
)
target_link_libraries(binocular_full_slam
    PRIVATE voyahslam_dataset_reader
)
