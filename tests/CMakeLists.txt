add_executable(voyahslam_tests
    test_voyahslam.cpp
    utils_test.cpp
    algebra/algebra_test.cpp
    geometry/geometry_test.cpp
    geometry/rigid_transformation_test.cpp
    images/bilinear_interpolation_tests.cpp
    images/filter_tests.cpp
    images/gaussian_blur_tests.cpp
    images/resizer_tests.cpp
    features/monocular_measurement_test.cpp
    features/image_features_test.cpp
    features/keypoint_matches_test.cpp
    features/distortion_models/distortion_model_test.cpp
    features/distortion_models/kannala_brandt_distortion_model_test.cpp
    memory/object_pool_test.cpp
    memory/column_buffer_test.cpp
    map/id_types_test.cpp
    features/stereo_rig_test.cpp
    features/binocular_measurement_test.cpp
    map/map_point_storage_test.cpp
    map/frame_test.cpp
    map/observation_storage_test.cpp
    events/event_system_test.cpp
    events/wake_signal_test.cpp
    events/worker_thread_test.cpp
    slam/slam_test.cpp
    tracking/pose_estimator_test.cpp
    tracking/tracker_test.cpp
    mapping/map_initializer_test.cpp
    mapping/mapper_test.cpp
    loop_closing/loop_closer_test.cpp
)

target_include_directories(voyahslam_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(voyahslam_tests
    PRIVATE voyahslam
    PRIVATE GTest::gtest_main
    PRIVATE ${OpenCV_LIBS}
)

include(GoogleTest)
gtest_discover_tests(voyahslam_tests)
