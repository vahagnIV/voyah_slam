add_executable(voyahslam_tests
    test_voyahslam.cpp
    utils_test.cpp
    algebra/algebra_test.cpp
    geometry/geometry_test.cpp
    images/bilinear_interpolation_tests.cpp
    images/filter_tests.cpp
    images/gaussian_blur_tests.cpp
    images/resizer_tests.cpp
    features/camera_frame_test.cpp
    features/image_features_test.cpp
    features/keypoint_matches_test.cpp
    features/distortion_models/distortion_model_test.cpp
    features/distortion_models/kannala_brandt_distortion_model_test.cpp
    memory/object_pool_test.cpp
    map/id_types_test.cpp
)

target_include_directories(voyahslam_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(voyahslam_tests
    PRIVATE voyahslam
    PRIVATE GTest::gtest_main
    PRIVATE opencv::opencv
)

include(GoogleTest)
gtest_discover_tests(voyahslam_tests)
