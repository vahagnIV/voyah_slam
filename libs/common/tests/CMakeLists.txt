add_executable(voyah_common_tests
    algebra/algebra_test.cpp
    features/binocular_measurement_test.cpp
    features/image_features_test.cpp
    features/monocular_measurement_test.cpp
    features/stereo_rig_test.cpp
    geometry/camera_test.cpp
    geometry/distortion_models/distortion_model_test.cpp
    geometry/distortion_models/kannala_brandt_distortion_model_test.cpp
    geometry/rigid_transformation_test.cpp
    map/covisibility_index_test.cpp
    map/frame_test.cpp
    map/id_types_test.cpp
    map/map_point_storage_test.cpp
    map/observation_storage_test.cpp
    memory/column_buffer_test.cpp
    memory/object_pool_test.cpp
)

# voyah_test_support (utils_test.h, used by algebra_test) links voyahslam
# until Task 6 reorganises test support.
target_link_libraries(voyah_common_tests
    PRIVATE voyah_common
    PRIVATE voyah_test_support
    PRIVATE GTest::gtest_main
    PRIVATE ${OpenCV_LIBS}
)

gtest_discover_tests(voyah_common_tests)
