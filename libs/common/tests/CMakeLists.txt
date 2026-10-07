# OpenCV: kannala_brandt_distortion_model_test uses calib3d as a reference.
voyah_add_test_binary(voyah_common_tests
    LABEL unit
    SOURCES
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
        support/two_view_scene_test.cpp
    LIBS
        voyah_common
        voyah_test_support
        ${OpenCV_LIBS}
)
