voyah_add_test_binary(voyahviz_tests
    LABEL unit
    SOURCES
        camera_frustum_test.cpp
        color_image_test.cpp
        keypoint_drawing_test.cpp
        scene_state_test.cpp
        map_viewer_test.cpp
    LIBS
        voyahviz
)
