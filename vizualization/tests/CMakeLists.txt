add_executable(voyahviz_tests
    color_image_test.cpp
    keypoint_drawing_test.cpp
    scene_state_test.cpp
    map_viewer_test.cpp
)

target_link_libraries(voyahviz_tests
    PRIVATE voyahviz
    PRIVATE GTest::gtest_main
)

gtest_discover_tests(voyahviz_tests)
