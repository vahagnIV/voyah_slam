add_executable(voyahviz_tests
    image_conversion_test.cpp
    keypoint_drawing_test.cpp
)

target_link_libraries(voyahviz_tests
    PRIVATE voyahviz
    PRIVATE GTest::gtest_main
)

gtest_discover_tests(voyahviz_tests)
