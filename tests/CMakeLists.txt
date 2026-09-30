add_executable(voyahslam_tests
    test_voyahslam.cpp
)

target_link_libraries(voyahslam_tests
    PRIVATE voyahslam
    PRIVATE GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(voyahslam_tests)
