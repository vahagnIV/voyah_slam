add_executable(dataset_reader_tests
    euroc_dataset_reader_test.cpp
    playback_clock_test.cpp
)

target_link_libraries(dataset_reader_tests
    PRIVATE voyahslam_dataset_reader
    PRIVATE GTest::gtest_main
    PRIVATE stb::stb
)

gtest_discover_tests(dataset_reader_tests)
