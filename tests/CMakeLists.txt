# Shared test helpers (tests/utils_test.*, tests/support/*.h). STATIC because
# utils_test.cpp holds helper definitions, not test cases.
add_library(voyah_test_support STATIC utils_test.cpp)
target_include_directories(voyah_test_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(voyah_test_support
    PUBLIC voyahslam
    PUBLIC GTest::gtest
)

add_executable(voyahslam_tests
    test_voyahslam.cpp
    geometry/two_view_scene_test.cpp
    slam/slam_test.cpp
    tracking/tracker_test.cpp
    tracking/map_replica_test.cpp
    tracking/local_map_test.cpp
    support/synthetic_sequence_test.cpp
    mapping/map_initializer_test.cpp
    mapping/paired_inliers_test.cpp
    mapping/mapper_test.cpp
    loop_closing/loop_closer_test.cpp
)

target_include_directories(voyahslam_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(voyahslam_tests
    PRIVATE voyah_test_support
    PRIVATE voyahslam
    PRIVATE GTest::gtest_main
    PRIVATE ${OpenCV_LIBS}
)

include(GoogleTest)
gtest_discover_tests(voyahslam_tests)
