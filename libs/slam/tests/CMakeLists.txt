# Unit tests of voyah_slam.
voyah_add_test_binary(voyah_slam_tests
    LABEL unit
    SOURCES
        test_voyahslam.cpp
        features/projection_matcher_test.cpp
        loop_closing/loop_closer_test.cpp
        mapping/map_initializer_test.cpp
        mapping/mapper_test.cpp
        mapping/paired_inliers_test.cpp
        tracking/local_map_test.cpp
        tracking/map_replica_test.cpp
        tracking/tracker_test.cpp
    LIBS
        voyah_slam
        voyah_test_support
)

# This directory, for tracking/tracker_test_fixture.h; the private sources
# root, for paired_inliers_test (mapping/paired_inliers.h).
target_include_directories(voyah_slam_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

# Integration tests: the whole pipeline (SlamTest) and long threaded
# Tracker/Mapper sequences. Run by check_all, not by check.
voyah_add_test_binary(voyah_integration_tests
    LABEL integration
    SOURCES
        integration/slam_test.cpp
        integration/tracker_test.cpp
    LIBS
        voyah_slam
        voyah_test_support
)

# This directory, for tracking/tracker_test_fixture.h.
target_include_directories(voyah_integration_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
