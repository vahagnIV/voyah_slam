# Unit tests of voyah_slam.
voyah_add_test_binary(voyah_slam_tests
    LABEL unit
    SOURCES
        test_voyahslam.cpp
        features/projection_matcher_test.cpp
        global_ba/global_ba_test.cpp
        global_ba/global_bundle_adjuster_test.cpp
        loop_closing/loop_closer_test.cpp
        mapping/map_follower_test.cpp
        mapping/map_initializer_test.cpp
        mapping/mapper_test.cpp
        mapping/paired_inliers_test.cpp
        tracking/local_map_test.cpp
        tracking/atlas_replica_test.cpp
        tracking/map_replica_test.cpp
        tracking/relocalization_test.cpp
        thread_affinity_test.cpp
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

target_compile_definitions(voyah_slam_tests PRIVATE
    VOYAHSLAM_TEST_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    VOYAHSLAM_TEST_VERSION_MINOR=${PROJECT_VERSION_MINOR}
)

# Build check (DESIGN "Map Follower"): an app that reads the map through a
# MapFollower, built at -O0, must not instantiate a function the libraries also
# instantiate, or the linker may give the libraries the app's unoptimized copy.
# Two -O0 objects of build_checks/follower_consumer.cpp, with and without the
# follower, are compared with nm (GNU-style toolchains only).
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND CMAKE_NM)
    find_program(VOYAH_CXXFILT NAMES c++filt llvm-cxxfilt)
    foreach(variant baseline follower)
        add_library(voyah_consumer_${variant} OBJECT build_checks/follower_consumer.cpp)
        target_link_libraries(voyah_consumer_${variant} PRIVATE voyah_slam)
        # Last on the command line, so it wins over the configuration's -O.
        target_compile_options(voyah_consumer_${variant} PRIVATE -O0)
        set_property(GLOBAL APPEND PROPERTY VOYAH_TEST_BINARIES voyah_consumer_${variant})
    endforeach()
    target_compile_definitions(voyah_consumer_baseline PRIVATE VOYAH_CONSUMER_USES_FOLLOWER=0)
    target_compile_definitions(voyah_consumer_follower PRIVATE VOYAH_CONSUMER_USES_FOLLOWER=1)
    add_test(NAME BuildCheck.MapFollowerAddsNoLibraryInstantiations
        COMMAND ${CMAKE_COMMAND}
            -DNM=${CMAKE_NM}
            -DCXXFILT=${VOYAH_CXXFILT}
            -DBASELINE=$<TARGET_OBJECTS:voyah_consumer_baseline>
            -DCONSUMER=$<TARGET_OBJECTS:voyah_consumer_follower>
            "-DLIBRARIES=$<TARGET_FILE:voyah_slam>|$<TARGET_FILE:voyah_algs>|$<TARGET_FILE:voyah_events>|$<TARGET_FILE:voyah_common>"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/build_checks/check_consumer_instantiations.cmake
    )
    set_tests_properties(BuildCheck.MapFollowerAddsNoLibraryInstantiations PROPERTIES LABELS unit)
endif()
