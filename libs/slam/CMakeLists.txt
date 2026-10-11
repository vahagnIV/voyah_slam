# voyah_slam: the SLAM pipeline (tracking, mapping, loop closing, the Slam
# facade). Builds on common, events and algs.
add_library(voyah_slam
    src/voyahslam.cpp
    src/tracking/tracker.cpp
    src/tracking/relocalizer.cpp
    src/mapping/mapper.cpp
    src/mapping/mapper_quality.cpp
    src/mapping/mapper_loop.cpp
    src/mapping/mapper_global_ba.cpp
    src/mapping/mapper_merge.cpp
    src/mapping/map_initializer.cpp
    src/loop_closing/common_region.cpp
    src/loop_closing/loop_closer.cpp
    src/global_ba/global_ba.cpp
    src/global_ba/global_bundle_adjuster.cpp
    src/slam/slam.cpp
)

voyah_optimize_target(voyah_slam)

target_include_directories(voyah_slam
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(voyah_slam
    PUBLIC voyah_common
    PUBLIC voyah_events
    PUBLIC voyah_algs
)

if(VOYAHSLAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
