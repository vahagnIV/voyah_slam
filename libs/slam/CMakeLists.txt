# voyah_slam: the SLAM pipeline (tracking, mapping, loop closing, the Slam
# facade). Builds on common, events and algs.
add_library(voyah_slam STATIC
    src/voyahslam.cpp
    src/tracking/tracker.cpp
    src/mapping/mapper.cpp
    src/mapping/map_initializer.cpp
    src/loop_closing/loop_closer.cpp
    src/slam/slam.cpp
)

voyah_optimize_target(voyah_slam)

target_include_directories(voyah_slam
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(voyah_slam
    PUBLIC voyah_common
    PUBLIC voyah_events
    PUBLIC voyah_algs
)

add_subdirectory(tests)
