add_library(voyahslam
    voyahslam.cpp
    tracking/tracker.cpp
    mapping/mapper.cpp
    mapping/map_initializer.cpp
    loop_closing/loop_closer.cpp
    slam/slam.cpp
)

target_include_directories(voyahslam
    PUBLIC ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(voyahslam
    PUBLIC voyah_common
    PUBLIC voyah_events
    PUBLIC voyah_algs
    PUBLIC Eigen3::Eigen
    PUBLIC spdlog::spdlog
    PUBLIC ${VOYAHSLAM_CONCURRENTQUEUE_TARGET}
)
