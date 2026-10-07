# voyah_events: the event system (observable/listener, event types, worker thread).
add_library(voyah_events STATIC
    src/events/worker_thread.cpp
)

voyah_optimize_target(voyah_events)

target_include_directories(voyah_events
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(voyah_events
    PUBLIC voyah_common
    PUBLIC ${VOYAHSLAM_CONCURRENTQUEUE_TARGET}
)

add_subdirectory(tests)
