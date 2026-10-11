# voyah_events: the event system (observable/listener, event types, worker thread).
add_library(voyah_events
    src/events/worker_thread.cpp
)

voyah_optimize_target(voyah_events)

target_include_directories(voyah_events
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(voyah_events
    PUBLIC voyah_common
    PUBLIC ${VOYAHSLAM_CONCURRENTQUEUE_TARGET}
)

if(VOYAHSLAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
