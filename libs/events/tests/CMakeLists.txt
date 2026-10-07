add_executable(voyah_events_tests
    events/event_system_test.cpp
    events/wake_signal_test.cpp
    events/worker_thread_test.cpp
)

target_link_libraries(voyah_events_tests
    PRIVATE voyah_events
    PRIVATE voyah_test_support
    PRIVATE GTest::gtest_main
    PRIVATE ${OpenCV_LIBS}
)

gtest_discover_tests(voyah_events_tests)
