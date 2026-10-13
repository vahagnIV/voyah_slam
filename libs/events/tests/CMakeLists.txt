voyah_add_test_binary(voyah_events_tests
    LABEL unit
    SOURCES
        events/event_lifetime_test.cpp
        events/event_system_test.cpp
        events/wake_signal_test.cpp
        events/worker_thread_test.cpp
    LIBS
        voyah_events
)
