add_subdirectory(common)
add_subdirectory(events)
add_subdirectory(algs)
add_subdirectory(slam)

# Shared test helpers (voyah_test_support), used by the libraries' tests.
add_subdirectory(test_support)

# voyahslam: umbrella target linking all four libraries, so apps and outside
# users keep linking a single name.
add_library(voyahslam INTERFACE)
target_link_libraries(voyahslam
    INTERFACE voyah_common
    INTERFACE voyah_events
    INTERFACE voyah_algs
    INTERFACE voyah_slam
)
