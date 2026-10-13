# A discarded [[nodiscard]] events::Subscription disconnects at once, so ignoring
# one is an error in the apps (01-RESEARCH Pitfall 6).
set(VOYAH_APP_NODISCARD_OPTIONS $<$<CXX_COMPILER_ID:GNU,Clang>:-Werror=unused-result>)

add_executable(monocular_full_slam
    monocular_full_slam.cpp
)
target_compile_options(monocular_full_slam PRIVATE ${VOYAH_APP_NODISCARD_OPTIONS})
target_link_libraries(monocular_full_slam
    PRIVATE voyahslam_dataset_reader
    PRIVATE voyahviz
)

add_executable(binocular_full_slam
    binocular_full_slam.cpp
)
target_compile_options(binocular_full_slam PRIVATE ${VOYAH_APP_NODISCARD_OPTIONS})
target_link_libraries(binocular_full_slam
    PRIVATE voyahslam_dataset_reader
    PRIVATE voyahviz
)
