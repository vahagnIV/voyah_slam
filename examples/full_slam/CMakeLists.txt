add_executable(full_slam
    main.cpp
)

target_link_libraries(full_slam
    PRIVATE voyahslam
)
