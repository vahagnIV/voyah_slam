add_executable(hello_voyahslam
    hello_voyahslam.cpp
)

target_link_libraries(hello_voyahslam
    PRIVATE voyahslam
)
