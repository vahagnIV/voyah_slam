add_executable(hello_voyahslam
    hello_voyahslam.cpp
)

target_link_libraries(hello_voyahslam
    PRIVATE voyahslam
)

find_package(yaml-cpp REQUIRED)

add_subdirectory(dataset_reader)
add_subdirectory(full_slam)
