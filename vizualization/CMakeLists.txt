# voyahviz: visualization for voyahslam (development tool, may depend on OpenCV).
add_library(voyahviz STATIC
    src/image_conversion.cpp
    src/keypoint_drawing.cpp
    src/image_window.cpp
)

target_include_directories(voyahviz
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(voyahviz
    PUBLIC voyahslam
    PUBLIC opencv::opencv
)

add_subdirectory(tests)
