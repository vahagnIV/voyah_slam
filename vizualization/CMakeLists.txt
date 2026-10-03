# voyahviz: visualization for voyahslam (development tool). Uses OpenCV
# internally; the public API only exposes voyahslam types.
add_library(voyahviz STATIC
    src/color_image.cpp
    src/keypoint_drawing.cpp
    src/image_window.cpp
    src/scene_state.cpp
)

target_include_directories(voyahviz
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(voyahviz
    PUBLIC voyahslam
    # OpenCV is an implementation detail: never in public headers, linked PRIVATE.
    PRIVATE opencv_core
    PRIVATE opencv_imgproc
    PRIVATE opencv_highgui
)

add_subdirectory(tests)
