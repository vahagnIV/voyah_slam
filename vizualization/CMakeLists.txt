# voyahviz: map viewer and image windows (development tool). Links only
# voyah_common and voyah_events: it consumes the libraries' events and never
# sees the algorithms or the SLAM components. Uses OpenCV and Pangolin
# internally; the public API exposes only voyah_common / voyah_events types.
find_package(Pangolin 0.9 REQUIRED)

add_library(voyahviz STATIC
    src/color_image.cpp
    src/keypoint_drawing.cpp
    src/image_window.cpp
    src/scene_state.cpp
    src/map_viewer.cpp
    src/pangolin_renderer.cpp
)

voyah_optimize_target(voyahviz)

target_include_directories(voyahviz
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(voyahviz
    PUBLIC voyah_common
    PUBLIC voyah_events
    # OpenCV is an implementation detail: never in public headers, linked PRIVATE.
    PRIVATE opencv_core
    PRIVATE opencv_imgproc
    PRIVATE opencv_highgui
    # Pangolin (window + GL) is private too: only src/pangolin_renderer.* use it.
    PRIVATE pango_display
)

add_subdirectory(tests)
