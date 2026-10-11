# Benchmarks: built with `all` but not part of `check` / `check_all`; run by
# hand from a Release build. OpenCV is linked here as the reference only.

# FAST port vs cv::FAST on corridor1 (DESIGN "FAST corner detector").
add_executable(fast_benchmark fast_benchmark.cpp)
target_link_libraries(fast_benchmark
    PRIVATE voyah_algs
    PRIVATE opencv_core
    PRIVATE opencv_features
)
