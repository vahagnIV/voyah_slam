# voyah_common: the bottom layer. Data types, ids, storage, camera models and
# the extractor class declarations (their implementations live higher up).
add_library(voyah_common STATIC
    src/algebra/algebra.cpp
    src/geometry/camera.cpp
    src/geometry/distortion_models/dummy_distortion_model.cpp
    src/geometry/distortion_models/kannala_brandt_distortion_model.cpp
    src/logging/logger.cpp
    src/map/covisibility_index.cpp
)

target_include_directories(voyah_common
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(voyah_common
    PUBLIC Eigen3::Eigen
    PUBLIC spdlog::spdlog
)

add_subdirectory(tests)
