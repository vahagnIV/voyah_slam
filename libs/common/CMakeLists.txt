# voyah_common: the bottom layer. Data types, ids, storage, camera models and
# the extractor class declarations (their implementations live higher up).
add_library(voyah_common
    src/algebra/algebra.cpp
    src/geometry/camera.cpp
    src/geometry/distortion_models/brown_conrady_distortion_model.cpp
    src/geometry/distortion_models/dummy_distortion_model.cpp
    src/geometry/distortion_models/kannala_brandt_distortion_model.cpp
    src/logging/logger.cpp
    src/map/covisibility_index.cpp
    src/memory/object_pool.cpp
)

voyah_optimize_target(voyah_common)

target_include_directories(voyah_common
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Public headers are C++20: consumers of the installed package get it too.
target_compile_features(voyah_common PUBLIC cxx_std_20)

target_link_libraries(voyah_common
    PUBLIC Eigen3::Eigen
    PUBLIC spdlog::spdlog
)

if(VOYAHSLAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
