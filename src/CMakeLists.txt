add_library(voyahslam
    voyahslam.cpp
    algebra/algebra.cpp
    geometry/geometry.cpp
    io/read_image.cpp
    logging/logger.cpp
    features/orb_feature_extractor.cpp
    features/sift_feature_extractor.cpp
    features/distortion_models/dummy_distortion_model.cpp
    features/distortion_models/kannala_brandt_distortion_model.cpp
    features/orb/image_pyramid.cpp
    features/orb/extract_node.cpp
    features/orb/quad_tree_distributor.cpp
)

target_include_directories(voyahslam
    PUBLIC ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(voyahslam
    PUBLIC Eigen3::Eigen
    PUBLIC spdlog::spdlog
    PRIVATE stb::stb
    PRIVATE opencv::opencv
)
