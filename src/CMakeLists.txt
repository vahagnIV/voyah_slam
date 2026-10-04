add_library(voyahslam
    voyahslam.cpp
    algebra/algebra.cpp
    estimators/essential_estimator.cpp
    estimators/homography_estimator.cpp
    geometry/geometry.cpp
    geometry/two_view_decomposition.cpp
    io/read_image.cpp
    logging/logger.cpp
    features/orb_feature_extractor.cpp
    features/sift_feature_extractor.cpp
    features/distortion_models/dummy_distortion_model.cpp
    features/distortion_models/kannala_brandt_distortion_model.cpp
    features/orb/image_pyramid.cpp
    features/orb/extract_node.cpp
    features/orb/quad_tree_distributor.cpp
    tracking/tracker.cpp
    tracking/pose_estimator.cpp
    mapping/mapper.cpp
    mapping/map_initializer.cpp
    loop_closing/loop_closer.cpp
    events/worker_thread.cpp
    slam/slam.cpp
)

target_include_directories(voyahslam
    PUBLIC ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(voyahslam
    PUBLIC Eigen3::Eigen
    PUBLIC spdlog::spdlog
    PUBLIC moodycamel::concurrentqueue
    PRIVATE stb::stb
    PRIVATE opencv_core
    PRIVATE opencv_imgproc
    PRIVATE opencv_features
)
