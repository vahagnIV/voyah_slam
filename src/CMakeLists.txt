find_package(g2o REQUIRED)

add_library(voyahslam
    voyahslam.cpp
    estimators/essential_estimator.cpp
    estimators/homography_estimator.cpp
    geometry/geometry.cpp
    geometry/two_view_decomposition.cpp
    geometry/two_view_reconstruction.cpp
    io/read_image.cpp
    features/feature_grid.cpp
    features/orb_feature_extractor.cpp
    features/sift_feature_extractor.cpp
    features/orb/image_pyramid.cpp
    features/orb/extract_node.cpp
    features/orb/quad_tree_distributor.cpp
    tracking/tracker.cpp
    mapping/mapper.cpp
    mapping/map_initializer.cpp
    mapping/keyframe_decision.cpp
    map/point_statistics.cpp
    loop_closing/loop_closer.cpp
    optimization/bundle_adjustment.cpp
    optimization/pose_optimization.cpp
    slam/slam.cpp
)

target_include_directories(voyahslam
    PUBLIC ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(voyahslam
    PUBLIC voyah_common
    PUBLIC voyah_events
    PUBLIC Eigen3::Eigen
    PUBLIC spdlog::spdlog
    PUBLIC ${VOYAHSLAM_CONCURRENTQUEUE_TARGET}
    PRIVATE stb::stb
    PRIVATE opencv_core
    PRIVATE opencv_imgproc
    PRIVATE opencv_features
    PRIVATE g2o::core g2o::stuff g2o::types_sba g2o::types_slam3d g2o::solver_eigen
)
