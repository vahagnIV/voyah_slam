# voyah_algs: stateless algorithms over common types (estimators, two-view
# geometry, image filters, feature extraction and matching, triangulation,
# optimization). OpenCV, stb and g2o are implementation details (PRIVATE).
find_package(g2o REQUIRED)

add_library(voyah_algs STATIC
    src/estimators/essential_estimator.cpp
    src/estimators/homography_estimator.cpp
    src/features/feature_grid.cpp
    src/features/orb/extract_node.cpp
    src/features/orb/image_pyramid.cpp
    src/features/orb/quad_tree_distributor.cpp
    src/features/orb_feature_extractor.cpp
    src/features/sift_feature_extractor.cpp
    src/geometry/geometry.cpp
    src/geometry/two_view_decomposition.cpp
    src/geometry/two_view_reconstruction.cpp
    src/io/read_image.cpp
    src/map/point_statistics.cpp
    src/mapping/keyframe_decision.cpp
    src/optimization/bundle_adjustment.cpp
    src/optimization/pose_optimization.cpp
)

target_include_directories(voyah_algs
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(voyah_algs
    PUBLIC voyah_common
    PRIVATE stb::stb
    PRIVATE opencv_core
    PRIVATE opencv_imgproc
    PRIVATE opencv_features
    PRIVATE g2o::core g2o::stuff g2o::types_sba g2o::types_slam3d g2o::solver_eigen
)

add_subdirectory(tests)
