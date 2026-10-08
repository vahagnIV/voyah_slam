# voyah_algs: stateless algorithms over common types (estimators, two-view
# geometry, image filters, feature extraction and matching, triangulation,
# optimization). OpenCV, stb and g2o are implementation details (PRIVATE).
find_package(g2o REQUIRED)

add_library(voyah_algs STATIC
    src/bow/vocabulary_io.cpp
    src/estimators/essential_estimator.cpp
    src/estimators/homography_estimator.cpp
    src/features/feature_grid.cpp
    src/features/orb/extract_node.cpp
    src/features/orb/image_pyramid.cpp
    src/features/orb/quad_tree_distributor.cpp
    src/features/orb_feature_extractor.cpp
    src/features/sift_feature_extractor.cpp
    src/geometry/geometry.cpp
    src/geometry/pnp.cpp
    src/geometry/sim3_solver.cpp
    src/geometry/two_view_decomposition.cpp
    src/geometry/two_view_reconstruction.cpp
    src/io/read_image.cpp
    src/map/point_statistics.cpp
    src/mapping/keyframe_decision.cpp
    src/mapping/map_quality.cpp
    src/optimization/bundle_adjustment.cpp
    src/optimization/essential_graph.cpp
    src/optimization/pose_optimization.cpp
    src/optimization/sim3_optimization.cpp
)

voyah_optimize_target(voyah_algs)

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
    PRIVATE g2o::core g2o::stuff g2o::types_sba g2o::types_sim3 g2o::types_slam3d g2o::solver_eigen
)

add_subdirectory(tests)
