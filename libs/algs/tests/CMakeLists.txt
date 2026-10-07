voyah_add_test_binary(voyah_algs_tests
    LABEL unit
    SOURCES
        estimators/essential_estimator_test.cpp
        estimators/homography_estimator_test.cpp
        estimators/ransac_test.cpp
        features/feature_extractor_levels_test.cpp
        features/feature_grid_test.cpp
        features/fuse_matcher_test.cpp
        features/initialization_matcher_test.cpp
        features/keypoint_matches_test.cpp
        features/stereo_matcher_test.cpp
        geometry/geometry_test.cpp
        geometry/two_view_decomposition_test.cpp
        geometry/two_view_reconstruction_test.cpp
        images/bilinear_interpolation_tests.cpp
        images/filter_tests.cpp
        images/gaussian_blur_tests.cpp
        images/resizer_tests.cpp
        map/point_statistics_test.cpp
        mapping/keyframe_decision_test.cpp
        mapping/triangulation_test.cpp
        optimization/bundle_adjustment_test.cpp
        optimization/pose_optimization_test.cpp
        optimization/projection_test.cpp
        support/synthetic_sequence_test.cpp
    LIBS
        voyah_algs
        voyah_test_support
)

# Private sources root, for tests of private headers (optimization/projection.h).
target_include_directories(voyah_algs_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
