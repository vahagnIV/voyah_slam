voyah_add_test_binary(voyah_algs_tests
    LABEL unit
    SOURCES
        bow/frame_database_test.cpp
        bow/vocabulary_io_test.cpp
        bow/vocabulary_test.cpp
        estimators/essential_estimator_test.cpp
        estimators/homography_estimator_test.cpp
        estimators/ransac_test.cpp
        features/fast_test.cpp
        features/feature_extractor_levels_test.cpp
        features/bow_matcher_test.cpp
        features/feature_grid_test.cpp
        features/fuse_matcher_test.cpp
        features/initialization_matcher_test.cpp
        features/keypoint_matches_test.cpp
        features/stereo_matcher_test.cpp
        geometry/geometry_test.cpp
        geometry/pnp_test.cpp
        geometry/sim3_solver_test.cpp
        geometry/two_view_decomposition_test.cpp
        geometry/two_view_reconstruction_test.cpp
        images/bilinear_interpolation_tests.cpp
        images/filter_tests.cpp
        images/gaussian_blur_tests.cpp
        images/resizer_tests.cpp
        map/point_statistics_test.cpp
        mapping/keyframe_decision_test.cpp
        mapping/map_quality_test.cpp
        mapping/triangulation_test.cpp
        optimization/bundle_adjustment_test.cpp
        optimization/essential_graph_test.cpp
        optimization/pose_optimization_test.cpp
        optimization/sim3_optimization_test.cpp
        optimization/projection_test.cpp
        support/synthetic_sequence_test.cpp
    LIBS
        voyah_algs
        voyah_test_support
        # Test reference only (FastTest compares with cv::FAST).
        opencv_core
        opencv_features
)

# Private sources root, for tests of private headers (optimization/projection.h).
target_include_directories(voyah_algs_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

# Fixture directory (bow/data), found at test time.
target_compile_definitions(voyah_algs_tests
    PRIVATE VOYAH_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/bow/data"
)

# Equivalence with the original DBoW2 on the real ORBvoc.txt ($VOYAHSLAM_ORBVOC; skips
# when unset). Its own binary and label, so `check` leaves it out.
voyah_add_test_binary(voyah_vocabulary_tests
    LABEL vocabulary
    SOURCES
        bow/vocabulary_equivalence_test.cpp
    LIBS
        voyah_algs
)
target_compile_definitions(voyah_vocabulary_tests
    PRIVATE VOYAH_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/bow/data"
)

# Real-data feature tests on TUM-VI corridor1 ($VOYAHSLAM_CORRIDOR1, else
# ~/Downloads/dataset-corridor1_1024_16; skip when missing): the ORB extractor
# golden regression and the FAST port against live cv::FAST. OpenCV is linked
# here as a test reference only, never by voyah_algs.
voyah_add_test_binary(voyah_algs_dataset_tests
    LABEL integration
    SOURCES
        features/fast_corridor1_test.cpp
        features/orb_extractor_golden_test.cpp
    LIBS
        voyah_algs
        opencv_core
        opencv_features
)
target_compile_definitions(voyah_algs_dataset_tests
    PRIVATE VOYAH_FEATURES_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/features/data"
)
