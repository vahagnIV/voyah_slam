# voyah_algs: stateless algorithms over common types (estimators, two-view
# geometry, image filters, feature extraction and matching, triangulation,
# optimization). stb and g2o are implementation details (PRIVATE). No OpenCV:
# only the tests and benchmarks link it, as a reference.
find_package(g2o REQUIRED)

add_library(voyah_algs
    src/bow/vocabulary_io.cpp
    src/estimators/essential_estimator.cpp
    src/estimators/homography_estimator.cpp
    src/features/fast/fast.cpp
    src/features/feature_grid.cpp
    src/features/orb/extract_node.cpp
    src/features/orb/image_pyramid.cpp
    src/features/orb/quad_tree_distributor.cpp
    src/features/orb_feature_extractor.cpp
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

# FAST SIMD row kernels (src/features/fast/; DESIGN "FAST corner detector").
# x86-64: SSE2 is the baseline; the AVX2 kernel is built with -mavx2 in its own
# file and chosen at run time (__builtin_cpu_supports), so the library still
# runs on CPUs without AVX2. arm64: NEON is the baseline. Other targets: scalar.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    target_sources(voyah_algs PRIVATE src/features/fast/fast_sse2.cpp)
    target_compile_definitions(voyah_algs PRIVATE VOYAHSLAM_FAST_SSE2)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_sources(voyah_algs PRIVATE src/features/fast/fast_avx2.cpp)
        set_source_files_properties(src/features/fast/fast_avx2.cpp
            PROPERTIES COMPILE_OPTIONS "-mavx2")
        target_compile_definitions(voyah_algs PRIVATE VOYAHSLAM_FAST_AVX2)
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    target_sources(voyah_algs PRIVATE src/features/fast/fast_neon.cpp)
    target_compile_definitions(voyah_algs PRIVATE VOYAHSLAM_FAST_NEON)
endif()

target_include_directories(voyah_algs
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# stb is header-only: BUILD_INTERFACE keeps it out of the exported link
# interface. g2o stays PRIVATE; for a static voyah_algs CMake still exports it
# as $<LINK_ONLY:g2o::...> (link-time only, no compile flags), which is why the
# package config calls find_dependency(g2o) for a static build.
target_link_libraries(voyah_algs
    PUBLIC voyah_common
    PRIVATE $<BUILD_INTERFACE:stb::stb>
    PRIVATE g2o::core g2o::stuff g2o::types_sba g2o::types_sim3 g2o::types_slam3d g2o::solver_eigen
)

if(VOYAHSLAM_BUILD_TESTS)
    add_subdirectory(tests)
    add_subdirectory(benchmarks)
endif()
