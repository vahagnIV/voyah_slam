cmake_minimum_required(VERSION 3.22)
project(voyah_slam VERSION 0.3.0)

if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options (DESIGN §7 "Build options and packaging"). Both OFF by default,
# so a package build (conan create) needs only the libraries' dependencies; the
# developer trees turn both ON through the Conan options with_tests /
# with_examples (README "Build and test").
option(VOYAHSLAM_BUILD_TESTS
    "Build the tests, the benchmarks and voyah_test_support (needs GTest, OpenCV)" OFF)
option(VOYAHSLAM_BUILD_EXAMPLES
    "Build the apps, voyahviz and the tools (needs OpenCV, Pangolin, yaml-cpp)" OFF)
option(VOYAHSLAM_OPTIMIZE_TESTS
    "Compile voyah libraries and tests with -O2 in Debug" ON)

# Library dependencies (Conan).
find_package(Eigen3 REQUIRED)
find_package(stb REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(concurrentqueue REQUIRED)
# Conan Center's concurrentqueue recipe revisions differ in the target name.
if(TARGET moodycamel::concurrentqueue)
    set(VOYAHSLAM_CONCURRENTQUEUE_TARGET moodycamel::concurrentqueue)
else()
    set(VOYAHSLAM_CONCURRENTQUEUE_TARGET concurrentqueue::concurrentqueue)
endif()

if(VOYAHSLAM_BUILD_TESTS)
    find_package(GTest REQUIRED)
endif()
if(VOYAHSLAM_BUILD_TESTS OR VOYAHSLAM_BUILD_EXAMPLES)
    # OpenCV comes from a system install (built from source, /usr/local), not
    # Conan. Only tests, benchmarks and the visualization tool link it; the
    # voyah_* libraries do not use OpenCV.
    find_package(OpenCV 5 REQUIRED COMPONENTS core imgproc features highgui)
endif()

include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(VoyahTests)
if(VOYAHSLAM_BUILD_TESTS)
    enable_testing()
endif()

# Libraries (each with its own include root), their install and export rules.
add_subdirectory(libs)

if(VOYAHSLAM_BUILD_EXAMPLES)
    # Visualization (development tool, links OpenCV)
    add_subdirectory(vizualization)

    # Tools
    add_subdirectory(tools/convert_vocabulary)

    # Examples
    add_subdirectory(examples)
endif()

if(VOYAHSLAM_BUILD_TESTS)
    # check (unit tests) and check_all (every test); after every test binary.
    voyah_add_check_targets()
endif()
