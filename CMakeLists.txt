cmake_minimum_required(VERSION 3.22)
project(voyah_slam)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Conan dependencies
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(stb REQUIRED)
# OpenCV comes from a system install (built from source, /usr/local), not Conan.
# Only tests, benchmarks and the visualization tool link it; the voyah_*
# libraries do not use OpenCV.
find_package(OpenCV 5 REQUIRED COMPONENTS core imgproc features highgui)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(concurrentqueue REQUIRED)
# Conan Center's concurrentqueue recipe revisions differ in the target name.
if(TARGET moodycamel::concurrentqueue)
    set(VOYAHSLAM_CONCURRENTQUEUE_TARGET moodycamel::concurrentqueue)
else()
    set(VOYAHSLAM_CONCURRENTQUEUE_TARGET concurrentqueue::concurrentqueue)
endif()

option(VOYAHSLAM_OPTIMIZE_TESTS
    "Compile voyah libraries and tests with -O2 in Debug" ON)

enable_testing()
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(VoyahTests)

# Libraries (each with its own include root)
add_subdirectory(libs)

# Visualization (development tool, links OpenCV)
add_subdirectory(vizualization)

# Tools
add_subdirectory(tools/convert_vocabulary)

# Examples
add_subdirectory(examples)

# check (unit tests) and check_all (every test); after every test binary.
voyah_add_check_targets()
