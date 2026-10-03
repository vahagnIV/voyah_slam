cmake_minimum_required(VERSION 3.22)
project(voyah_slam)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Conan dependencies
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(stb REQUIRED)
# OpenCV comes from a system install (built from source, /usr/local), not Conan.
find_package(OpenCV 5 REQUIRED COMPONENTS core imgproc features highgui)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(concurrentqueue REQUIRED)

# Main library
add_subdirectory(src)

# Tests
enable_testing()
include(GoogleTest)
add_subdirectory(tests)

# Visualization (development tool, links OpenCV)
add_subdirectory(vizualization)

# Examples
add_subdirectory(examples)
