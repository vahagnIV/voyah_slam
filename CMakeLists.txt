cmake_minimum_required(VERSION 3.22)
project(voyah_slam)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Conan dependencies
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(stb REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(concurrentqueue REQUIRED)

# Main library
add_subdirectory(src)

# Tests
enable_testing()
add_subdirectory(tests)

# Examples
add_subdirectory(examples)
