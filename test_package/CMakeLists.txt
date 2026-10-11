cmake_minimum_required(VERSION 3.15)
project(test_package CXX)

find_package(voyahslam REQUIRED CONFIG)

add_executable(test_package main.cpp)
target_compile_features(test_package PRIVATE cxx_std_20)
target_link_libraries(test_package PRIVATE voyahslam::voyahslam)
