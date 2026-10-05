cmake_minimum_required(VERSION 3.15)
project(test_package CXX)

find_package(g2o REQUIRED CONFIG)

add_executable(test_package test_package.cpp)
target_compile_features(test_package PRIVATE cxx_std_20)
target_link_libraries(test_package PRIVATE
  g2o::solver_eigen g2o::types_sba g2o::types_slam3d g2o::core g2o::stuff)
