add_library(voyahslam_dataset_reader STATIC
    src/euroc_dataset_reader.cpp
    src/playback_clock.cpp
)

target_include_directories(voyahslam_dataset_reader
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(voyahslam_dataset_reader
    PUBLIC voyahslam
    PRIVATE yaml-cpp::yaml-cpp
)

if(VOYAHSLAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
