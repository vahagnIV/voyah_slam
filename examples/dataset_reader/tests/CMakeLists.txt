voyah_add_test_binary(dataset_reader_tests
    LABEL unit
    SOURCES
        euroc_dataset_reader_test.cpp
        playback_clock_test.cpp
    LIBS
        voyahslam_dataset_reader
        stb::stb
)
