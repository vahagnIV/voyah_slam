# voyah_test_support: shared test helpers (utils_test.*, support/*.h). STATIC
# because utils_test.cpp holds helper definitions, not test cases.
#
# It links voyah_common only, so a test binary sees no layer above its own.
# support/synthetic_sequence.h also includes an algs header
# (features/stereo_matcher.h): only binaries linking voyah_algs can include it.
add_library(voyah_test_support STATIC utils_test.cpp)
target_include_directories(voyah_test_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(voyah_test_support
    PUBLIC voyah_common
    PUBLIC GTest::gtest
)
