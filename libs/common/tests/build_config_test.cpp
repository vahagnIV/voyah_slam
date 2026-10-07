#include <gtest/gtest.h>

// Guards the optimized test build (VOYAHSLAM_OPTIMIZE_TESTS): -O2 must never
// come with NDEBUG, so asserts stay on in Debug. VOYAH_EXPECT_ASSERTS is set
// by CMake: 1 for Debug builds, 0 otherwise.
#ifndef VOYAH_EXPECT_ASSERTS
#error "VOYAH_EXPECT_ASSERTS must be defined by the build"
#endif

TEST(BuildConfigTest, AssertsEnabled) {
#if VOYAH_EXPECT_ASSERTS
#ifdef NDEBUG
    FAIL() << "Debug build must not define NDEBUG (asserts would be disabled)";
#else
    SUCCEED();
#endif
#else
    GTEST_SKIP() << "Non-Debug build";
#endif
}
