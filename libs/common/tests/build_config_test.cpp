#include <gtest/gtest.h>

// Guards the optimized test build (VOYAHSLAM_OPTIMIZE_TESTS): -O2 must never
// come with NDEBUG, so asserts stay on in Debug.
TEST(BuildConfigTest, AssertsEnabled) {
#ifdef NDEBUG
    GTEST_SKIP() << "Release build";
#else
    SUCCEED();
#endif
}
