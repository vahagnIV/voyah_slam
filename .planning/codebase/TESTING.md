---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
# Testing Patterns

**Analysis Date:** 2026-10-12

## Test Framework

**Runner:**
- Google Test (gtest) - header-only in CMake as `GTest::gtest_main`
- Config: No separate config file; integrated into CMakeLists.txt via `cmake/VoyahTests.cmake`
- Version: `find_package(GTest REQUIRED)` in root CMakeLists.txt

**Build:**
- Enable tests with CMake option: `-o with_tests=True` in Conan install
- Or set `VOYAHSLAM_BUILD_TESTS=ON` in CMake
- Tests only built when explicitly requested (off by default for package builds)

**Run Commands:**

```bash

# Fast loop: 746 unit tests (~10s + build)

cmake --build cmake-build-debug/build/Debug --target check

# Every test: 773 tests, unit + integration (~30s + build)

cmake --build cmake-build-debug/build/Debug --target check_all

# CTest directly (discovered at build time)

ctest -L unit -C Debug -j8 --timeout 120 --output-on-failure
ctest -C Debug -j8 --timeout 300 --output-on-failure
```

**Optimization:**
- Default: `VOYAHSLAM_OPTIMIZE_TESTS=ON` compiles tests with `-O2 -g` even in Debug builds
- Keeps asserts on (no `NDEBUG` flag)
- Disabling: `-DVOYAHSLAM_OPTIMIZE_TESTS=OFF` to step through code in debugger

## Test File Organization

**Location:**
- Co-located with source: `libs/<lib>/tests/` parallel to `libs/<lib>/src/`
- Example: `libs/common/tests/memory/`, `libs/slam/tests/tracking/`, `libs/algs/tests/geometry/`

**Naming:**
- Test files: `*_test.cpp` (e.g., `tracker_test.cpp`, `event_system_test.cpp`)
- Test fixtures/shared: `*_test_fixture.h` (e.g., `tracker_test_fixture.h`)
- Support/helpers: `libs/test_support/` (synthetic sequences, test utilities)

**Structure:**

```
libs/
  common/
    tests/
      CMakeLists.txt
      memory/
        object_pool_test.cpp
        recyclable_test.cpp
      algebra/
      ...
  slam/
    tests/
      tracking/
        tracker_test.cpp
        tracker_test_fixture.h
      mapping/
        ...
```

## Test Structure

**Suite Organization - Basic Pattern:**

```cpp
#include <gtest/gtest.h>
#include "support/synthetic_sequence.h"

namespace voyahslam::tracking {
namespace {

// Unnamed namespace for test details (helpers, test data)

TEST(EventSystemTest, EventTypeTag) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();

    EXPECT_EQ(event->type, EventType::MapTransaction);
    event->Recycle();
}

} // namespace
} // namespace voyahslam::tracking
```

**Fixture Pattern - Shared Across Unit and Integration Tests:**

```cpp
// In tracker_test_fixture.h:
class TrackerTest : public ::testing::Test {
protected:
    explicit TrackerTest(mapping::MapperOptions mapper_options = {},
                         TrackerOptions tracker_options = {},
                         const std::shared_ptr<const TVocabulary>& vocabulary = nullptr)
        : mapper_(std::move(mapper_options), vocabulary),
          tracker_(std::move(tracker_options), vocabulary) {
        mapper_.AddObserver(tracker_.inbox());
        tracker_.AddObserver(mapper_.inbox());
        injector_.ConnectAsync(tracker_.inbox());
        tracker_.AddObserver(capture_);
    }

    void TearDown() override {
        tracker_.Stop();
        Drain();  // empty all inboxes
    }

    void Drain() {
        bool processed = true;
        while (processed) {
            processed = false;
            while (tracker_.ProcessPending()) processed = true;
            while (mapper_.ProcessPending()) processed = true;
        }
    }

    // Helper: feed a measurement synchronously, drain both components
    template <typename TMeasurement>
    std::vector<Captured> Feed(TMeasurement measurement) {
        tracker_.Track(std::move(measurement));
        Drain();
        EXPECT_TRUE(tracker_.map_replica().CheckConsistency());
        return TakeCaptured();
    }

    TMapper mapper_;
    TTracker tracker_;
    events::Observable injector_;
    events::AsyncObserver capture_;
    events::Listener listener_{capture_.queue()};
};
```

**Derived Fixtures - Specialization:**

```cpp
class TrackerMonoAccuracyTest : public TrackerTest {
protected:
    TrackerMonoAccuracyTest() : TrackerTest(MonoAccuracyMapperOptions()) {}

    static mapping::MapperOptions MonoAccuracyMapperOptions() {
        mapping::MapperOptions options;
        options.initializer.reconstruction.min_parallax_deg = 4.0;
        return options;
    }
};

TEST_F(TrackerMonoAccuracyTest, InitializesAndTracks) {
    // Uses MonoAccuracyMapperOptions by default
}
```

**Patterns:**

1. **Setup in Constructor:**
   - Avoid `SetUp()` when possible; use constructor initialization
   - Fixture members initialized in-place (mapper, tracker, observer)
   - `TearDown()` ensures cleanup (stops workers, drains queues)

2. **Teardown in TearDown():**
   ```cpp
   void TearDown() override {
       tracker_.Stop();
       Drain();  // empty all queues before destruction
   }
   ```

3. **Synchronous Test Loop:**
   - No threads in tests; call `ProcessPending()` to drive state
   - Mapper and Tracker connected via inboxes
   - Injector pushes hand-built events into Tracker's inbox

4. **Event Capture Helper:**
   ```cpp
   std::vector<Captured> TakeCaptured() {
       std::vector<Captured> captured;
       while (events::Event* event = listener_.GetNextEvent()) {
           Captured c{...};
           if (event->type == events::EventType::FrameTracked) {
               // Extract fields from FrameTracked
           }
           captured.push_back(c);
       }
       return captured;
   }
   ```

## Mocking

**Framework:** No explicit mocking library (Google Mock not found in dependencies)

**Approach:**
- **Minimal mocking:** Real Mapper and Tracker driven synchronously
- **Test doubles:** Synthetic data (measurements, sequences) instead of mocks
- **Injection:** Hand-built events via Observable injector

**What NOT to Mock:**
- Core components (Mapper, Tracker, LoopCloser)
- Event system (publish/subscribe)
- Map data structures (replicas, point storage)

**What to Substitute:**
- **Measurements:** Synthetic stereo sequences with ground-truth poses
- **Events:** Hand-constructed (e.g., MapTransaction with specific points/frames)
- **Configuration:** Options structs with test values (e.g., MapperOptions, TrackerOptions)

**Example - Hand-Built Event (from `event_system_test.cpp`):**

```cpp
TEST(EventSystemTest, ObservableAsyncFanOut) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);

    Observable observable;
    AsyncObserver observer1;
    AsyncObserver observer2;

    observable.ConnectAsync(observer1);
    observable.ConnectAsync(observer2);

    auto* event = pool.Create();  // Hand-built
    observable.Notify(event);

    {
        auto listener1 = observer1.GetListener();
        auto* got1 = listener1.GetNextEvent();
        ASSERT_NE(got1, nullptr);
        EXPECT_EQ(got1->type, EventType::MapTransaction);
    }
    // listener1 destructor recycles event; listener2 pops it
}
```

## Fixtures and Factories

**Test Data:**

1. **Synthetic Sequences (`support/synthetic_sequence.h`):**
   ```cpp
   // Generate stereo frames with ground-truth poses and 600 scene points
   test::SyntheticStereoSequence StereoSequence(double step = 0.05, double yaw_step_deg = 0.2) {
       test::SyntheticStereoSequenceOptions options;
       options.trajectory.num_points = 600;
       options.trajectory.step = step;
       options.trajectory.yaw_step_deg = yaw_step_deg;
       return test::MakeSyntheticStereoSequence(options);
   }
   ```

2. **Test Measurements (`support/test_measurements.h`):**
   ```cpp
   auto frame = test::MakeSyntheticStereoFrame();
   auto measurement = test::MakeMonocular(200, 0.0);  // 200 features, timestamp 0.0
   ```

3. **Custom Vocabulary (`tracker_test_fixture.h`):**
   ```cpp
   // k = 2, L = 1: two words of weight 1 under root. With levels_up = 4,
   // every FeatureVector files features under root (all pairs compared).
   inline std::shared_ptr<const TVocabulary> MakeRootVocabulary() {
       using TDescriptors = typename Extractor::TDescriptors;
       TDescriptors descriptors = TDescriptors::Zero(Extractor::DescriptorSize, 3);
       descriptors(0, 2) = 0xFF;
       return std::make_shared<const TVocabulary>(
           TVocabulary::FromTree(2, 1, {0, 0, 0}, std::move(descriptors), {0.0, 1.0, 1.0}));
   }
   ```

**Location:**
- `libs/test_support/` - shared across all test suites
- `libs/test_support/support/` - synthetic sequences, test measurements, trajectory error metrics

**Reuse:**
- Fixtures shared between unit and integration tests via `*_test_fixture.h`
- Synthesis factories (SyntheticStereoSequence, etc.) reused in multiple test suites

## Coverage

**Requirements:** No explicit coverage target enforced (not detected in CMake configuration)

**View Coverage:** Not configured in CMakeLists.txt or documentation

## Test Types

**Unit Tests (Label: `unit`):**
- Scope: Single component in isolation (e.g., ObjectPool, Event recycling, Tracker processing one measurement)
- Approach: Synchronous ProcessPending() without threads
- Examples:
  - `event_system_test.cpp` - Event pooling, listener lifecycle, fanout
  - `build_config_test.cpp` - Assert flag verification
  - `object_pool_test.cpp` - Pool growth, recycling, free-list management

**Integration Tests (Label: `integration`):**
- Scope: Multiple components interacting (e.g., Tracker + Mapper, full tracking pipeline, map initialization)
- Approach: Synchronous with both components running via ProcessPending() calls
- Examples:
  - `tracker_test.cpp::TrackerMonoAccuracyTest` - Mono initialization and tracking accuracy
  - `tracker_test.cpp::TrackerTest::TracksAndPublishesFrameTracked` - Frame tracking pipeline
  - Loop closing and map merging tests

**End-to-End / Vocabulary Tests (Label: `vocabulary`):**
- Scope: Full system with real ORBvoc.txt vocabulary
- Approach: Synchronous like integration tests, but require `$VOYAHSLAM_ORBVOC` env var
- Run in `check_all` only; skipped in `check`

## Common Patterns

**Async Testing - Synchronous Simulation:**

```cpp
// Instead of running workers asynchronously, test synchronously:
template <typename TMeasurement>
std::vector<Captured> Feed(TMeasurement measurement) {
    tracker_.Track(std::move(measurement));  // enqueue
    Drain();  // processAll in both components until quiescent
    EXPECT_TRUE(tracker_.map_replica().CheckConsistency());
    return TakeCaptured();
}
```

**Error Testing - Assertion Verification:**

```cpp
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
```

**Floating-Point Comparison:**

```cpp
// Use Eigen's isApprox() for matrices:
EXPECT_TRUE(binocular.depths().isApprox(expected.depths));

// Or EXPECT_DOUBLE_EQ for scalars:
EXPECT_DOUBLE_EQ(captured[0].timestamp, sequence.frames[1].measurement.timestamp());

// For pose matrices:
EXPECT_TRUE(replica.keyframe(ids[1])->T_cw.ToMatrix34().isApprox(
    geometry::RigidTransformation::Identity().ToMatrix34()));
```

**State Verification:**

```cpp
EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
EXPECT_EQ(tracker_.current_map_id(), map::MapId{1});
EXPECT_GE(replica.num_points(), 100u);
```

**Event Validation:**

```cpp
auto* event = listener_.GetNextEvent();
ASSERT_NE(event, nullptr);
ASSERT_EQ(event->type, events::EventType::FrameTracked);
const auto& tracked = *static_cast<TFrameTracked*>(event);
EXPECT_EQ(tracked.reference_frame_id.value_or(map::FrameId{0}), keyframe);
```

**Parametrized Tests (if used):**
- Not found in codebase; would use `TEST_P(Fixture, Name)` with `INSTANTIATE_TEST_SUITE_P`
- Consider for testing multiple feature extractors, camera models, etc.

## Test Discovery and Execution

**Discovery:**
- CMake's `gtest_discover_tests()` runs at build time
- Tests labeled with `LABEL` property (unit, integration, vocabulary)
- Located in `<test_binary>` executable (e.g., `libs/common/tests/CMakeFiles/`)

**Execution:**
- `check` target: `-L unit` (746 tests, ~10s)
- `check_all` target: no label filter (773 tests, ~30s, includes integration + vocabulary)
- CTest timeout: 120s per unit test, 300s per integration test
- Parallel: `-j8` (8 workers)

---

*Testing analysis: 2026-10-12*
