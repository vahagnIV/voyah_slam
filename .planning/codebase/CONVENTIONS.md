---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
# Coding Conventions

**Analysis Date:** 2026-10-12

## Naming Patterns

**Files:**
- Header files: `.h` extension
- Source files: `.cpp` extension  
- Test files: `*_test.cpp` (e.g., `tracker_test.cpp`)
- Include guard: `VOYAHSLAM_<PATH>_H` in UPPER_CASE_WITH_UNDERSCORES (e.g., `VOYAHSLAM_TRACKING_TRACKER_H`)

**Functions and Classes:**
- Class/struct/enum names: `PascalCase` (e.g., `AsyncObserver`, `TrackerState`, `Tracker<TFeatureExtractor>`)
- Function names: `PascalCase` (e.g., `ProcessPending()`, `GetNextEvent()`)
- Member functions: `PascalCase` (e.g., `SetWakeSignal()`, `TryCreate()`)
- Getter functions: no `Get` prefix unless accessing a complex object (e.g., `state()`, `pending_priority()`, `GetListener()`)

**Variables:**
- Local variables: `snake_case` (e.g., `captured_frames`, `T_cw`)
- Member variables (private): `snake_case_` with trailing underscore (e.g., `queue_`, `wake_signal_`, `pending_priority_`)
- Static/module-level: `snake_case` or `kCamelCase` if constant (e.g., `kNil`, `kAlign`)
- Template parameters: `T` prefix in PascalCase (e.g., `TFeatureExtractor`, `TMeasurement`, `TTracker`)

**Parameters:**
- Input parameters: `snake_case`
- Output parameters (non-const references): `out_` prefix (e.g., `out_event`, `out_frames`) — documented in comments
- Configuration structs: `Options` suffix (e.g., `TrackerOptions`, `MapperOptions`, `TrackingOptions`)

**Enumerations:**
- Enum class names: `PascalCase` (e.g., `TrackerState`, `EventType`, `TrackingStage`)
- Enum values: `PascalCase` (e.g., `WaitingForMap`, `Tracking`, `Lost`)

**Type Aliases:**
- Typedef prefix: `T` in PascalCase (e.g., `TPoint3DF`, `TMatrix33F`, `TPoints2DF`)
- Measurement types: `T[Mono|Binocular|Shared]Measurement<FE>` pattern

## Code Style

**Formatting:**
- Tool: `.clang-format` (LLVM-based)
- Column limit: 120 characters
- Indentation: 4 spaces (no tabs)
- Namespace indentation: full indentation (NamespaceIndentation: All)
- Brace style: custom LLVM variant (AfterClass: false, AfterFunction: false, AfterNamespace: false)
- Constructor initializer list: one per line after colon (BreakConstructorInitializers: AfterColon)
- Empty functions: split onto multiple lines (SplitEmptyFunction: true)
- Template declarations: always break to new line (AlwaysBreakTemplateDeclarations: Yes)
- Newline at EOF: always insert (InsertNewlineAtEOF: true)
- Max empty lines to keep: 2

**Linting:**
- No explicit linter configuration (clang-format only)
- Compile warnings: treated seriously; code compiles cleanly with GCC 13

## Import Organization

**Order (in headers and source files):**
1. System headers with `<angle>` brackets (priority 1)
2. Project headers with `"quotes"` (priority 2)
3. Other headers (priority 3)

Within each group, alphabetically ordered.

**Example from `async_observer.h`:**

```cpp
#include <concurrentqueue.h>
#include <atomic>
#include <cstdint>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/wake_signal.h>
```

**Path Aliases:**
- Headers use absolute includes: `#include <voyahslam/path/to/header.h>` (public) or `#include <voyahslam/...>` for internal
- Include roots defined per library: `libs/<lib>/include/` contains the public namespace

## Error Handling

**Patterns:**
- Return `nullptr` for nullable pointer returns (e.g., `TryCreate()`, `try_dequeue()`)
- Return `false` for success/failure operations (e.g., `TryPop(...)`)
- Return `std::optional<T>` for operations that may not produce a value (e.g., `TrackFrame()`)
- Use assertions (`assert()`) for invariant violations in debug builds
- No exceptions thrown in library code (C++ core guidelines)
- Logging errors/warnings via spdlog (see Logging below)

**Assertion Style:**
- Asserts remain on in optimized Debug builds (`VOYAHSLAM_OPTIMIZE_TESTS=ON`, `-O2 -g` but no `NDEBUG`)
- Tests verify assertions with `VOYAH_EXPECT_ASSERTS` macro

**Example from `async_observer.h`:**

```cpp
[[nodiscard]] bool TryPop(Event*& out_event) {
    if (!queue_.try_dequeue(out_event)) {
        return false;
    }
    if (out_event->priority) {
        pending_priority_.fetch_sub(1, std::memory_order_acq_rel);
    }
    return true;
}
```

## Logging

**Framework:** spdlog (header-only, included as `<spdlog/spdlog.h>`)

**Setup:**
- Central initialization in `Logger::Initialize()` (`logging/logger.h`)
- Per-class loggers created with `Logger::CreateLogger(name)` macro

**Macro Usage:**

```cpp
// In class definition (header or source):
CLASS_LOGGER(ClassName);

// In implementation:
auto logger = log_;  // or directly: log_->info("message")
```

**Severity Levels:**
- `info()` - normal operation milestones (e.g., map initialization, pool growth)
- `warn()` - recoverable issues (e.g., event pool overflow, lost tracking)
- `error()` - recoverable errors (e.g., invalid pose, culled points)
- `debug()` - detailed trace information

**Examples from codebase:**
- `ObjectPool::Grow()` logs pool growth with old/new capacity
- Tracker logs state transitions
- Mapper logs map initialization and merging

## Comments

**When to Comment:**
- Complex synchronization logic (lock-free operations, memory ordering)
- Non-obvious design decisions (explained in DESIGN.md sections)
- Deviations from standard patterns
- Subtle bugs or edge cases
- Parameter meaning and constraints

**Style:**
- Line comments: `// explanation`
- Block comments for multi-paragraph explanations
- Comments stay close to the code they explain

**Example from `tracker.h` (lines 48-78, TrackingOptions struct):**

```cpp
// Frame-to-map tracking thresholds (ORB-SLAM3 TrackWithMotionModel /
// TrackLocalMap).
struct TrackingOptions {
    double window_mono_px = 15.0;       // last-frame search window, monocular
    double window_stereo_px = 7.0;      // last-frame search window, binocular
    size_t min_motion_matches = 20;     // below: retry with the window x2; still below: fail
    // ...
};
```

**JSDoc/Documentation Comments:**
- Not used; design and constraints documented in DESIGN.md and header comments
- Complex algorithms referenced to research papers or ORB-SLAM3

## Function Design

**Size:**
- Prefer small functions (~20-50 lines) that do one thing
- Complex tracking/mapping logic split into helper functions

**Parameters:**
- Input by value for small types (primitives, `size_t`, `map::MapPointId`)
- Input by const reference for large types (matrices, measurements)
- Rvalue references for ownership transfer (measurements in `Track()`)
- Output via non-const reference with `out_` prefix (rare in library code)
- Move constructors for thread handoff (WorkerThread, AsyncObserver)

**Return Values:**
- By value for owned data (measurements, transactions)
- By const reference for data owned elsewhere (keyframes from replica)
- By pointer for nullable returns (`nullptr`)
- `std::optional<T>` for "may not exist" values

**Const Correctness:**
- Member functions that don't modify state: `const`
- Const-correctness enforced in tests

**Example from `tracker.h` (lines 217-220):**

```cpp
[[nodiscard]] const AtlasReplica<TFeatureExtractor>& atlas() const { return atlas_; }
[[nodiscard]] const MapReplica<TFeatureExtractor>& map_replica() const { 
    return atlas_.active_or_empty(); 
}
```

**Thread Safety Annotations:**
- `[[nodiscard]]` on important return values (esp. error indicators)
- Atomics with explicit memory ordering (`std::memory_order_acquire`, `acq_rel`, `release`)
- Documented thread affinity (Tracker: "Tracker-thread state, for tests...")

## Module Design

**Exports:**
- Public headers in `libs/<lib>/include/voyahslam/` (no OpenCV in voyah_* libraries)
- Private implementation in `libs/<lib>/src/`
- Include root: `#include <voyahslam/path/to/header.h>` for public, implementation details not included

**Namespace Organization:**
- Nested: `voyahslam::tracking`, `voyahslam::mapping`, `voyahslam::events`
- Anonymous namespaces in `.cpp` files for static helpers and test details

**Barrel Files:**
- Some modules export all event types in one `events/events.h`
- Measurement types in `features/measurement.h`

**Instantiation:**
- Template classes instantiated with `INSTANTIATE_CLASS` macro in `.cpp`
- Currently: `ORB` feature extractor only (SIFT removed 2026-10-11)

**Example from tracker.h:**

```cpp
template <typename TFeatureExtractor>
class Tracker : public events::EventNode {
    // ...
};
// Instantiation in tracker.cpp:
// template class Tracker<features::OrbFeatureExtractor>;
```

**Const Pointers to Immutable Data:**
- Shared-owned via `std::shared_ptr<const T>` (measurements, distortion models)
- Raw pointers for compile-time-constant config (StereoRig, cameras)

## Synchronization Patterns

**Memory Ordering:**
- `memory_order_acquire` on reads of published state
- `memory_order_acq_rel` on CAS operations and counters
- `memory_order_release` on publication of state
- `memory_order_relaxed` for stats counters

**Lock-Free Queues:**
- `moodycamel::ConcurrentQueue<Event*>` for async events
- No locks on map data; per-thread copies with replica rule

**Atomic Types:**
- `std::atomic<uint32_t>` for counters and wakeup signals
- `std::atomic<uint64_t>` for growing counter (pool head in free-list)

## Design Constraints (from DESIGN.md)

**No Locks on Map Data:**
- All map mutations in Mapper only
- Other threads track read-only replicas
- Communication only via events on lock-free queues

**Pointer Rule:**
- Raw pointers: only to data that never changes (config, immutable)
- Shared ownership: `shared_ptr<const T>` for data that may be short-lived
- No pointers to reused buffers

**ID Non-Reuse:**
- FrameId unique per run
- MapPointId unique per map
- IDs start at 1, only increase
- Enables stale-ID detection via IsValid() check

---

*Convention analysis: 2026-10-12*
