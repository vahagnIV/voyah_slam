# Core SLAM Data Structures Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the core data layer for an event-driven SLAM system: memory pool, map storage, frames, observations, camera rig, and pub-sub event delivery.

**Architecture:** Structure-of-Arrays storage for map points and observations, template-based mono/stereo switching, and a lock-free event system using moodycamel::ConcurrentQueue with pooled ref-counted events. Observable/AsyncObserver/Listener are non-template classes operating on `Event*`. No virtual functions on the hot path.

**Tech Stack:** C++20, Eigen3, moodycamel::ConcurrentQueue (new dep), GTest, Conan 2

**Spec:** `docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md`

## Global Constraints

- C++20 standard (`-std=c++20`)
- Include guards: `#ifndef VOYAHSLAM_<PATH>_H` / `#define` / `#endif` (no `#pragma once`)
- Public headers use `#include <voyahslam/...>` with angle brackets
- Template classes are header-only; non-template classes have `.cpp` in `src/` mirroring `include/`
- `[[nodiscard]]` on const accessor methods
- Move semantics throughout; protected data members in base classes that need subclass access
- `INSTANTIATE_CLASS` macro from `include/voyahslam/features/instantiation.h` for templates on `TFeatureExtractor`
- New source files added to `src/CMakeLists.txt`; new test files added to `tests/CMakeLists.txt`
- Existing 78 tests must continue passing after every task

## Review Focus

1. **MapPointStorage::Remove then Add reuse** — removing a point and adding a new one must reuse the freed slot; the new point's data must not contain stale values from the removed point
2. **Listener move semantics** — a moved-from Listener must not double-recycle `current_`; only the destination Listener owns the event
3. **Observable::Notify with zero async observers** — must not call `SetRefCount(0)` which would make `Recycle()` immediately return to pool before sync handlers finish; sync-only path must skip ref counting entirely
4. **ObservationStorage stereo Add on Monocular type** — the 5-arg `Add()` overload must not compile when `Type == ObservationType::Monocular`
5. **ObjectPool exhaustion** — calling `Create()` on a full pool must assert, not silently return nullptr and crash later on dereference

---

### Task 1: Memory Module — SyncPolicy, Recyclable, ObjectPool

Port from `slamlab2/include/slamlab/memory/` with namespace `slamlab::memory` → `voyahslam::memory`. Add `SetRefCount(int n)` to `Recyclable`.

**Files:**
- Create: `include/voyahslam/memory/sync_policy.h`
- Create: `include/voyahslam/memory/recyclable.h`
- Create: `include/voyahslam/memory/object_pool.h`
- Create: `tests/memory/object_pool_test.cpp`
- Modify: `tests/CMakeLists.txt` — add test source file

**Interfaces:**
- Produces:
  - `voyahslam::memory::ThreadSafePolicy` — struct with `std::mutex mtx` and nested `Lock` (RAII `lock_guard`)
  - `voyahslam::memory::NoSyncPolicy` — struct with no-op nested `Lock`
  - `voyahslam::memory::Recyclable<T, SyncPolicy>` — CRTP mixin; `void AddRef()`, `void Recycle()`, `void SetRefCount(int n)`. Member `std::atomic_int ref_count_{1}`. Friend `ObjectPool<T, SyncPolicy>` accesses `object_pool_`.
  - `voyahslam::memory::ObjectPool<T, SyncPolicy>` — `explicit ObjectPool(size_t max_count = 1'000'000)`, `template<typename... Args> T* Create(Args&&...)`, `void Recycle(T* obj)`

- [ ] **Step 1: Create `include/voyahslam/memory/sync_policy.h`**

Port from slamlab2's `sync_policy.h`. Change namespace to `voyahslam::memory`. Guard: `VOYAHSLAM_MEMORY_SYNC_POLICY_H`.

- [ ] **Step 2: Create `include/voyahslam/memory/recyclable.h`**

Port from slamlab2's `recyclable.h`. Change namespace to `voyahslam::memory`. Guard: `VOYAHSLAM_MEMORY_RECYCLABLE_H`. Add this new method alongside `AddRef()` and `Recycle()`:

```cpp
void SetRefCount(int n) {
    ref_count_.store(n, std::memory_order_relaxed);
}
```

- [ ] **Step 3: Create `include/voyahslam/memory/object_pool.h`**

Port from slamlab2's `object_pool.h`. Change namespace to `voyahslam::memory`. Guard: `VOYAHSLAM_MEMORY_OBJECT_POOL_H`. Include `<voyahslam/memory/sync_policy.h>`. No other changes — the slamlab2 implementation is correct as-is.

- [ ] **Step 4: Write tests in `tests/memory/object_pool_test.cpp`**

Test fixture with a `TestObject` inheriting `Recyclable<TestObject, SyncPolicy>`. Tests:

```cpp
// Test: CreateAndRecycle — create object, verify fields, recycle, create again at same address
// Test: RefCounting — SetRefCount(3), Recycle() three times, verify object returns to pool on third call
// Test: SetRefCount — set to 2, AddRef() (now 3), Recycle() three times, verify return on third
// Test: ThreadSafety — 4 threads, each create/recycle 100 objects from ThreadSafePolicy pool
// Test: PoolExhaustion — pool of size 1, create one object, assert death on second Create()
```

- [ ] **Step 5: Add `tests/memory/object_pool_test.cpp` to `tests/CMakeLists.txt`**

- [ ] **Step 6: Build and run tests**

Run: `cmake --build cmake-build-debug/build/Debug && ctest --test-dir cmake-build-debug/build/Debug --output-on-failure`
Expected: All new memory tests pass, all 78 existing tests still pass.

- [ ] **Step 7: Commit**

```bash
git add include/voyahslam/memory/ tests/memory/ tests/CMakeLists.txt
git commit -m "Add memory module: ObjectPool, Recyclable, SyncPolicy

Port from slamlab2 with namespace voyahslam::memory. Add SetRefCount()
to Recyclable for batch ref-count initialization."
```

---

### Task 2: Strong ID Types — MapPointId, FrameId

**Files:**
- Create: `include/voyahslam/map/map_point_id.h`
- Create: `include/voyahslam/map/frame_id.h`
- Create: `tests/map/id_types_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `voyahslam::map::MapPointId` — struct with `uint32_t value`, defaulted `==` and `<=>` operators
  - `voyahslam::map::FrameId` — same structure

- [ ] **Step 1: Create `include/voyahslam/map/map_point_id.h`**

Guard: `VOYAHSLAM_MAP_MAP_POINT_ID_H`. Namespace `voyahslam::map`.

```cpp
struct MapPointId {
    uint32_t value;
    bool operator==(const MapPointId&) const = default;
    auto operator<=>(const MapPointId&) const = default;
};
```

- [ ] **Step 2: Create `include/voyahslam/map/frame_id.h`**

Same pattern. Guard: `VOYAHSLAM_MAP_FRAME_ID_H`.

- [ ] **Step 3: Write tests in `tests/map/id_types_test.cpp`**

```cpp
// Test: MapPointIdEquality — MapPointId{5} == MapPointId{5}, MapPointId{5} != MapPointId{6}
// Test: MapPointIdOrdering — MapPointId{3} < MapPointId{7}
// Test: FrameIdEquality — same pattern
// Test: TypeSafety — static_assert(!std::is_convertible_v<MapPointId, FrameId>) or runtime distinct type check
```

- [ ] **Step 4: Add to `tests/CMakeLists.txt`, build and run**

- [ ] **Step 5: Commit**

```bash
git add include/voyahslam/map/ tests/map/ tests/CMakeLists.txt
git commit -m "Add strong ID types: MapPointId, FrameId"
```

---

### Task 3: StereoRig

**Files:**
- Create: `include/voyahslam/features/stereo_rig.h`
- Create: `tests/features/stereo_rig_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `voyahslam::geometry::TMatrix33F`, `TVector3DF`, `voyahslam::precision_t`, `distortion_models::DistortionModel*`
- Produces:
  - `voyahslam::features::StereoRig` — struct with fields: `K_left` (TMatrix33F), `K_right` (TMatrix33F), `R_rl` (TMatrix33F), `t_rl` (TVector3DF), `baseline` (precision_t), `distortion_left` (const DistortionModel*, default nullptr), `distortion_right` (const DistortionModel*, default nullptr)

- [ ] **Step 1: Create `include/voyahslam/features/stereo_rig.h`**

Guard: `VOYAHSLAM_FEATURES_STEREO_RIG_H`. Namespace `voyahslam::features`. Plain struct with aggregate initialization. Include `<voyahslam/defs.h>` and `<voyahslam/features/distortion_models/distortion_model.h>`.

- [ ] **Step 2: Write test in `tests/features/stereo_rig_test.cpp`**

```cpp
// Test: Construction — create StereoRig with identity K_left, K_right, R_rl = I, t_rl = (0.12, 0, 0), baseline = 0.12
//       Verify all fields match. Verify distortion pointers are nullptr by default.
```

- [ ] **Step 3: Add to `tests/CMakeLists.txt`, build and run**

- [ ] **Step 4: Commit**

```bash
git add include/voyahslam/features/stereo_rig.h tests/features/stereo_rig_test.cpp tests/CMakeLists.txt
git commit -m "Add StereoRig: immutable camera rig configuration"
```

---

### Task 4: BinocularCameraFrame

**Files:**
- Create: `include/voyahslam/features/binocular_camera_frame.h`
- Create: `tests/features/binocular_camera_frame_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CameraFrame<TScalar, Length>`, `ImageFeatures<TScalar, Length>`, `StereoRig`
- Produces:
  - `voyahslam::features::BinocularCameraFrame<TScalar, Length>` — constructor takes `ImageFeatures&&` left, `ImageFeatures&&` right, `const StereoRig&`. Accessors: `left()`, `right()` (const CameraFrame refs), `depths()` (const TRowVectorF&), `right_indices()` (const VectorXi&), `depth(int idx)` (precision_t)

- [ ] **Step 1: Create `include/voyahslam/features/binocular_camera_frame.h`**

Guard: `VOYAHSLAM_FEATURES_BINOCULAR_CAMERA_FRAME_H`. Template `<typename TScalar, unsigned int Length>`. Header-only. Include `<voyahslam/features/camera_frame.h>` and `<voyahslam/features/stereo_rig.h>`.

Constructor: build left/right CameraFrames from the moved ImageFeatures using rig's K_left/K_right and distortion models. Initialize `depths_` and `right_indices_` to appropriate sizes (depths to -1.0 or similar sentinel for unmatched, right_indices to -1). Stereo matching and depth computation are left as stubs for now — fill depths/right_indices with defaults. The matching algorithm is a Tracker concern, not this class's.

- [ ] **Step 2: Write test in `tests/features/binocular_camera_frame_test.cpp`**

```cpp
// Test: Construction — create StereoRig (identity K, baseline 0.12),
//       create left/right ImageFeatures with 5 random keypoints each,
//       construct BinocularCameraFrame, verify left().K() == K_left, right().K() == K_right,
//       depths().size() == left feature count, right_indices().size() == left feature count
// Test: Accessors — verify depth(0) returns depths()(0), left().points() matches input
```

- [ ] **Step 3: Add to `tests/CMakeLists.txt`, build and run**

- [ ] **Step 4: Commit**

```bash
git add include/voyahslam/features/binocular_camera_frame.h tests/features/binocular_camera_frame_test.cpp tests/CMakeLists.txt
git commit -m "Add BinocularCameraFrame: stereo pair with depths and right indices"
```

---

### Task 5: MapPointStorage (SoA)

**Files:**
- Create: `include/voyahslam/map/map_point_storage.h`
- Create: `tests/map/map_point_storage_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `MapPointId` (Task 2), feature extractor protocol (`TDescriptorScalarType`, `DescriptorSize`, `TDescriptor`, `TDescriptors`)
- Produces:
  - `voyahslam::map::MapPointStorage<TFeatureExtractor>` — `MapPointId Add(const TPoint3DF& pos, const TDescriptor& desc, const TPoint3DF& normal)`, `void Remove(MapPointId)`, `auto Position(MapPointId)`, `auto Descriptor(MapPointId)`, `auto Normal(MapPointId)`, `bool IsValid(MapPointId) const`, `uint32_t size() const`, `const TPoints3DF& positions() const`, `const auto& descriptors() const`

- [ ] **Step 1: Create `include/voyahslam/map/map_point_storage.h`**

Guard: `VOYAHSLAM_MAP_MAP_POINT_STORAGE_H`. Header-only template. Namespace `voyahslam::map`. Include `<voyahslam/defs.h>`, `<voyahslam/map/map_point_id.h>`.

Growth strategy for `Add()`: if free_list_ is non-empty, pop and reuse. Otherwise `conservativeResize` all Eigen matrices by doubling capacity (start at 256). On reuse, overwrite the slot's data and set `valid_[id] = true`, increment `size_`. `Remove()`: set `valid_[id] = false`, push to free_list_, decrement `size_`.

`Position(id)`, `Descriptor(id)`, `Normal(id)` return `positions_.col(id.value)` etc. (Eigen column expression).

- [ ] **Step 2: Write tests in `tests/map/map_point_storage_test.cpp`**

Use `OrbFeatureExtractor` as the template argument.

```cpp
// Test: AddAndAccess — add 3 points, verify Position/Descriptor/Normal for each, verify size() == 3
// Test: Remove — add 2, remove first, verify IsValid(first) == false, size() == 1, second still accessible
// Test: ReuseSlot — add, remove, add again — new ID == old ID (reused from free list), new data correct
// Test: BulkAccess — add 10 points, verify positions().cols() >= 10, iterate checking valid slots
// Test: RemoveInvalidId — remove already-removed ID is a no-op (or assert) — define behavior
```

- [ ] **Step 3: Add to `tests/CMakeLists.txt`, build and run**

- [ ] **Step 4: Commit**

```bash
git add include/voyahslam/map/map_point_storage.h tests/map/map_point_storage_test.cpp tests/CMakeLists.txt
git commit -m "Add MapPointStorage: SoA map point storage with dense IDs and free-list"
```

---

### Task 6: Frame

**Files:**
- Create: `include/voyahslam/map/frame.h`
- Create: `tests/map/frame_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `FrameId` (Task 2), `CameraFrame`, `BinocularCameraFrame` (Task 4), `TPose`
- Produces:
  - `voyahslam::map::Frame<TFeatureExtractor, TCameraFrameType>` — constructor `Frame(FrameId, const TPose&, TCameraFrameType&&, double timestamp)`. Accessors: `id()`, `pose()`, `mutable_pose()`, `camera_frame()`, `timestamp()`.
  - `voyahslam::map::MonoFrame<FE>` — alias for `Frame<FE, CameraFrame<FE::TDescriptorScalarType, FE::DescriptorSize>>`
  - `voyahslam::map::StereoFrame<FE>` — alias for `Frame<FE, BinocularCameraFrame<FE::TDescriptorScalarType, FE::DescriptorSize>>`

- [ ] **Step 1: Create `include/voyahslam/map/frame.h`**

Guard: `VOYAHSLAM_MAP_FRAME_H`. Header-only. Namespace `voyahslam::map`. Include `<voyahslam/defs.h>`, `<voyahslam/map/frame_id.h>`, `<voyahslam/features/camera_frame.h>`, `<voyahslam/features/binocular_camera_frame.h>`.

- [ ] **Step 2: Write tests in `tests/map/frame_test.cpp`**

```cpp
// Test: MonoFrameConstruction — create MonoFrame<OrbFeatureExtractor> with FrameId{0}, identity pose,
//       a CameraFrame built from ImageFeatures, timestamp 1.0. Verify all accessors.
// Test: StereoFrameConstruction — same with StereoFrame and BinocularCameraFrame
// Test: MutablePose — create frame, modify mutable_pose(), verify pose() reflects change
```

- [ ] **Step 3: Add to `tests/CMakeLists.txt`, build and run**

- [ ] **Step 4: Commit**

```bash
git add include/voyahslam/map/frame.h tests/map/frame_test.cpp tests/CMakeLists.txt
git commit -m "Add Frame with MonoFrame/StereoFrame aliases"
```

---

### Task 7: ObservationStorage (SoA)

**Files:**
- Create: `include/voyahslam/map/observation_storage.h`
- Create: `tests/map/observation_storage_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `MapPointId`, `FrameId` (Task 2)
- Produces:
  - `voyahslam::map::ObservationType` — enum `{ Monocular, Stereo }`
  - `voyahslam::map::ObservationStorage<ObservationType Type>` — `void Add(MapPointId, FrameId, uint16_t feat_idx)` (always available), `void Add(MapPointId, FrameId, uint16_t, precision_t depth, int16_t right_idx)` (Stereo only, compile-time disabled for Mono), `size_t size() const`, `MapPointId mappoint_id(size_t) const`, `FrameId frame_id(size_t) const`, `uint16_t feature_index(size_t) const`

- [ ] **Step 1: Create `include/voyahslam/map/observation_storage.h`**

Guard: `VOYAHSLAM_MAP_OBSERVATION_STORAGE_H`. Header-only. Use `if constexpr (Type == ObservationType::Stereo)` to enable/disable the stereo `Add` overload and the stereo vectors.

- [ ] **Step 2: Write tests in `tests/map/observation_storage_test.cpp`**

```cpp
// Test: MonocularAdd — add 3 observations, verify size() == 3, verify mappoint_id/frame_id/feature_index for each
// Test: StereoAdd — ObservationStorage<Stereo>, add with depth and right_idx, verify all fields
// Test: MonocularStereoOverloadDisabled — static_assert or SFINAE check that 5-arg Add does not compile for Monocular
```

- [ ] **Step 3: Add to `tests/CMakeLists.txt`, build and run**

- [ ] **Step 4: Commit**

```bash
git add include/voyahslam/map/observation_storage.h tests/map/observation_storage_test.cpp tests/CMakeLists.txt
git commit -m "Add ObservationStorage: SoA observation table with mono/stereo variants"
```

---

### Task 8: Event System — Types, Concrete Events, and Delivery

Add moodycamel::ConcurrentQueue dependency and implement the full event system: `Event` base, concrete events, `Observable`, `AsyncObserver`, `Listener`.

**Files:**
- Modify: `conanfile.txt` — add `concurrentqueue/1.0.4`
- Modify: `src/CMakeLists.txt` — add `PUBLIC concurrentqueue::concurrentqueue` to link libs
- Create: `include/voyahslam/events/event_types.h`
- Create: `include/voyahslam/events/map_events.h`
- Create: `include/voyahslam/events/observable.h`
- Create: `include/voyahslam/events/async_observer.h`
- Create: `include/voyahslam/events/listener.h`
- Create: `tests/events/event_system_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `Recyclable`, `ObjectPool` (Task 1), `MapPointId`, `FrameId` (Task 2), `Frame` (Task 6)
- Produces:
  - `voyahslam::events::EventType` — enum class uint8_t with values from spec §3
  - `voyahslam::events::Event` — struct with `EventType type`, `void (*recycle_)(Event*)`, protected constructor
  - `voyahslam::events::MapPointsAdded<FE>`, `MapPointsUpdated<FE>`, `MapPointsRemoved`, `KeyFrameCreated<FE,CF>`, `KeyFramePoseUpdated`, `NewKeyFrameCandidate<FE,CF>`, `PositionUpdated`, `LoopDetected` — concrete event structs
  - `voyahslam::events::Observable` — `void ConnectAsync(AsyncObserver&)`, `void ConnectSync(void(*)(void*, Event*), void*)`, `void Notify(Event*)`, `size_t async_count() const`
  - `voyahslam::events::AsyncObserver` — `Listener GetListener()`
  - `voyahslam::events::Listener` — `Event* GetNextEvent()`, move-only, auto-recycles

- [ ] **Step 1: Add `concurrentqueue/1.0.4` to `conanfile.txt`**

Add under `[requires]`.

- [ ] **Step 2: Add `concurrentqueue::concurrentqueue` to `src/CMakeLists.txt`**

Add as PUBLIC link dependency so downstream targets (tests) get the include path.

- [ ] **Step 3: Re-run conan install for debug preset**

Run: `conan install . --output-folder=cmake-build-debug --build=missing -s build_type=Debug`

- [ ] **Step 4: Create `include/voyahslam/events/event_types.h`**

Guard: `VOYAHSLAM_EVENTS_EVENT_TYPES_H`. Namespace `voyahslam::events`. Define `EventType` enum and `Event` struct. Include `<cstdint>`.

`Event` needs TWO function pointers so Observable::Notify can call SetRefCount through the base pointer:

```cpp
struct Event {
    EventType type;
    void (*recycle_)(Event*);
    void (*set_ref_count_)(Event*, int);
protected:
    Event(EventType t, void (*r)(Event*), void (*s)(Event*, int))
        : type(t), recycle_(r), set_ref_count_(s) {}
};
```

- [ ] **Step 5: Create `include/voyahslam/events/map_events.h`**

Guard: `VOYAHSLAM_EVENTS_MAP_EVENTS_H`. Namespace `voyahslam::events`. Define all concrete event structs. Each inherits `Event` + `memory::Recyclable<Self, memory::ThreadSafePolicy>`. Constructor wires both `recycle_` and `set_ref_count_` lambdas:

```cpp
MapPointsAdded()
    : Event(EventType::MapPointsAdded,
            [](Event* e) {
                static_cast<MapPointsAdded*>(e)->
                    Recyclable<MapPointsAdded, memory::ThreadSafePolicy>::Recycle();
            },
            [](Event* e, int n) {
                static_cast<MapPointsAdded*>(e)->SetRefCount(n);
            }) {}
```

Include `<voyahslam/events/event_types.h>`, `<voyahslam/memory/recyclable.h>`, `<voyahslam/defs.h>`, `<voyahslam/map/map_point_id.h>`, `<voyahslam/map/frame_id.h>`.

- [ ] **Step 6: Create `include/voyahslam/events/listener.h`**

Guard: `VOYAHSLAM_EVENTS_LISTENER_H`. Non-template class. Include `<concurrentqueue/concurrentqueue.h>`. Forward-declare `Event`. Implement `GetNextEvent()` and destructor as in spec §2. Move constructor nulls out source's `current_`. Copy deleted.

- [ ] **Step 7: Create `include/voyahslam/events/async_observer.h`**

Guard: `VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H`. Non-template class. Include `<voyahslam/events/listener.h>`. Owns `ConcurrentQueue<Event*> queue_`. `GetListener()` returns `Listener{queue_}`. Expose `queue()` for Observable to connect.

- [ ] **Step 8: Create `include/voyahslam/events/observable.h`**

Guard: `VOYAHSLAM_EVENTS_OBSERVABLE_H`. Non-template class. Include `<voyahslam/events/async_observer.h>`, `<voyahslam/events/event_types.h>`, `<vector>`.

`Notify(Event* event)` implementation:
1. Call each sync handler: `handler.fn(handler.ctx, event)`
2. If `async_queues_` is non-empty: `event->set_ref_count_(event, async_queues_.size())`, then enqueue to each queue
3. If `async_queues_` IS empty: skip ref counting entirely (sync-only path, event may be stack-allocated)

- [ ] **Step 9: Write tests in `tests/events/event_system_test.cpp`**

```cpp
// Test: EventTypeTag — create MapPointsAdded<OrbFeatureExtractor> from pool, verify event.type == EventType::MapPointsAdded
// Test: ListenerAutoRecycle — pool of 2 events, create both, push to queue, drain via Listener, verify both returned to pool (pool can Create 2 more)
// Test: ListenerMoveSemantics — create Listener, move it, verify moved-from doesn't double-recycle
// Test: ObservableSyncHandler — connect sync handler that increments counter, Notify, verify counter == 1
// Test: ObservableAsyncFanOut — connect 2 AsyncObservers, create event with pool, Notify, verify each observer's Listener gets the event, verify event returned to pool after both drain
// Test: ObservableSyncAndAsync — connect 1 sync + 1 async, Notify, verify sync handler called and async observer gets event
// Test: ObservableNoAsyncSkipsRefCount — connect only sync handler, Notify with stack-allocated event (no pool), verify no crash
```

- [ ] **Step 10: Add to `tests/CMakeLists.txt`, build and run**

Run: reconfigure with preset, build, run all tests.
Expected: All new event tests pass, all 78 existing tests still pass.

- [ ] **Step 11: Commit**

```bash
git add conanfile.txt src/CMakeLists.txt include/voyahslam/events/ tests/events/ tests/CMakeLists.txt
git commit -m "Add event system: Event types, Observable, AsyncObserver, Listener

Lock-free pub-sub using moodycamel::ConcurrentQueue. Observable supports
sync (inline callback) and async (queue) connections. Listener auto-recycles
events via function pointer, no virtual dispatch."
```

---

## Verification

After all tasks:

1. `cmake --build cmake-build-debug/build/Debug` — clean build
2. `ctest --test-dir cmake-build-debug/build/Debug --output-on-failure` — all tests pass
3. Repeat with release preset: `conan install . --output-folder=build --build=missing -s build_type=Release`, configure with `conan-release` preset, build and test
4. Verify no new compiler warnings with `-Wall -Wextra`
