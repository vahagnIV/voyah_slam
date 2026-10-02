# voyahslam — Living Design Document

> **Purpose:** the single source of truth for *what* we are building and *why*.
> Update this file whenever a design decision is made or changed. A new session
> (human or Claude) should be able to read this file and continue the work.
>
> Detailed per-feature specs and implementation plans live in
> `docs/superpowers/specs/` and `docs/superpowers/plans/`. When they disagree
> with this document, **this document wins** — fix the spec or note the change here.

Last updated: 2026-10-02

---

## 1. Goal

An ORB-SLAM3-inspired visual SLAM library (mono + stereo) with a different
concurrency model: **no locks anywhere on the map**. Threads never share mutable
state; they communicate exclusively through publish/subscribe events over
lock-free queues, and every thread keeps its own replica of the map.

Predecessor project: `../slamlab2`. Image processing, feature extraction,
geometry, distortion models etc. were ported from there.

## 2. Core Principles

1. **No locks.** No mutex anywhere on the map or event path. Cross-thread
   communication only via events on `moodycamel::ConcurrentQueue`; event pools
   are lock-free (§6.1).
2. **One replica of the map per thread.** Tracker, Mapper, LoopCloser (and later
   the Visualizer) each own a local copy: `MapPointStorage`, `Frame`s,
   `ObservationStorage`.
3. **Mapper is the single writer.** *All* changes to map objects — creating,
   updating, culling, fusing map points; creating keyframes; adjusting keyframe
   poses; closing loops — happen in the Mapper. The Mapper then publishes diff
   events and every other replica applies them verbatim.
4. **Other threads are read-only on the map.** They may compute things *from*
   their replica (a pose, a loop candidate) and publish those as requests or
   facts, but they never mutate map objects themselves.
5. **IDs are never reused.** A `MapPointId` / `FrameId` refers to one object for
   the entire run. A stale ID in an event from a lagging replica is therefore
   always detectable (the slot is invalid) and can never alias a newer object.
6. **No virtual dispatch on hot paths.** Events dispatch on a type tag +
   `static_cast`; recycling goes through a function pointer. Mono/stereo is chosen
   at compile time via templates.
7. **Structure-of-Arrays storage** for map points and observations, so the hot
   loops (projection, matching, BA) run over contiguous Eigen matrices.

## 3. Terminology

| voyahslam | ORB-SLAM3 | Notes |
|---|---|---|
| `Frame` | KeyFrame | Only keyframes become `Frame` objects. |
| current tracking state | Frame | Lives privately inside the Tracker; never published as a map object. |
| `MapPointStorage` | set of MapPoints | SoA, indexed by `MapPointId`. |
| `ObservationStorage` | MapPoint↔KeyFrame observations | Flat SoA table. |
| `CameraFrame` / `BinocularCameraFrame` | features of a frame | Mono / stereo feature containers. |
| `StereoRig` | camera calibration | Immutable, shared by all threads. |

## 4. Threads and Responsibilities

```
            NewKeyFrameCandidate                LoopDetected
 Tracker ───────────────────────────► Mapper ◄────────────── LoopCloser
    ▲                                   │                        ▲
    │      map diff events              │   map diff events      │
    └───────────────────────────────────┴────────────────────────┘
                                        │
                                        ▼   map diffs, PositionUpdated (from Tracker)
                                   Visualizer (future)
```

### Tracker
- Input: images. Extracts features, tracks against **its own** map replica.
- Optimizes **only its own camera pose** relative to the map (motion-only BA).
  Never touches `Frame`s, map points, or observations.
- Publishes `PositionUpdated` (frame id, pose, timestamp) for each tracked frame.
- Decides when a keyframe *might* be needed and publishes
  `NewKeyFrameCandidate` (features, pose, and the tracked
  `(map_point_id, feature_idx)` associations). The Mapper makes the final decision.
- Its replica only changes when Mapper diff events arrive. Between publishing a
  candidate and receiving the resulting diffs it simply keeps tracking with the
  map it has. This lag is accepted (ORB-SLAM3 behaves similarly while
  LocalMapping processes a keyframe).
- Any temporary helper points the Tracker uses (e.g. ORB-SLAM3's stereo "temporal
  points" for the last frame) are Tracker-private and never part of the map.

### Mapper
- The authoritative map and the **only** map writer.
- Consumes `NewKeyFrameCandidate`: accepts/rejects, creates the `Frame`, records
  observations, triangulates new map points, local BA, map point culling and
  fusion, keyframe culling.
- Consumes `LoopDetected`: performs the **entire** loop closure — validation,
  Sim3/SE3 correction, map point fusion, essential graph optimization, global BA.
- Publishes diff events after every change (see §6).
- Validates incoming IDs against its own replica: events from other threads were
  computed on a lagging replica, so referenced points/frames may already be culled.
  Thanks to ID non-reuse this is a plain `IsValid()` check.

### LoopCloser
- **Detection only.** Place recognition on its replica (keyframe database /
  BoW-like) and, possibly, geometric verification of the candidate.
- Publishes `LoopDetected` (query frame, matched frame, and whatever verification
  data is useful to the Mapper). Never modifies the map.

### Visualizer (future)
- Subscribes to Mapper diffs and Tracker poses; owns a render-only replica.

## 5. Data Layer (implemented)

Detailed spec: `docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md`.

| Component | Header | Summary |
|---|---|---|
| `ObjectPool<T, SyncPolicy>` | `memory/object_pool.h` | Pre-allocated slab, intrusive free-list, asserts on exhaustion. |
| `Recyclable<T, SyncPolicy>` | `memory/recyclable.h` | CRTP atomic ref count (acq_rel); returns object to its pool at 0. `SetRefCount(n)` for fan-out. |
| `ThreadSafePolicy` / `NoSyncPolicy` | `memory/sync_policy.h` | Locking policy for the pool. |
| `MapPointId`, `FrameId` | `map/*_id.h` | Strong ID wrappers. |
| `MapPointStorage<FE>` | `map/map_point_storage.h` | SoA: positions, descriptors, normals, num observations, valid flags. |
| `Frame<FE, CF>` | `map/frame.h` | Keyframe: id, pose, camera frame, timestamp. `MonoFrame<FE>`, `StereoFrame<FE>` aliases. |
| `ObservationStorage<Type>` | `map/observation_storage.h` | SoA observation table; stereo fields compiled out for mono. |
| `StereoRig` | `features/stereo_rig.h` | K left/right, R/t right→left, baseline, distortion models. |
| `BinocularCameraFrame<T, L>` | `features/binocular_camera_frame.h` | Left/right `CameraFrame`, per-feature depth, right match index. |
| `Event`, `EventType` | `events/event_types.h` | Base event: type tag + `recycle_` / `set_ref_count_` fn pointers. |
| concrete events | `events/map_events.h` | See §6. |
| `Observable` | `events/observable.h` | Sync handlers (inline, fn ptr + ctx) run first, then async enqueue with ref count = #async queues. |
| `AsyncObserver` | `events/async_observer.h` | One queue per observer (1:1 channel). |
| `Listener` | `events/listener.h` | Iterates a queue; auto-recycles previous/last event; move-only. |

Ported from slamlab2: `images/`, `io/`, `algebra/`, `geometry/`, `features/`
(ORB + SIFT extractors, brute-force matcher, Kannala-Brandt distortion), `logging/`.

## 6. Events

| Event | Producer → Consumer | Payload |
|---|---|---|
| `NewKeyFrameCandidate<FE, CF>` | Tracker → Mapper | frame, observations `(map_point_id, feature_idx)` |
| `PositionUpdated` | Tracker → Visualizer/others | frame_id, pose, timestamp |
| `MapPointsAdded<FE>` | Mapper → all | start_id, count, positions, descriptors, normals |
| `MapPointsUpdated<FE>` | Mapper → all | ids, positions, descriptors, normals |
| `MapPointsRemoved` | Mapper → all | ids |
| `KeyFrameCreated<FE, CF>` | Mapper → all | frame |
| `KeyFramePoseUpdated` | Mapper → all | frame_id, pose |
| `LoopDetected` | LoopCloser → Mapper | query/match frames + verification data (TBD) |

Rules:
- Events are coarse-grained (one event per batch, Eigen matrices inside).
- Each concrete event type has its own `ObjectPool`.
- Consumers never call `Recycle()` — the `Listener` does.
- Per channel delivery is FIFO, so a replica applying diffs in order always ends
  up identical to the Mapper's map as of the last applied event.
- Because IDs are never reused, `MapPointsAdded` allocates a contiguous id range
  `[start_id, start_id + count)` and every replica stores point `i` at the same slot.

### 6.1 Event Lifecycle: Pooled, Reference-Counted, Acknowledged

**Goal:** publishing an event must not hit the general-purpose heap allocator.
Events are produced at frame rate on every thread; `new`/`delete` per event
means allocator contention between threads and unpredictable latency. Instead,
every concrete event type has a pre-allocated **`ObjectPool`**, and each event
is a **`Recyclable`** object that knows how to return itself to its pool.

**The building blocks (`include/voyahslam/memory/`):**

- `ObjectPool<T, SyncPolicy>` — one contiguous slab of `max_count` slots
  allocated once in the constructor. Free slots form a stack of slot indices
  whose links live in a separate `atomic<uint32_t>` array, so slot memory is
  never read by the allocator and is not touched (or committed by the OS) until
  first use. `Create(args...)` pops a slot and placement-constructs `T`
  (asserts when exhausted); `TryCreate(args...)` returns `nullptr` instead;
  `Recycle(T*)` destroys `T` and pushes the slot back. All O(1).
- `Recyclable<T, SyncPolicy>` — CRTP mixin that gives an object an atomic
  reference count (starts at **1**) and a back-pointer to the pool that created it.
  `Recycle()` decrements; whoever drops the count to 0 returns the object to its
  pool. `SetRefCount(n)` sets the count in one store (used for fan-out).
- `SyncPolicy` — `ThreadSafePolicy`: the free-list is a lock-free Treiber stack
  (CAS on a 64-bit head packing `tag << 32 | index`; the tag is bumped on every
  push/pop to defeat ABA). Required for event pools, because `Create` runs on the
  producer thread and `Recycle` on whichever consumer acknowledges last.
  `NoSyncPolicy`: same structure with plain loads/stores, for single-thread pools.
- `Event` — non-virtual base with a type tag and two function pointers
  (`recycle_`, `set_ref_count_`) wired by each concrete event's constructor, so
  code holding only an `Event*` can manage the lifetime without virtual dispatch
  and without knowing the concrete type.

**The lifecycle of one async event:**

```
 producer thread                         consumer threads (N async observers)
 ───────────────                         ────────────────────────────────────
 1. e = pool.Create()        ref = 1
 2. fill payload
 3. observable.Notify(e)
      a. sync handlers run inline (e alive, producer still owns it)
      b. SetRefCount(N)       ref = N    ◄── ownership transferred to consumers
      c. enqueue e on each of the N queues
 4. producer forgets e (must not touch it again)
                                          5. listener.GetNextEvent() → e
                                             consumer reads the payload
                                          6. next GetNextEvent() / ~Listener()
                                             = acknowledgement: e->Recycle()
                                             ref = ref - 1
                                          7. last consumer to acknowledge
                                             (ref → 0) returns e to the pool
```

**Acknowledgement is implicit.** A consumer never calls `Recycle()` itself. The
`Listener` treats "asking for the next event" as "I am done with the previous
one", and its destructor acknowledges the last event it handed out. So the
pointer returned by `GetNextEvent()` is a **borrow**, valid until the next
`GetNextEvent()` call or until the `Listener` is destroyed.

**Rules for consumers:**
1. Do not keep an `Event*` (or references/`Map`s into its payload) beyond the
   next `GetNextEvent()`. Copy whatever must outlive it into the local replica.
2. Treat the payload as **read-only**. With fan-out the same object is read
   concurrently by all N consumers, so moving data *out* of an event (e.g.
   `std::move(e->positions)`) is a data race.
3. Don't sit on events: until every consumer has acknowledged, the slot is not
   back in the pool. A stalled consumer drains the pool and the producer's
   next `Create()` asserts. Pool sizes must cover the worst-case backlog.
4. A `Listener` is move-only; moving it transfers the borrowed event, so it is
   acknowledged exactly once.

**Rules for producers:**
1. **`Notify()` always takes ownership.** After it returns the producer must not
   touch the event. With N > 0 async observers, the count is set to N and the
   consumers release it; with none, `Notify()` recycles it right after the sync
   handlers. The producer doesn't need to know how it is wired.
2. An event with a null `recycle_` is never recycled — this is how a
   stack-allocated event is published to sync-only observers.
3. Sync handlers run inline before the async enqueue and must not retain the
   event or `AddRef()` it — the subsequent `SetRefCount(N)` would overwrite that
   reference.
4. Use `TryCreate()` where exhaustion is an expected, recoverable condition
   (back-pressure) and `Create()` where it indicates a bug.

## 7. Coding Conventions

**Naming (Google C++ style):**
- Classes, structs, functions, methods: `PascalCase` — `MapPointStorage`, `Add()`, `GetNextEvent()`.
- Trivial accessors may be `snake_case` and mirror the member — `size()`, `positions()`, `normalized_points()`.
- Local variables and parameters: `snake_case`.
- Member variables: `snake_case_` with trailing underscore.
- Parameters passed by non-const reference (outputs): `out_` prefix — `TPoint3DF& out_triangulated`.
- Constants: `kPascalCase` — `kInitialCapacity`, `kDescSize`.
- Template parameters and type aliases: `T` prefix — `TFeatureExtractor`, `TDescriptors`, `TPose`.
- Enums: `enum class` with `PascalCase` values.
- Namespaces: `voyahslam::<module>` (`map`, `events`, `memory`, `features`, `geometry`, …).

**Structure:**
- C++20.
- Include guards `VOYAHSLAM_<PATH>_H` (no `#pragma once`).
- Public headers included as `#include <voyahslam/...>`.
- Template classes header-only; non-template classes have a `.cpp` in `src/` mirroring `include/`.
- `[[nodiscard]]` on const accessors.
- Move semantics throughout.
- Templates over the feature extractor are explicitly instantiated with
  `INSTANTIATE_CLASS` (`features/instantiation.h`) for ORB and SIFT.
- Formatting: `.clang-format` in repo root (4-space indent).
- New sources go in `src/CMakeLists.txt`, new tests in `tests/CMakeLists.txt`
  (mirroring the `include/` layout under `tests/`).

**Build & test (Conan 2 + CMake):**
- Dependencies (`conanfile.txt`): gtest, eigen, stb, opencv, spdlog, concurrentqueue.
- Debug: `cmake --build cmake-build-debug/build/Debug && ctest --test-dir cmake-build-debug/build/Debug --output-on-failure`
- Release: `conan install . --output-folder=build --build=missing -s build_type=Release`, then the `conan-release` preset.

**Workflow:** brainstorm → spec (`docs/superpowers/specs/`) → plan
(`docs/superpowers/plans/`) → implement task-by-task with tests on a feature
branch → review → merge to `master`. Update this document at the end.

## 8. Decisions Log

| Date | Decision | Rationale |
|---|---|---|
| 2026-09-30 | Event delivery layer (`Observable`/`AsyncObserver`/`Listener`) is non-template, carries `Event*`. | Avoid template bloat; concrete types only known at the consumer's `switch`. |
| 2026-09-30 | Sync handlers run before async enqueue; sync-only notify skips ref counting. | No race on the event pointer; stack-allocated events possible for sync. |
| 2026-09-30 | `Recyclable::Recycle()` uses `acq_rel`. | Correct cross-thread release of event payloads. |
| 2026-10-02 | Zero locks; each thread owns a map replica; pub/sub only. | Core design goal. |
| 2026-10-02 | Mapper is the only map writer, including all loop closure logic. LoopCloser only detects. | Single writer keeps replicas consistent by construction. |
| 2026-10-02 | Tracker optimizes only its own pose; never mutates map objects. Its replica changes only via Mapper diffs. | Same. |
| 2026-10-02 | `MapPointId` / `FrameId` are never reused. | Stale IDs from lagging replicas are detectable and can't alias new objects; replicas share slot layout. |
| 2026-10-02 | Tracker lag between `NewKeyFrameCandidate` and resulting diffs is accepted. | Matches ORB-SLAM3 behaviour; Tracker keeps tracking with its current replica. |
| 2026-10-02 | `ObjectPool` free-list is a lock-free Treiber stack with tagged head; links in a separate index array. | Removes the last mutex from the event path; slot memory never read concurrently with construction. Verified with ThreadSanitizer. |
| 2026-10-02 | `Observable::Notify()` always takes ownership of the event. | One rule for producers regardless of how observers are wired. |

## 9. Open Questions

1. **ID width:** `uint32_t` (4·10⁹ ids, enough for any realistic run) vs `uint64_t`.
   With no reuse, memory — not id width — is the real limit (see 2).
2. **Storage growth without reuse:** `MapPointStorage` is indexed directly by id,
   so culled points leave dead slots forever. ORB-SLAM culls most new points
   early, so dead slots dominate on long runs (~90 B/point for ORB with double
   precision). Options: accept it; chunked storage that frees fully-dead chunks;
   or an id→slot indirection with compaction (must then be identical across replicas
   or be per-replica).
3. **Large diffs:** a loop closure corrects many points/keyframes at once — one
   huge `MapPointsUpdated` or chunked events?
4. **Mapper back-pressure:** how the Tracker knows the Mapper is busy (ORB-SLAM3's
   `AcceptKeyFrames()`): an atomic flag (lock-free) or an event.
5. **Global BA after loop closure:** runs inside the Mapper (blocks local mapping)
   or in a Mapper-owned worker whose result the Mapper applies and publishes?
6. **`LoopDetected` payload:** just frame ids, or also matches / Sim3 estimate from
   geometric verification?
7. **Map-level versioning:** do events need a map version number so consumers can
   tell which Mapper state a request was computed against?
8. Covisibility graph / spanning tree / essential graph representation (needed for
   local BA and loop closure) — not designed yet.
9. Keyframe culling and how `Frame` removal propagates (`KeyFrameRemoved` event?).
10. IMU / multi-map (Atlas) support — out of scope for now.
11. **Payload buffers are not actually reused.** `ObjectPool::Recycle()` runs
    `~T()` and `Create()` placement-constructs a fresh `T`, so an event's Eigen
    matrices / `std::vector`s free their heap buffers on recycle and reallocate
    when the next producer fills them. The pool saves the event *shell*, not the
    payload — which is where the bulk of the allocation is. Options: keep objects
    constructed in the pool and `Reset()` them instead of destroying them
    (`std::vector::clear()` keeps capacity; Eigen dynamic matrices do **not** —
    `resize()` to a different size reallocates, so they'd need fixed max-capacity
    storage plus a `count`, or `Eigen::Map` over a capacity-preserving buffer).
12. ~~Event pool takes a mutex~~ — resolved 2026-10-02 (lock-free Treiber stack).
13. ~~Ownership in `Notify()` depends on wiring~~ — resolved 2026-10-02
    (`Notify()` always takes ownership).
14. **Pool sizing.** Slot memory is now only committed on first use, but the
    index array (4 B/slot) is written up front, and the default `max_count` is
    still 1,000,000. Pools should be sized per event type from the worst-case
    backlog (consumer rule 3 in §6.1).

## 10. Status and Next Steps

**Done (on `master`):**
- Ported core modules from slamlab2 (78 tests).
- Core data layer + event system (merge `91b56aa`).

**Required follow-up from 2026-10-02 decisions:**
- Remove free-list reuse from `MapPointStorage` (ids are monotonic now); decide
  storage-growth strategy (Open Question 2).
- Make sure `FrameId` allocation is monotonic too.
- Decide on Open Questions 11 (event payload buffer reuse) and 14 (pool sizing).

**Next:**
- Design the Tracker (initialization mono/stereo, track-with-motion-model,
  track-reference-keyframe, track-local-map, relocalization, keyframe decision).
- Then the Mapper, then the LoopCloser.
