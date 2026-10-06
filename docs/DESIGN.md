# voyahslam — Living Design Document

> **Purpose:** the single source of truth for *what* we are building and *why*.
> Update this file whenever a design decision is made or changed. A new session
> (human or Claude) should be able to read this file and continue the work.
>
> Detailed per-feature specs and implementation plans live in
> `docs/superpowers/specs/` and `docs/superpowers/plans/`. When they disagree
> with this document, **this document wins** — fix the spec or note the change here.

Last updated: 2026-10-05

---

## 1. Goal

An ORB-SLAM3-inspired visual SLAM library (mono + stereo) with a different
concurrency model: **no locks anywhere on the map**. Threads never share mutable
state; they communicate exclusively through publish/subscribe events over
lock-free queues, and every thread keeps its own replica of the map.

Predecessor project: `../slamlab2`. Image processing, feature extraction,
geometry, distortion models etc. were ported from there.

**Long-term goal beyond ORB-SLAM3:** *mixed* maps — monocular and binocular
keyframes in the same map (e.g. a stereo rig whose second camera drops out, or
merging mono and stereo sessions). Data structures should not hard-wire one
sensor type per map. For now the Tracker accepts both measurement types; how
keyframes of different types share the map is still open (Open Question 17).

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
5. **IDs are never reused.** A `FrameId` is unique for the entire run; a
   `MapPointId` is unique within its map (point storage restarts per map), so
   `(map_id, id)` identifies a point. Within a map a stale ID in an event from
   a lagging replica is therefore always detectable (the slot is invalid) and
   can never alias a newer object.
6. **No virtual dispatch on hot paths.** Events dispatch on a type tag +
   `static_cast`; recycling goes through a function pointer. Mono/stereo is chosen
   at compile time via templates.
7. **Structure-of-Arrays storage** for map points and observations, so the hot
   loops (projection, matching, BA) run over contiguous Eigen matrices.
8. **OpenCV is an implementation detail.** Our public headers
   (`include/voyahslam/`, `vizualization/include/voyahviz/`) never include
   OpenCV; public APIs take and return our types (`images::Image`,
   measurements, `ImageFeatures`, ...) and OpenCV is linked `PRIVATE`, so a
   user of the library never needs its headers or `find_package(OpenCV)`.
   Small dependencies (Eigen, spdlog, concurrentqueue) may appear in public
   headers — requiring users to have them is fine.

## 3. Terminology

| voyahslam | ORB-SLAM3 | Notes |
|---|---|---|
| `Frame` | KeyFrame | Only keyframes become `Frame` objects. |
| current tracking state | Frame | Lives privately inside the Tracker; never published as a map object. |
| `MapPointStorage` | set of MapPoints | SoA, indexed by `MapPointId`. |
| `ObservationStorage` | MapPoint↔KeyFrame observations | Flat SoA table. |
| `MonocularMeasurement` / `BinocularMeasurement` | Frame (features part) | Features of one image / a stereo pair at a timestamp. Input to `Track()`. |
| `StereoRig` | camera calibration | Immutable, shared by all threads. |

## 4. Threads and Responsibilities

```
               FrameTracked                     LoopDetected
 Tracker ───────────────────────────► Mapper ◄────────────── LoopCloser
    ▲                                   │                        ▲
    │      map diff events              │   map diff events      │
    └───────────────────────────────────┴────────────────────────┘
                                        │
                                        ▼   map diffs, FrameTracked (from Tracker)
                                   Visualizer (MapViewer)
```

### Slam (orchestrator) and EventNode

`slam::Slam<TFeatureExtractor>` (`slam/slam.h`) owns the three components and
wires them asynchronously in its constructor:

| Publisher | → Async observers |
|---|---|
| Tracker | Mapper |
| Mapper | Tracker, LoopCloser |
| LoopCloser | Mapper |

`Start()` starts consumers before producers (Mapper, LoopCloser, Tracker);
`Stop()` stops producers first (Tracker, LoopCloser, Mapper); the destructor
calls `Stop()`. `Slam` is neither copyable nor movable — components hold
pointers to each other's inboxes. Extra observers (visualizer, logging) attach
via `slam.tracker()/mapper()/loop_closer().AddObserver(...)` before `Start()`.

Every component derives from `events::EventNode` (`events/event_node.h`, no
virtuals): an `Observable` it publishes through, one `AsyncObserver` inbox it
consumes, `AddObserver(AsyncObserver&)`, `AddSyncObserver(fn, ctx)`, `inbox()`.
Components are templated on `TFeatureExtractor`, declared in headers and
defined in `src/` with `INSTANTIATE_CLASS` (ORB + SIFT).

| Component | Header | Public API so far |
|---|---|---|
| `tracking::Tracker<FE>` | `tracking/tracker.h` | `Start()`, `Stop()`, `Track(TMonocularMeasurement)`, `Track(TBinocularMeasurement)`, `ProcessPending()`, `state()`, `current_map_id()`, `pending_measurements()` |
| `mapping::Mapper<FE>` | `mapping/mapper.h` | `Mapper(MapperOptions)`, `Start()`, `Stop()`, `ProcessPending()`, `state()`, `current_map_id()` |
| `loop_closing::LoopCloser<FE>` | `loop_closing/loop_closer.h` | `Start()`, `Stop()`, `ProcessPending()` (drains and acknowledges only) |

Each component owns a `events::WorkerThread` (`events/worker_thread.h`) bound to
the `events::WakeSignal` of its `EventNode` (`events/wake_signal.h`) and a
public `[[nodiscard]] bool ProcessPending()` (one pass, returns true if it did
work; tests drive it without a thread). Spec:
`docs/superpowers/specs/2026-10-03-tracking-states-design.md`.

**Waking without locks (resolves OQ 20).** `WakeSignal` is a
`std::atomic<uint32_t>` counter with C++20 `wait`/`notify_one` (a futex on
Linux): `Snapshot()`, `Notify()`, `Wait(seen)`. `AsyncObserver::Push()` enqueues
and notifies the attached signal (`SetWakeSignal()`); `Tracker::Track()`
notifies after enqueuing. `WorkerThread` loop: `seen = Snapshot(); if (!running)
break; if (!process_pending()) Wait(seen);`. The snapshot is taken **before**
the `running` check — otherwise a `Stop()` landing between the check and the
snapshot is already counted in `seen` and `Wait` would never return (spec
amendment found during implementation). A `Notify()` between `Snapshot()` and
`Wait()` changes the counter, so no wake-up is lost.

**Geometry status.** Monocular initialization is real (see "Monocular
Initialization"): `mapping::MapInitializer<FE>` (`mapping/map_initializer.h`:
`TryInitialize()`, `Reset()`) matches, reconstructs and scales the two views.
Stereo initialization is real too (see "Stereo Initialization"). Tracking
against the map is real (see "Tracking Against the Map"); the `PoseEstimator`
stub was deleted on 2026-10-05.

### Tracker
- **State machine** (`TrackerState`, readable from any thread via `state()`):
  `WaitingForMap` -> `Tracking` -> `Lost` -> `Tracking`...
  One `ProcessPending()` pass: (1) drain the whole inbox applying the `map_id`
  consumer rule (§6) — every Mapper diff of the current map is applied to the
  Tracker's `tracking::MapReplica`; `MapCreated` sets `last_frame_` to the
  keyframe's replica entry (measurement, `T_cw`, observations as matches),
  resets the velocity, sets the map's frame id and moves
  `WaitingForMap`/`Lost` to `Tracking`; (2) take **one**
  measurement: in `WaitingForMap`/`Lost` publish it as
  `InitializationMeasurement` (`map_id` 0); in `Tracking` run the frame-to-map
  tracking described in "Tracking Against the Map" — success publishes
  `FrameTracked`, failure moves to `Lost` and then publishes
  `TrackingLost(current map)`. `Lost` currently behaves like `WaitingForMap`
  (relocalization will live there; deferred, TODO).
- Runs its **own thread with two input queues**:
  - `inbox()` — map diffs from the Mapper (events),
  - `measurements_` — `std::variant<TMonocularMeasurement, TBinocularMeasurement>`
    filled by `Track()`.

  `Track()` does not track: it moves the measurement into the queue and returns
  (taken by value — pass an rvalue to avoid copying the feature matrices).
  Loop on the Tracker thread:
  ```
  while running:
      apply ALL pending map diffs from inbox()     // replica is now current
      take a measurement (see Open Question 19)
      track it; publish FrameTracked
  ```
  Diffs and tracking run on the same thread, so the replica never changes while
  being tracked against — no locks. Two queues rather than one: map diffs get
  priority, stale measurements can be dropped, and a single
  `moodycamel::ConcurrentQueue` wouldn't order Mapper diffs relative to
  measurements anyway (it is FIFO per producer only).
- Input: feature measurements (mono or binocular), each with its capture
  timestamp. Both types may arrive in the same session (mixed-map goal, §1).
  Feature extraction happens before `Track()` (in the user application for now).
- Tracks against **its own** map replica.
- Optimizes **only its own camera pose** relative to the map (motion-only BA).
  Never touches `Frame`s, map points, or observations.
- Publishes `FrameTracked` for each tracked measurement: pose, the measurement
  copy, the inlier `(feature, map point)` pairs, the visible local-map points,
  the reference Frame, the local Frames, the stage and the match / inlier
  counts (§6). It does **not** decide on keyframes: the Mapper decides from
  `FrameTracked` (see "Keyframes and Map Growth").
- Its replica only changes when Mapper diff events arrive. Between publishing a
  `FrameTracked` and receiving the resulting `KeyFrameCreated` it simply keeps
  tracking with the map it has. This lag is accepted (ORB-SLAM3 behaves
  similarly while LocalMapping processes a keyframe); the next `KeyFrameCreated`
  for the last tracked measurement is merged into `last_frame_` at once (fast
  pickup).
- Any temporary helper points the Tracker uses (e.g. ORB-SLAM3's stereo "temporal
  points" for the last frame) are Tracker-private and never part of the map.

### Mapper
- The authoritative map and the **only** map writer.
- **State machine** (`MapperState`, `state()`): `Initializing` -> `Mapping`.
  *Initialization protocol:* while `Tracker` has no map it forwards
  measurements as `InitializationMeasurement`. In `Initializing` the Mapper
  feeds a **copy** of each to `MapInitializer::TryInitialize()` (the event is
  shared; never move out of it). Monocular initialization takes many frames
  (reference + a later frame), stereo one measurement. On success the Mapper
  creates a `Frame` per initial keyframe (ids from a monotonic counter, never
  reused across maps), takes `current_map_id_ = next_map_id_++`, moves to
  `Mapping`, then publishes one `KeyFrameCreated` per initial keyframe (mono:
  the first carries every initial point, stereo: one event) and then
  `MapCreated(map_id, frame id, pose)`; the Tracker switches to `Tracking` on
  `MapCreated`. `InitializationMeasurement` is ignored while `Mapping`.
  `TrackingLost` for the current map (in `Mapping`) clears keyframes and the
  map-point and observation storage, resets
  the initializer and returns to `Initializing`, so the next success starts
  map n+1 (relocalization / multi-map later). A stale `TrackingLost` is ignored.
  `LoopDetected` is only acknowledged for now.
- Consumes `FrameTracked` (current map, `Mapping` only): drains to the latest
  per map, decides with `ShouldCreateFrame` whether the measurement becomes a
  Frame, and creates it — tracked observations, stereo close points,
  triangulation against covisible neighbour Frames — published as **one**
  `KeyFrameCreated` (see "Keyframes and Map Growth"). Local BA, point fusion
  and culling of points and Frames follow in sub-project 3.
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

### Visualizer (MapViewer)
- `voyahviz::MapViewer` (see "Visualization Library") subscribes to Tracker
  and Mapper events through one inbox and builds a render-only `SceneState`
  from them. It is not part of `Slam`; the apps attach it.
- **Remote visualization (target):** SLAM and viewer run on different
  machines. On the SLAM machine an async observer serializes events and sends
  them over HTTP or ROS; on the viewer machine a receiver rebuilds the events
  and feeds the same `SceneState`/renderer code. Hence the viewer may use
  nothing but event contents. Camera images are a separate flow (acquisition
  and feature extraction are outside the SLAM logic).

## 5. Data Layer (implemented)

Detailed spec: `docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md`.

| Component | Header | Summary |
|---|---|---|
| `ObjectPool<T, SyncPolicy>` | `memory/object_pool.h` | Pre-allocated slab, intrusive free-list, asserts on exhaustion. |
| `Recyclable<T, SyncPolicy>` | `memory/recyclable.h` | CRTP atomic ref count (acq_rel); returns object to its pool at 0. `SetRefCount(n)` for fan-out. |
| `ThreadSafePolicy` / `NoSyncPolicy` | `memory/sync_policy.h` | Pool free-list mode: lock-free CAS / single-thread. |
| `ColumnBuffer<TScalar, Rows>` | `memory/column_buffer.h` | Grow-only `Rows × N` Eigen buffer; `Resize(n)` allocates only beyond capacity. |
| `MapPointId`, `FrameId` | `map/*_id.h` | Strong ID wrappers. |
| `MapPointStorage<FE>` | `map/map_point_storage.h` | SoA: positions, descriptors, normals, num observations, valid flags. |
| `MonocularMeasurement<T, L>` | `features/monocular_measurement.h` | `ImageFeatures` + K, raw keypoints (`distorted_points()`), undistorted & normalized points, `camera()` (`geometry::Camera`: K + distortion pointer), `timestamp()`. |
| `Frame<FE, TMeasurement>` | `map/frame.h` | Keyframe: id, pose, measurement; `timestamp()` comes from the measurement. `MonoFrame<FE>`, `StereoFrame<FE>` aliases. |
| `ObservationStorage<Type>` | `map/observation_storage.h` | SoA observation table; stereo fields compiled out for mono. |
| `StereoRig` | `features/stereo_rig.h` | K left/right, `T_rl` (left → right: `p_r = T_rl p_l`), baseline, distortion models. |
| `BinocularMeasurement<T, L>` | `features/binocular_measurement.h` | Left/right `MonocularMeasurement`, copy of its `StereoRig`, stereo matches (`SetStereoMatches`; per-feature depth, right index); `timestamp()` = left's. |
| `Event`, `EventType` | `events/event_types.h` | Base event: type tag + `recycle_` / `set_ref_count_` fn pointers. |
| concrete events | `events/<event_name>.h` | One header per event (e.g. `events/key_frame_created.h`); `events/events.h` includes all. See §6. |
| `Observable` | `events/observable.h` | Sync handlers (inline, fn ptr + ctx) run first, then async enqueue with ref count = #async queues. |
| `AsyncObserver` | `events/async_observer.h` | One queue per observer (1:1 channel). |
| `Listener` | `events/listener.h` | Iterates a queue; auto-recycles previous/last event; move-only. |

Ported from slamlab2: `images/`, `io/`, `algebra/`, `geometry/`, `features/`
(ORB + SIFT extractors, brute-force matcher, Kannala-Brandt distortion), `logging/`.
The distortion models now live in `geometry/distortion_models/` (moved from
`features/` on 2026-10-05; forwarding headers remain at the old path).

## 6. Events

| Event | Producer → Consumer | Payload |
|---|---|---|
| `FrameTracked<FE>` | Tracker → Mapper, viewer | every tracked measurement: timestamp, pose (`T_cw` after `OptimizePose`), measurement copy (binocular: stereo-matched), inlier pairs `observed_features[i]` -> `observed_points[i]`, `visible_points` (local points passing the projection gate, matched ones included), `reference_frame_id` (optional), `local_frames`, `stage` (`MotionModel` / `ZeroMotion` / `LocalMap`), `num_matches`, `num_inliers` |
| `KeyFrameCreated<FE>` | Mapper → all | **one Frame insertion as one transaction**: the Frame (frame_id, pose `T_cw`, measurement, its observations `observed_features[i]` -> `observed_points[i]`); new points (`new_points_start_id`, contiguous ids, `new_positions` / `new_descriptors` / `new_normals`); `added_observations` on existing Frames (`frame_id`, feature, point; stereo Frames also depth and right index, -1 when absent); updated points (`updated_ids`, positions, descriptors, normals). Used by initialization too |
| `MapPointsUpdated<FE>` | Mapper → all | ids, positions, descriptors, normals (`Resize(n)`, `count()`); changes that create no Frame (sub-project 3) |
| `MapPointsRemoved` | Mapper → all | ids (sub-project 3) |
| `KeyFramePoseUpdated` | Mapper → all | frame_id, pose |
| `LoopDetected` | LoopCloser → Mapper | query/match frames + verification data (TBD) |
| `InitializationMeasurement<FE>` | Tracker → Mapper (+ other Tracker observers) | `features::TMeasurement<FE>` (`map_id` 0) |
| `MapCreated` | Mapper → all | frame_id, pose of the keyframe the Tracker continues from; marks the initial map complete |
| `TrackingLost` | Tracker → all | timestamp (`map_id` = the lost map) |

Rules:
- Events are coarse-grained (one event per batch, Eigen matrices inside).
- Each concrete event type has its own `ObjectPool`.
- Consumers never call `Recycle()` — the `Listener` does.
- Per channel delivery is FIFO, so a replica applying diffs in order always ends
  up identical to the Mapper's map as of the last applied event.
- Because IDs are never reused, a `KeyFrameCreated` allocates its new points a
  contiguous id range `[new_points_start_id, new_points_start_id + count)` and
  every replica stores point `i` at the same slot.
- **Atomic application:** a consumer applies a `KeyFrameCreated` in one step,
  in the order new points, updated points, observations on existing Frames,
  the Frame — the Tracker never tracks against a half-applied insertion.
- **Pointer rule:** an event may point only at data that never changes and lives
  for the whole run (calibration and configuration, e.g. a `DistortionModel` or
  `StereoRig`). Measurements and every `geometry::Camera` copy hold the
  app-owned `DistortionModel` pointer, so the model must outlive every
  measurement, event and `Camera` copy (the app's calibration objects live for
  the whole run). A serializer turns such references into values (inline, or sent
  once and referenced). Everything mutable or short-lived travels by value —
  e.g. a pointer to a reused Tracker buffer is never allowed.
- **Measurement variant (resolves OQ 24).** `features/measurement.h` defines
  `TMonocularMeasurement<FE>`, `TBinocularMeasurement<FE>` and
  `TMeasurement<FE> = std::variant<mono, binocular>`. `KeyFrameCreated`,
  `FrameTracked` and `InitializationMeasurement` hold a
  `TMeasurement<FE>`, so mono and stereo share one event type each.
- **`map_id`.** `map::MapId` (`map/map_id.h`, strong `uint32_t`; ids start at
  1, only increase, `MapId{0}` = not tied to a map). `Event` has a public
  `map_id` field that **producers must set on every publish** (pooled events
  are reused without reset). It is the map the Mapper is building (diffs,
  `MapCreated`), the map the Tracker tracks against (`FrameTracked`,
  `TrackingLost`) or 0 (`InitializationMeasurement`).
  **Consumer rule** (`map_id == 0` exempt): `< current` -> stale, acknowledge
  and ignore; `> current` -> first event of a newer map: reset the replica,
  adopt `current = map_id`, apply (the initial map's `KeyFrameCreated`
  events arrive *before* its `MapCreated`, so they must not be dropped); `== current`
  -> apply. The Mapper builds map n+1 only after `TrackingLost(n)`, so a newer
  map never appears while the Tracker tracks an older one.
- **No subscription filtering:** an `Observable` delivers every event to every
  connected observer (the viewer also receives `InitializationMeasurement` and
  just counts it). A future serializing observer chooses what to send.

### 6.1 Event Lifecycle: Pooled, Reference-Counted, Acknowledged

**Goal:** publishing an event must not hit the general-purpose heap allocator —
neither for the event object nor for its payload. Events are produced at frame
rate on every thread; `new`/`delete` per event means allocator contention
between threads and unpredictable latency. Instead, every concrete event type
has a pre-allocated **`ObjectPool`**, each event is a **`Recyclable`** object
that knows how to return itself to its pool, and pooled objects are **kept
alive** between uses so their payload buffers are reused (steady state: zero
allocations per event).

**The building blocks (`include/voyahslam/memory/`):**

- `ObjectPool<T, SyncPolicy>` — one contiguous slab of `max_count` slots
  allocated once in the constructor. Free slots form a stack of slot indices
  whose links live in a separate `atomic<uint32_t>` array, so slot memory is
  never read by the allocator and is not touched (or committed by the OS) until
  first use. `Create()` pops a slot (asserts when exhausted); `TryCreate()`
  returns `nullptr` instead. The first time a slot is used its `T` is
  default-constructed; on every later reuse the pool resets the ref count to 1
  and calls `T::Reset()` if `T` defines one. `Recycle(T*)` just pushes the slot
  back — **the object is not destroyed**, so vectors and Eigen buffers inside it
  keep their allocations. Objects are destroyed only in `~ObjectPool`. All O(1).
- `ColumnBuffer<TScalar, Rows>` — grow-only `Rows × N` Eigen buffer used for
  event payloads. `Resize(n)` sets the visible column count and reallocates
  (geometrically, preserving data) only when `n` exceeds capacity; `matrix()`
  returns the `leftCols(n)` view. Plain `Eigen::Matrix` can't be used here
  because it reallocates on every size change.
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
5. A created event may be a reused one. Events with a `Reset()` come back
   cleared (counts 0, ids empty, capacity kept). Events **without** `Reset()`
   (`KeyFramePoseUpdated`, `LoopDetected`, `MapCreated`, `TrackingLost`,
   `InitializationMeasurement`) still hold the previous values: the producer
   must set every field. `FrameTracked` and `KeyFrameCreated` have `Reset()`.
6. Fill batched payloads with `e->Resize(n)` and then write into
   `e->positions.matrix()` / `.col(i)` etc. — never assign a freshly built
   `Eigen::Matrix` to a buffer's storage, that would throw away the reuse.

**Adding a new event type:** inherit `Event` and
`memory::Recyclable<Self, memory::ThreadSafePolicy>`, wire `recycle_` /
`set_ref_count_` in the constructor (see e.g. `events/frame_tracked.h`), make it default
constructible, store variable-size payloads in `ColumnBuffer`s / `std::vector`s,
and give it `Reset()` if it has any. Put it in its own header
`events/<snake_case_name>.h`, add the `EventType` value, and add it to
`events/events.h`.

## Example Applications (`examples/`)

Configuration lives in the user application (§9 Q18), so `examples/` holds the
reference applications:

| Target | Kind | Purpose |
|---|---|---|
| `voyahslam_dataset_reader` | static lib (`examples/dataset_reader/`) | `EurocDatasetReader`, `PlaybackClock` |
| `monocular_full_slam` | executable (`examples/full_slam/`) | cam0 → `MonocularMeasurement` → `Tracker::Track()` |
| `binocular_full_slam` | executable (`examples/full_slam/`) | cam0+cam1 → `BinocularMeasurement` → `Tracker::Track()` |
| `dataset_reader_tests` | gtest | Synthetic-dataset tests for the reader and clock |

Usage: `<app> <dataset_root> [--rate=R] [--max-frames=N]` (rate 1 = real
time, 0 = as fast as possible). Both apps print lateness and load/extract
timings.

**`EurocDatasetReader`** reads the EuRoC layout (`mav0/cam{0,1}/data.csv`,
`data/<ns>.png`) used by EuRoC MAV and TUM-VI. Calibration: Kalibr
`dso/camchain.yaml` (TUM-VI) if present, else `mav0/camN/sensor.yaml` (EuRoC;
the OpenCV `%YAML:1.0` directive is stripped for yaml-cpp). It owns the
distortion models and builds the `StereoRig` from both cameras' `T_cam_imu`
(`T_rl = T_right_imu · T_left_imu⁻¹` with `T_cam_imu` mapping IMU to camera, so `p_right = T_rl p_left`: `StereoRig::T_rl` maps left → right).
Images: decoded to 8 bit (16-bit PNGs reduced) and returned as
`images::TWorkingImage` (8-bit grayscale).
Only `equidistant` (Kannala-Brandt) and `none` distortion are supported.

**Timing — no reader thread.** The reader is synchronous and index-based. The
app's main loop paces itself with `PlaybackClock::WaitUntil(timestamp)`:
absolute deadlines (`anchor + (t - t0) / rate`), so per-frame work doesn't
accumulate drift as long as it fits in the frame gap; if it doesn't, the clock
returns the lateness rather than drifting silently. Measured on TUM-VI
corridor1 (Release, 1024², 1500 ORB features): load ≈ 10 ms, ORB extraction
≈ 220 ms per image — so a separate reader thread would buy nothing; the
extractor is the bottleneck (Open Question 21).

**Reference dataset:** TUM-VI `dataset-corridor1_1024_16` (20 Hz stereo,
1024×1024 16-bit, equidistant fisheye, baseline 0.101 m).

## Visualization Library (`vizualization/`, `voyahviz`)

Development tool, never part of the production build. Implemented with
OpenCV (HighGUI, imgproc) and Pangolin (3D map viewer), but — like every
library of ours — both are private: the public API uses only voyahslam types
and OpenCV and Pangolin are linked `PRIVATE` (principle 8). Namespace / include prefix: `voyahviz`.

| Header | Contents |
|---|---|
| `color_image.h` | `TColorImage` = `images::Image<uint8_t, 3>` (interleaved B, G, R); `Color` (RGB); `ToColor(TImageU8Gray)`; `HorizontalConcat(left, right)` |
| `keypoint_drawing.h` | `KeypointStyle`; `DrawKeypoints(TColorImage& out_canvas, points, angles*)` and a gray-image overload returning a new canvas; `DrawFeatures(image, ImageFeatures)` for raw extractor output; `ProjectToImage(measurement, distortion)` / `DrawMeasurement(...)` |
| `image_window.h` | `ImageWindow`: named window, `Show(TImageU8Gray / TColorImage)`, `WaitKey()`, `GuiBackend()`; throws if no GUI backend |

Internally `src/opencv_wrapping.h` wraps our images as `cv::Mat` without
copying (channels interleaved, stride in pixels).

**Pitfall:** `MonocularMeasurement::points()` are *undistorted* pixel
coordinates — on a fisheye they don't line up with the raw image.
`DrawFeatures()` is deleted for measurements; use `DrawMeasurement()`, which
maps features back via `K · Distort(normalized)`. Since 2026-10-05 measurements
also keep the raw keypoints (`distorted_points()`), which line up with the raw
image directly.

**Threading:** HighGUI isn't thread-safe; create the window and call
`Show()`/`WaitKey()` from one thread (the apps' main thread). A future live
viewer fed by Tracker/Mapper events must marshal frames to that thread.

`monocular_full_slam` / `binocular_full_slam --show` display the live frame(s)
with keypoints (stereo side by side); `q`/`Esc` quits.

### Map Viewer

Spec: `docs/superpowers/specs/2026-10-03-map-viewer-design.md`.

| Header | Contents |
|---|---|
| `scene_state.h` | `SceneState<FE>`: `Apply(const Event&)`, per-`EventType` counters, and the decoded scene: map points (dense, indexed by per-map point id), keyframe poses (`T_cw`), current camera pose, trajectory, and from the latest `FrameTracked` the tracked and visible point ids and the local Frames. Built only from events; no threads/GUI |
| `camera_frustum.h` | Camera frustum geometry helper (corner points of a frustum of a given size) |
| `map_viewer.h` | `MapViewer<FE>(MapViewerOptions{fps, show_window, width, height, camera_size})`: one `AsyncObserver` inbox, `Start()` / `Stop()` / `RunOnCurrentThread()` / `RequestStop()`, `inbox()`, `events_processed()`, `window_closed()`, `WaitForWindowClose()` |
| `pangolin_renderer.h` (private, `src/`) | Pangolin window: points (tracked green, visible blue, others grey), keyframe frustums (local-map Frames orange, others blue), current camera (green, red when lost), trajectory, control panel (toggles, map id, counts, status) |

Specs: `docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md`
(Pangolin viewer), `docs/superpowers/specs/2026-10-03-map-viewer-design.md`.

- **`SceneState` is the scene model.** It is rebuilt purely from events and
  follows the multi-map rule: it shows the newest map seen; events of older
  maps are ignored, an event of a newer map clears the scene and adopts it;
  events with map 0 are counted only.
- **Loop-thread invariant:** only the thread running the loop touches the
  `SceneState`, the renderer and the GL context. The window is created at loop
  entry and destroyed at loop exit (or on close), on that thread. Other threads
  only use the atomics (`window_closed()`, `events_processed()`) and `inbox()`.
- Loop: drain inbox through a local `Listener` (acknowledges each event) ->
  `SceneState::Apply` -> render (if the window is open) -> sleep to the next
  absolute deadline (no catch-up when late). No busy-waiting.
- **`Start()` vs `RunOnCurrentThread()`:** `Start()` runs the loop on the
  viewer's own thread (Linux). `RunOnCurrentThread()` runs it on the caller's
  thread, because on macOS the GUI (Pangolin/Cocoa) must live on the main
  thread; the app would then run playback on a worker thread. The apps use
  `Start()` today; the macOS setup is in TODO.
- **Stop semantics:** `Stop()` in `Start()` mode joins and drains once more so
  queued events return to their pools; idempotent, safe without `Start()`,
  restart allowed, destructor stops. During `RunOnCurrentThread()`, `Stop()`
  only requests the stop and the loop drains itself on its own thread; other
  threads end that run with `RequestStop()` only (a `Stop()` before the loop is
  entered would consume the request and a later `RunOnCurrentThread()` would
  block), and `Stop()` / the destructor belong on the loop's thread after it
  returns. `Start()`/`Stop()` must be called from one controlling thread
  (externally ordered). A
  `RequestStop()` made before the loop is entered is kept (the run drains once
  and returns). `fps <= 0` throws.
- **Window close does not end the loop.** It keeps draining headless (so the
  producers' event pools never stall); only `RequestStop()` / `Stop()` end it.
  `window_closed()` is also true if the window could not be created (logged,
  runs headless). Apps stop playback when it turns true and, after the last
  frame, wait with `WaitForWindowClose()`.
- **Wayland:** on Linux under Wayland with `PANGOLIN_WINDOW_URI` unset, the
  `MapViewer` constructor sets it to `x11://` (XWayland): Pangolin's Wayland
  backend blocks in `eglSwapBuffers` on GNOME, which would stall the drain and
  the producers' pools. Construct the viewer before other threads start.
- **Ordering:** connect with `AddObserver(viewer.inbox())` before the producers
  start; stop `Slam` before the viewer.
- Pools feeding the viewer must cover the backlog published between two ticks
  (about 33 ms at 30 fps) - relevant to OQ 14.
- **Pangolin is a private dependency:** linked `PRIVATE` via `pango_display`
  (installed in `/usr/local`, 0.9); no Pangolin types in public headers
  (principle 8).
- The full_slam apps always attach a `MapViewer` (headless by default) and
  print `viewer: N events processed`; `--viewer` opens the window (also stops
  playback when the window is closed and, at the end of playback, waits for it
  to be closed).
- Verified with ThreadSanitizer (`MapViewer*:SceneState*`, 2026-10-04: clean).

## Geometry Toolkit (`estimators/`, `geometry/`)

Spec: `docs/superpowers/specs/2026-10-04-two-view-geometry-design.md` (part A of
the real monocular initializer; matching and `MapInitializer` integration are
spec B+C, now done: see "Monocular Initialization"). Pure math on
**normalized, undistorted points** (z = 1), so the calibrated model is the essential matrix E, not F + K. Sigma is in normalized
units (σ_px / f). Verified on synthetic scenes only. This is the "rays stay
rays" half of the residual-space rule (see "Camera Model and Raw-Pixel
Residuals"): two-view RANSAC, triangulation and the 85° ray test work on rays;
every reprojection residual and gate after them is in raw pixels.

| Component | Header | Summary |
|---|---|---|
| `Camera` | `geometry/camera.h` | K + `const DistortionModel*` (null = pinhole); `Project` (camera point → raw pixel), `ProjectJacobian` (2×3), `Unproject` (raw pixel → z = 1 ray). See "Camera Model and Raw-Pixel Residuals". |
| `DistortionModel` (+ Dummy, Kannala-Brandt) | `geometry/distortion_models/*.h` | `Distort` / `Undistort` on normalized points, analytic `DistortJacobian`. Forwarding headers at `features/distortion_models/`. |
| `RigidTransformation` | `geometry/rigid_transformation.h` | Value type, hidden representation (R, t now). `T_ab` maps frame b to frame a: `p_a = T_ab * p_b`, `T_ac = T_ab * T_bc`. `Inverse()`, `Transform()`, `ToMatrix34/44()`. |
| `TwoViewCorrespondences` | `geometry/two_view_correspondences.h` | `x1`, `x2` (2xN normalized) + `sigma`. |
| `TwoViewEstimator` concept, `ModelScore` | `estimators/two_view_estimator.h` | `TModel`, `kSampleSize`, `Estimate(corr, sample)` -> 0..n candidates, `Score(model, corr)` -> score + inlier mask. |
| `HomographyEstimator` | `estimators/homography_estimator.h` | 4-point DLT (Hartley-normalized), `H_21`; score = ORB-SLAM3 `CheckHomography` (symmetric transfer, χ² < 5.991). |
| `EssentialEstimator` | `estimators/essential_estimator.h` | Normalized 8-point + projection to the essential manifold, `E_21`; score = ORB-SLAM3 `CheckFundamental` (χ² < 3.841 per direction, adds 5.991 - χ²). |
| `Ransac<TEstimator>` | `estimators/ransac.h` | Generic, header-only, deterministic (own `mt19937` from `seed`). |
| `DecomposeEssential`, `DecomposeHomography` | `geometry/two_view_decomposition.h` | 4 (R, ±t) poses from E; up to 8 Faugeras candidates (+ plane normal) from H. `‖t‖ = 1`, `det R = +1`. |
| `SelectPose` | `geometry/two_view_reconstruction.h` | ORB-SLAM3 `CheckRT` + acceptance rules. |
| `TwoViewReconstructor` | `geometry/two_view_reconstruction.h` | Fits H and E, picks the model, decomposes, selects the pose. |

**Estimator contract.** `Estimate` and `Score` are required; scores of all
estimators are on one scale (truncated χ², σ in normalized units). An estimator
may also provide `Refine(model, corr, inliers)`; `Ransac` detects it with a
`requires` expression (optional, not part of the concept). Both estimators
implement it: Levenberg-Marquardt over the inliers (E: R update + t on the unit
sphere, Sampson error; H: 8 parameters, symmetric transfer error); at most 20
accepted steps / 40 trials, early stop at relative decrease < 1e-6; a non-finite
or ill-conditioned result returns the input model.

**Ransac.** Per iteration: sample `kSampleSize` distinct indices (partial
Fisher-Yates), score every candidate, keep the best. Adaptive stop from the
inlier ratio (`confidence`, clamped to `[min_iterations, max_iterations]`).
Afterwards up to `refinement_rounds` (5) of local refinement: refine on the
current inliers, re-score, keep only strict improvements. `success = false` for
N < `kSampleSize` or no positive score.

**SelectPose / CheckRT.** For every inlier, triangulate with `P1 = [I|0]`,
`P2 = T_21`; skip non-finite points, points behind a camera (unless the point
has almost no parallax) and points with reprojection error > 4σ² in either view.
Accept the candidate with the most good points if it has
`> max(0.9 x inliers, 50)` good points, no other candidate has more than
`uniqueness x best` (E: 0.7, H: 0.75) and parallax >= 1°, the parallax being
the angle at the 51st-smallest cosine, i.e. the 51st-largest parallax angle
(index `min(50, n - 1)`, as ORB-SLAM3). Far points count as good but are not returned as
triangulated.

**Model selection (COLMAP rule).** `Reconstruct` runs `Ransac` for H and for E
(seeds `seed`, `seed + 1`) and picks H iff `inliers_H >= 0.8 x inliers_E`
(a homography explains nearly everything E does: planar scene or near-pure
rotation); otherwise E. ORB-SLAM3's score ratio `S_H / (S_H + S_E) > 0.4` does
not transfer: it relies on the *uncalibrated* F degenerating on planes, whereas
a calibrated E fits planar points as well as general ones (measured: even the
ground-truth H vs E gives a ratio of about 0.41). Both scores are still
reported. A non-positive sigma fails the reconstruction. The default narrow,
near-frontal planar scene is rejected as ambiguous by design.

**Measured accuracy limits.** On a wide-FOV general scene the
translation-direction error is up to 9.9° (about half of 21 seeds under 2°);
rotation < 1°. The cause is inlier-set lock-in, not LM convergence: LM reaches
the same answer from the true E and from the returned E on the same inlier set,
but that set is self-selected by a hard χ² threshold (3.841) — least squares
over it drops true inliers and keeps 2-3 outliers as levers. On the worst seeds
the true E scores higher than the returned model (2079 vs 1920 at seed 108,
1891 vs 1670 at 113, 2111 vs 1862 at 116); more refinement rounds (30) or a
looser re-collection threshold do not fix it. Only ~58 % of true inliers are flagged at
σ = noise (noise in both views). Planar wide-FOV: 21/21 seeds < 1° / < 2°.
Further accuracy is left to bundle adjustment right after initialization (as in
ORB-SLAM3).

## Camera Model and Raw-Pixel Residuals (`geometry::Camera`)

Spec: `docs/superpowers/specs/2026-10-05-camera-model-design.md` (branch
`feat/camera-model`).

**Why.** Two diagnostic spikes on corridor1 (2026-10-05, branch
`spike/mono-tracking-diag`, not merged) found that mono tracking lost its map
because search windows and noise models lived in *undistorted* pixel or
normalized space, where the fisheye periphery is stretched (median 5.8× at
60–85°). Only 29 % of findable points landed in the 2.5 px window, and at
60–85° `OptimizePose` rejected about half of the correct matches. ORB-SLAM3
projects, searches and measures residuals in the raw image through its camera
model; we now do the same.

**Residual-space rule.** Every reprojection residual and gate compares **raw
(distorted) pixels**, with σ = `sigma_px · 1.2^level` pixels (the extractor's
level scale); the χ² thresholds are unchanged (5.991 for 2 dof, 9.488 for
4 dof). **Rays stay rays:** two-view RANSAC, triangulation and the 85° ray
test work on undistorted normalized points (z = 1) as before.

**`geometry::Camera`** (`geometry/camera.h`, approach A: a small value type,
not camera-model classes and not free functions):
- `K` plus `const distortion_models::DistortionModel* distortion` (null =
  pinhole).
- `Project(Xc)`: camera-frame point → raw pixel, `K · [Distort(Xc.xy / Xc.z); 1]`.
  Precondition `Xc.z() > 0`.
- `ProjectJacobian(Xc)` (2×3): `K₂ₓ₂ · DistortJacobian(Xc.xy / Xc.z) ·
  [[1/z, 0, −x/z²], [0, 1/z, −y/z²]]`; `K₂ₓ₂` includes the skew.
- `Unproject(pixel)`: raw pixel → undistorted normalized ray via K⁻¹ and
  `Undistort`.

**`DistortionModel` in `geometry/`.** The models moved from
`features/distortion_models/` to `include/voyahslam/geometry/distortion_models/`
(namespace `voyahslam::geometry::distortion_models`, sources in
`src/geometry/distortion_models/`, tests in `tests/geometry/distortion_models/`),
so `geometry` stays at the bottom of the dependency stack. The old headers are
forwarding headers (`namespace features { namespace distortion_models =
geometry::distortion_models; }`), so old includes and qualified names still
compile; in-repo users use the new path. New pure virtual `DistortJacobian(p)`
= d(distorted normalized) / d(undistorted normalized):
- Dummy: identity.
- Kannala-Brandt: with r = ‖p‖, θ = atan r, d(θ) = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ +
  k4θ⁸) and p̂ = p / r, **J = (d/r)·I + (d′(θ)·θ′(r) − d/r)·p̂p̂ᵀ**, θ′(r) =
  1 / (1 + r²), d′(θ) = 1 + 3k1θ² + 5k2θ⁴ + 7k3θ⁶ + 9k4θ⁸; J = I for r < 1e-8
  (the limit). Verified against central finite differences (Ruling U: the
  first spec draft had an extra 1/r).

**Measurements keep both** the distortion model pointer and the raw keypoints:
`MonocularMeasurement::distorted_points()` are the keypoints as passed to the
constructor (compacted with every other array when features beyond 85° are
dropped), and `camera()` returns `{K, distortion_model}`. `points()`
(undistorted pixels) and `normalized_points()` are unchanged.
`BinocularMeasurement` uses `left().camera()`, `right().camera()` and
`rig().T_rl`. The pointer rule (§6) covers the model: it must outlive every
measurement, event and `Camera` copy.

**The five consumers** (all in raw pixels):
1. Projection matcher: projects with `current.camera().Project(Xc)`; the
   bounding box and `FeatureGrid` use `distorted_points()`; window values
   unchanged (now raw px).
2. `OptimizePose`: `OptimizePose(initial_T_cw, left Camera,
   optional<StereoCameras{right, T_rl}>, observations, options)`;
   `PoseObservation::left/right` are raw pixels and `sigma` is in pixels. Mono
   e = `left.Project(T_cw·X) − z`, J = `left.ProjectJacobian(Xc)·[−[Xc]× | I]`;
   stereo rows through `right.Project(T_rl·Xc)`. A `right` observation without
   `stereo` → `success = false`. The helpers in `src/optimization/projection.h`
   (`ProjectPixel`, `PixelJacobians`, `ProjectPixelRight`,
   `PixelJacobianRightPose`) take a `Camera`; the normalized-plane helpers were
   removed.
3. `BundleAdjust`: `BaProblem::keyframe_cameras` (one per keyframe,
   validated); `BaObservation::measurement` in raw pixels, `sigma` in pixels;
   point block `ProjectJacobian · R`. Two-stage logic and rulings N, O, O′
   unchanged. `MapInitializer` builds observations from `distorted_points()`
   and each view's `camera()`.
4. Stereo matcher gate: triangulation stays on rays; the gate projects the
   triangulated point with each image's `camera()` and compares it with
   `distorted_points()` (`PixelChi2`, σ = `sigma_px · level_scale^level`).
5. Initialization matcher: grid on the current frame's `distorted_points()`;
   `last_matched` starts as the reference's `distorted_points()` and is updated
   in raw pixels; the 100 px window is ORB-SLAM3's raw-pixel value.

The Tracker builds `PoseObservation`s from `distorted_points()` and passes
`left().camera()` (plus `StereoCameras{right().camera(), rig().T_rl}` for
stereo); `TrackerOptions::sigma_px` keeps its meaning, now in raw pixels.

**Pinhole invariance (Ruling V).** With a null distortion every consumer's
χ², inlier decisions and results equal the old normalized code (1e-9) **only
for fx = fy**: the old code divided σ by a single f, while `OptimizePose`,
`BundleAdjust` and the stereo gate now weight x and y separately (σ in pixels
on each axis). On fx ≠ fy rigs (e.g. EuRoC) χ² changes slightly; this is the
intended behaviour.

**Spike numbers** (corridor1, mono, 400 frames, tracked frames /
re-initializations; spike code, not this branch):

| Variant | Result |
|---|---|
| Baseline (undistorted windows and σ) | 32 / 7 |
| Window and σ scaled by the Kannala-Brandt stretch | 53 / 5 |
| …plus local-map gate 20 | 108 / 5 |
| Angle-only allowance σ·1.5·(1 + r²), window ×(1 + r²) | 85 tracked |
| …plus local-map gate 20 | 99 tracked |
| Uniform σ 3 | effectively disables outlier rejection |

The calibration-free angle allowance (within 0.6 % of the Kannala-Brandt
stretch up to 60°) is recorded as a fallback for lenses without a model; it is
not implemented (TODO).

**Smoke runs on this branch** (2026-10-06, Release, `dataset-corridor1_1024_16`,
ORB, `--rate=0`, apps at debug level):

| Run | Result |
|---|---|
| Mono, 400 frames (default gate 30) | **82 tracked / 18 re-initializations** (19 maps; per map 0–14, median 4); median local-map inliers 41 (30–61); 16 losses at the local-map gate (23–29 inliers), 3 in the zero-motion step. Repeat run identical. |
| Mono, 400 frames, local-map gate 20 (temporary, measurement only) | 137 tracked / 14 re-initializations (15 maps) |
| Mono initialization | first map at about frame 118 with 71 points; the 19 maps have 62–109 points |
| Stereo, 200 frames | initialized on frame 1 with 358 points; 197 local-map successes, no loss, 1 map; median 232 inliers (68–269), was 189 |

Against the success criterion (≥ 53 tracked and ≤ 5 re-initializations) mono
passes on tracked frames and **misses on re-initializations**. The cause is
initialization acceptance, not shorter-lived maps: the initialization
matcher's 100 px window is now in raw pixels, which is effectively wider in the
fisheye periphery, so the reference frame is reset about half as often (94 vs
181 resets) and many more pairs initialize (19 maps vs 6; 11 of them from
adjacent frames, tracked for about 2.5 frames on average). Every accepted
init still passes the unchanged 1° parallax gate (1.02–2.34°). Maps from
comparable inits live as long as before; the 82 / 19 ≈ 4.3 frames per map
average (baseline 32 / 8 = 4.0, spike KB 53 / 6 ≈ 8.8) is diluted by the extra
short inits. Causal check (final review, scratch worktree): reverting only the
init matcher to undistorted pixels gives 6 maps / 56 tracked / 9.3 frames per
map, matching the spike. The branch keeps the raw-pixel init matcher: more
tracked frames (82 vs 56) at the cost of more, mostly short, maps. Every map
still dies at the 30-inlier local-map gate because nothing grows the
62–109-point initial map (sub-project 2). Three adjacent-frame maps
(≈ frames 311–318) died on their first tracked frame with 0–1 pose inliers out
of 39–47 matches; this is not diagnosed (TODO). No parameter was tuned (gate,
init acceptance and init-map tuning are in TODO).

## Optimization (`optimization/`, g2o behind Eigen types)

Spec: `docs/superpowers/specs/2026-10-05-init-bundle-adjustment-design.md`
(amended during implementation; the decisions log has the changes).

`optimization::BundleAdjust(BaProblem, BundleAdjustmentOptions) -> BaResult`
(`include/voyahslam/optimization/bundle_adjustment.h`) jointly refines keyframe
poses (`T_cw`, some may be fixed) and world points from **raw (distorted)
pixel observations**, each projected through its keyframe's `geometry::Camera`
(`BaProblem::keyframe_cameras`, one per keyframe; per-observation sigma in
pixels; normalized-plane residuals until 2026-10-05). The public
header uses only our types and Eigen; g2o is linked `PRIVATE` and never appears
in `include/` (principle 8 applies to g2o as well). Internals: `src/optimization/`
(`projection.h` has the pixel projection helpers and analytic Jacobians through
`Camera::ProjectJacobian`; the g2o
edge/vertex glue lives in `bundle_adjustment.cpp`). Levenberg-Marquardt with
`BlockSolver_6_3` + `LinearSolverEigen`, points marginalized.

**Two stages like ORB-SLAM3 LocalBA:** `robust_iterations` (5) with the Huber
kernel (delta^2 = `huber_chi2` = 5.991), then edges with chi2 > `huber_chi2` or
camera-frame depth <= 0 are deactivated and `iterations` (default 10) plain
iterations run. `BaResult` has per-observation `chi2` and `inlier` (for all
observations, deactivated ones included), initial/final cost and the iteration
count. Invalid input gives `success == false` without optimizing.

**Vendored g2o.** `third_party/g2o/src/` is pristine upstream (tag
`20241228_git`, commit `eec325a1`); the local Conan recipe
`third_party/g2o/conanfile.py` (reference `g2o/20241228@voyah/stable`, user/channel
set in the recipe) builds only the BSD-2-Clause core (`stuff`, `core`,
`types_slam3d`, `types_sba`, `solver_eigen`) and exposes them as components
(`g2o::core`, ...). The one-time `conan create` is part of the README build steps.
Details: `third_party/README.md`.

## Monocular Initialization (Mapper, `MapInitializer`, `InitializationMatcher`)

Spec: `docs/superpowers/specs/2026-10-04-mono-initialization-design.md`.
Verified on synthetic sequences; the app smoke run is recorded in `docs/TODO.md`.

**Pipeline** (ORB-SLAM3 `MonocularInitialization`). The Mapper feeds each
`InitializationMeasurement` to `MapInitializer::TryInitialize`:
1. A frame with more than `min_features` features becomes the **reference**
   (`last_matched` = its raw keypoints, `distorted_points()`).
2. Each later frame: `FeatureGrid` over the current frame's raw keypoints, then
   `MatchForInitialization` (ORB-SLAM3 `SearchForInitialization`: window around
   the last matched position (100 px, raw pixels), level restriction, best/second-best ratio,
   one-to-one with replacement, orientation histogram keeping the three largest
   bins). Fewer than `min_matches` matches: the current frame becomes the
   reference.
3. `TwoViewReconstructor` on the normalized matched points. On failure the
   reference **and** the matcher's updated `last_matched` positions are kept
   (ORB-SLAM3 `mvbPrevMatched`). Zero triangulated points or a non-finite /
   non-positive median depth also count as failure.
4. **Bundle adjustment** (`bundle_adjust`, default on): a two-keyframe
   `BundleAdjust` on the *unscaled* reconstruction (keyframe 1 fixed at identity,
   keyframe 2 and the points free; observations are `distorted_points()` through
   each view's `camera()`, sigma = `sigma_px * level_scale^level` pixels per
   observation). 5 + 40 iterations (baseline 1, depths ~40, so the plain stage
   needs ~40 iterations; LM stops at convergence). Points with any outlier
   observation are dropped; fewer than `min_points_after_ba` (50) survivors, or an
   unsuccessful adjustment, fail the attempt (reference kept, like any failure).
5. **Scaling:** positions and the translation of `T_21` are divided by the median
   depth of the surviving points in camera 1, so the median depth is 1.
6. **Publication order** (the Mapper sets state and map id first): a
   `KeyFrameCreated` for keyframe 1 carrying every initial point (ids from 0)
   and keyframe 1's observations, a `KeyFrameCreated` for keyframe 2 with its
   observations, then `MapCreated` (since 2026-10-06; before, a separate
   `MapPointsAdded` came first). Replicas therefore have the points before the
   keyframes that observe them.

**Conventions.** Keyframe/frame poses are `T_cw` (world -> camera,
`RigidTransformation`); the world frame is camera 1 (first keyframe =
identity). Observations travel on `KeyFrameCreated` as parallel
`observed_features` / `observed_points` vectors. Map point ids restart at 0
**per map**; `(map_id, MapPointId)` identifies a point. Frame ids stay
monotonic across maps.

**Keypoint levels.** `ImageFeatures` stores a pyramid level per keypoint
(`levels()`); each extractor defines `ScaleFactor(level)`: ORB `scale_factor^level`
(1.2), SIFT `2^(level/3)/2` with the level decoded from OpenCV's packed
octave/layer. The matcher restricts candidates to the reference level
(`max_reference_level`, ORB default 0).
SIFT keypoints are sorted by response and truncated to `number_of_features`,
and **SIFT angles are converted to radians** (they were degrees, and columns
were left uninitialized when over-detecting).

## Stereo Initialization (Tracker stereo matching, stereo `MapInitializer`)

Spec: `docs/superpowers/specs/2026-10-04-stereo-initialization-design.md`.
Verified on synthetic stereo frames and a TUM-VI smoke run (see `docs/TODO.md`).

**Pipeline.**
1. The Tracker thread, on dequeuing a `BinocularMeasurement`, calls
   `features::MatchStereo` (options in `TrackerOptions::stereo_matcher`) and
   stores the result with `SetStereoMatches` before the measurement is tracked
   or forwarded as `InitializationMeasurement`. Mono measurements are
   untouched.
2. `MatchStereo` is modelled on ORB-SLAM3 `ComputeStereoFishEyeMatches`:
   brute-force descriptor matching (best < 0.7 x second best), one-to-one (the
   closer claim wins), DLT triangulation of the undistorted normalized points,
   then checks: positive depth in both cameras, per-image reprojection
   chi-square (sigma scaled by pyramid level), and a parallax-cosine limit
   (`max_parallax_cos` 0.99998). The reprojection test projects the point with
   each image's `camera()` and compares raw pixels with `distorted_points()`, as
   ORB-SLAM3 does (undistorted normalized coordinates until 2026-10-05). A rectified row search does not apply to
   fisheye images; epipolar-guided search is a deferred optimization.
3. `StereoMatches` (`right_indices`, `depths`, `num_matches`) lives in its own
   header (`features/stereo_matches.h`) so that `BinocularMeasurement` and the
   matcher do not include each other.
4. Stereo `MapInitializer` (ORB-SLAM3): needs more than
   `stereo_min_features` (500) left features; every left feature with a valid
   depth becomes a map point at **metric scale** (`depth * normalized_point`,
   no median normalization); at least `stereo_min_points` (100) required. The
   result is **one** keyframe with an identity `T_cw`.
   `InitialMapPoint::feature_in_second == -1` is the sentinel for maps with a
   single keyframe. A binocular measurement drops any mono reference.
5. The Mapper publishes one `KeyFrameCreated` (the keyframe with every
   initial point) -> `MapCreated`. Point descriptors come from the left image;
   the normal is the viewing direction of the single keyframe.

**`StereoRig::T_rl`.** A `RigidTransformation` mapping left-camera coordinates
to right-camera coordinates (`p_r = T_rl p_l`); `baseline = ‖T_rl.translation()‖`.
It replaces `R_rl`/`t_rl`; the dataset reader previously computed the inverse.
Test rigs put the right camera at +x, so the translation is `(-b, 0, 0)`.

**Observation storage.** Each map keeps a mono `ObservationStorage` and a
stereo one; stereo keyframes store `depth` and `right_feature_index`
(`int16_t`) there. Both are cleared on a new map and on `TrackingLost`. Keeping
both allows mixed mono/stereo sessions later (TODO).

**Depth limit.** `max_parallax_cos = 0.99998` rejects rays closer than about
0.36 degrees; with the 0.11 m baseline this caps stereo depth at about 17 m.
ORB-SLAM3's `KannalaBrandt8::TriangulateMatches` is stricter: it rejects
`cosParallaxRays > 0.9998` (about 1.15 degrees, a cap of about 5.5 m at
0.11 m), so our default is more permissive; tuning comes last (TODO).

**Smoke run** (TUM-VI `dataset-corridor1_1024_16`, Release, ORB):
initialized on the first frame with 355 points; median point depth 1.96 m
(quartiles 1.50 / 2.13 m, range 0.25-9.50 m).

## Tracking Against the Map (Tracker, `MapReplica`, `ProjectionMatcher`, `OptimizePose`)

Spec: `docs/superpowers/specs/2026-10-05-tracking-design.md` (sub-project 1 of
4; keyframe creation / map growth is sub-project 2, see "Keyframes and Map
Growth"; local BA and culling, relocalization are sub-projects 3-4, deferred;
the user has their own designs for them).

**Replica.** `tracking::MapReplica<FE>` (`tracking/map_replica.h`) is the
Tracker's private copy of the map, touched only on the Tracker thread and fed
by the Mapper's events with the `map_id` rule (0 exempt, older ignored, newer
clears and adopts). It stores point position/descriptor/normal/alive,
keyframes (`T_cw`, measurement copy, observations) and, per point, the first
keyframe observation as a `map::PointReference` (keyframe, feature, pyramid
level, distance to the camera centre). It does not count events (decision R).

**Flow per measurement in `Tracking`** (stereo matching has run on dequeue):
1. **Motion model** (when a velocity exists): predict `T_cw = velocity * T_last`,
   `SearchFromLastFrame` with a 15 px (mono) / 7 px (binocular) window scaled
   by `level_scale^level` of the last-frame feature; fewer than 20 matches
   retries with the window x2; `OptimizePose`; the step needs >= 10 inliers.
2. **Zero-motion fallback** when step 1 failed or there is no velocity: same
   with the predicted pose = `T_last` and the window x4 (the BoW-less stand-in
   for ORB-SLAM3's `TrackReferenceKeyFrame`; see limitation below).
3. **Local map** (since 2026-10-06 built from covisibility, see "Keyframes and
   Map Growth"): `SearchFromLocalMap` over the alive, unmatched, visible points
   of the local Frames (`th` = 1), then `OptimizePose` on all matches; success
   needs >= 30 inliers.
4. **Success**: `FrameTracked` (pose, inlier pairs, visible points, reference
   Frame, local Frames, stage, counts), `velocity_ = T_cur * T_last^-1`,
   `last_frame_` = this frame with its inlier matches; info log at most 1 Hz
   ("Tracking: frame N, M inliers"), debug log per step. **Failure**:
   `TrackingLost`; the debug log names the step and counts.

**Thresholds** (`TrackingOptions`): window 15 px mono / 7 px stereo; motion
model needs 20 matches (else x2 retry) and 10 inliers; zero-motion window x4;
local-map `th` 1.0; 30 local-map inliers.

**Projection matching** (`features/projection_matcher.h`, header-only
template on the extractor; raw (distorted) pixel space: projection through
`current.camera()`, grid and bounding box on `distorted_points()`). `SearchFromLastFrame`:
project the last frame's matched points, candidates within the level-scaled
window on levels [l-1, l+1], best descriptor <= 100, one-to-one claims
(closer wins), rotation-consistency histogram (12 bins, three maxima).
`SearchFromLocalMap`: visibility (z > 0, within 85 deg, view/normal cosine >
0.5), predicted level from the reference observation, radius `th * (cos > 0.998
? 2.5 : 4.0) * level_scale^l`, levels [l-1, l], ratio test 0.8 unless the two
best are on different levels. Deviations from ORB-SLAM3: the histogram is built
from the final claims; points without a reference observation are skipped by
the local-map search; there is no scale-invariance distance check (the level is
clamped instead).

**`OptimizePose`** (`optimization/pose_optimization.h`, g2o private). Unary
edges on one `VertexSE3Expmap`: mono (2 rows, raw pixels through the left
`Camera`) and stereo (4 rows: left and right via `StereoCameras{right, T_rl}`,
with `PixelJacobianRightPose`).
Schedule as ORB-SLAM3: 4 rounds x 10 iterations, every round restarts from the
initial pose, Huber kernel in rounds 0-1 and plain after, edges classified per
round by chi2 > 5.991 (mono) / 9.488 (stereo, 4 dof) or depth <= 0; success
needs >= 3 inliers (decision S). `sigma = sigma_px * level_scale^level` pixels.

**BoW-less fallback limitation.** There is no vocabulary / place recognition,
so a lost frame cannot be tracked against the reference keyframe; the wide
zero-motion projection search replaces it. It recovers only small
frame-to-frame motion; real recovery is relocalization (sub-project 4).

**Measured behaviour** (TUM-VI `dataset-corridor1_1024_16`, Release, ORB,
2026-10-05; details in `docs/TODO.md` smoke note): binocular tracks all 200
frames of the smoke run (median 189 inliers). Monocular initializes at about
frame 120 but is lost within 1-11 frames, repeatedly (8 maps in 400 frames):
the initialization map has 67-94 points, so the local map drops below 30
inliers quickly, and without map growth (sub-project 2) there is nothing to
refill it. Mono translation accuracy is also limited by the initial map
(decision T).

After the camera-model change (raw-pixel residuals, 2026-10-06 smoke run; see
"Camera Model and Raw-Pixel Residuals"): binocular still tracks the whole run
(median 232 inliers); monocular tracks 82 frames in 400 with 18
re-initializations (19 maps, 62-109 points each, median 41 local-map inliers).
Maps are still short-lived for the same reason (no map growth). Map growth
arrived with sub-project 2: see "Keyframes and Map Growth" for its smoke run.

## Keyframes and Map Growth (sub-project 2: Mapper decides, `KeyFrameCreated` transaction, covisibility local map)

Spec: `docs/superpowers/specs/2026-10-06-keyframes-design.md` (amended in
place for rulings F, G, H). A `Frame` is what ORB-SLAM3 calls a KeyFrame;
event names keep "KeyFrame".

**Who decides: the Mapper.** The Tracker publishes `FrameTracked` for every
tracked measurement (§6) and decides nothing about keyframes;
`NewKeyFrameCandidate` and `PositionUpdated` are deleted. The Mapper, the only
map writer, has the authoritative observation counts, its own backlog and the
Frames the Tracker's replica has not received yet, so it decides.

**Mapper inbox.** `FrameTracked` is considered only in `Mapping` and only for
the current map. *Drain to latest:* a `FrameTracked` is skipped when a newer
`FrameTracked` **or a `TrackingLost`** of the same map follows it in the
drained batch (ruling I); all other events keep their order. *Backlog* = the
`FrameTracked` events skipped since the last evaluated one; 0 means idle.

**Signals and rule** (`mapping/keyframe_decision.h`, pure
`ShouldCreateFrame(KeyFrameSignals, KeyFrameOptions)`):

| Signal | Definition |
|---|---|
| `ref_points` | points of the reference Frame (in the Mapper's map) with >= `min_obs` observations, `min_obs` = 2 while the map has <= 2 Frames, else 3; **a stereo-matched observation counts 2** (ruling H, ORB-SLAM3 `AddObservation`) |
| `since_last` | timestamp − the last Frame's timestamp |
| `inliers` | `num_inliers` |
| `close_tracked` / `close_untracked` | stereo: stereo-matched left features with depth < 40 baselines, inside / outside the inlier pairs |
| `need_close` | stereo: `close_tracked < 100 && close_untracked > 70` |

```
c1a = since_last >= max_interval (1 s)       c1b = backlog == 0
c1c = stereo && (inliers < 0.25 · ref_points || need_close)
ratio = mono ? 0.9 : (num_frames < 2 ? 0.4 : 0.75)
c2  = (inliers < ratio · ref_points || need_close) && inliers > 15
create = (c1a || c1b || c1c) && c2
```

The reference Frame comes from `FrameTracked::reference_frame_id`, a
`std::optional` (ruling A); when it is absent or no longer in the Mapper's map
the newest Frame is used. Every evaluation is logged at debug level with all
signals and the result.

**Creating a Frame** (`Mapper::CreateFrame`, one `KeyFrameCreated`):
1. Frame id from `next_frame_id_` (never reused), pose and the copied
   measurement.
2. Tracked observations of alive points (every id checked with `IsValid`);
   each touched point gets its normal (`MeanViewingDirection`) and
   representative descriptor (`DistinctiveDescriptor`: least median Hamming
   distance; even counts take the mean of the two middle distances, ties the
   lowest index — ruling E) recomputed, published as updated points.
3. Stereo close points (binocular Frames): untracked stereo-matched left
   features by depth; every one closer than 40 baselines and at least the 100
   closest; one stereo observation each.
4. Triangulation against neighbour Frames (`mapping/triangulation.h`): the top
   30 (mono) / 10 (stereo) Frames by covisibility; a neighbour is skipped when
   mono baseline / its median scene depth < 0.01, or stereo baseline < the rig
   baseline. Matching is **brute force** over the features without a point in
   either Frame: distance <= 50, ratio 0.6, the epipolar test on rays, the
   rotation histogram (12 bins), one-to-one. **Epipolar band (ruling F):** the
   angle between Frame b's ray and the epipolar plane (normal n) must satisfy
   `asin(|n·ray_b|) <= k·σ·‖(JJᵀ)⁻¹J n‖` with J = `Camera::ProjectJacobian` at
   the unit ray, k = 3 and σ = `sigma_px · level_scale^level` px — the exact
   directional 3σ bound (σ/f at a pinhole centre), not the norm of J.
   Acceptance: DLT on rays, ray parallax cos < 0.9998 (or ORB-SLAM3's stereo
   fallback when a stereo depth has more parallax), positive depth in both,
   raw-pixel χ² < 5.991 in both, and **scale consistency (ruling G):**
   dist_b / dist_a within [1/f, f] · level_scale^(level_a − level_b), f = 1.5 ·
   level_scale (ORB-SLAM3 order). Each new point has the new Frame's
   observation and the neighbour's (`added_observations`).
5. Publish. `sigma_px`, `level_scale` and the matching thresholds live in
   `MapperOptions::triangulation` (`TriangulationOptions`, ruling B); the
   neighbour counts, `min_close_points` and the baseline ratio in
   `MapperOptions`.

The Mapper keeps a `map::CovisibilityIndex` (per point: observing Frames; per
Frame: points), fed by initialization and `CreateFrame`, readable through
`Mapper::covisibility()` on the Mapper thread / in tests only (ruling C). The
replica keeps its own, fed by `KeyFrameCreated`.

**Tracker local map** (replaces "every alive point of the replica"):
- **K1:** the Frames observing a current match; **K2:** for each K1 Frame its
  top `local_neighbours` (10) covisible Frames not yet included (ruling K:
  each K1 Frame adds up to 10, as the spec words it; ORB-SLAM3 adds only the
  first new one); cap `max_local_frames` (80), K1 first by shared points.
- **Local points:** the alive points of the local Frames
  (`CovisibilityIndex` does not purge removed points, so `IsAlive` filters).
- **Reference Frame:** after the final `OptimizePose`, the Frame sharing the
  most inliers among **all** observing Frames (ruling L; the spec said K1).
- **`visible_points`:** local points passing the projection gate, **plus every
  already-matched point** without re-projection (ruling K, ORB-SLAM3
  `SearchLocalPoints`), so `visible ⊇ matched`.
- **Fast pickup:** a `KeyFrameCreated` whose timestamp equals `last_frame_`'s
  merges its observations into `last_frame_.matches`.

**Viewer.** `SceneState` consumes `FrameTracked` (pose, trajectory, tracked and
visible point sets, local Frames) and the combined `KeyFrameCreated`. Colours:
tracked points **green**, visible-but-unmatched points **blue**, other points
**grey**; local-map Frames **orange**, other Frames blue; current camera green
(red when lost). Checked on corridor1 mono with `--viewer` (200 frames,
2026-10-07): green, blue and orange all appear.

**Knowingly deferred** (sub-project 3/4, `docs/TODO.md`): fusion, point and
Frame culling, local BA (duplicates accumulate; tracking tolerates them through
the pose outlier rejection), `ObservationsRemoved`, interruptible BA and the
busy-Mapper branch, the post-relocalization Frame guard, the Faiss vs
DBoW2/FBoW index for matching.

**Synthetic end-to-end tests** (`SlamTest.{Mono,Stereo}MapGrowsWhileLeavingInitialView`):
120 frames, 6 m along +x through a 10 m room (`surround_scene`: points on the
walls, up to 1 m relief), turning 90° towards +x (`turn_deg`), so the last
frames see only a wall the first frames never saw (mono 1000 scene points;
stereo 700 with `stereo_min_features` 150, to keep the Debug run near 45 s).
Both track every frame
with no `TrackingLost`, create Frames and grow the map. The error bounds are
looser than planned (30 % of the path mono, 0.3 m stereo instead of 5 % and
2 cm): without local BA the error random-walks with the 0.5 px pixel noise
(measured 20 % / 0.15 m; 0 / 0 without noise). Tighten with local BA (TODO).
ThreadSanitizer (`Tracker*:Mapper*:Slam*`, 55 tests, 2026-10-07): clean.

**Smoke (corridor1 1024, Release, ORB 1500, debug logs, headless, 2026-10-07).**
Lifetimes are tracked frames per map; the last map of a run is cut by its end.

| Run | Maps (tracked frames) | First map | Median lifetime | Re-inits | Frames created | Points | Inliers (median) | Bar |
|---|---|---|---|---|---|---|---|---|
| Mono, 400 frames | 2 (129, 142+) | 129, lost | 135.5 | 1 | 1 / 2 (beside 2 init Frames per map) | 329 / 582 | 83 / 141 | first map ≥ 150: **missed** (129); median ≥ 50: met |
| Stereo, 1000 frames | 2 (360, 637+) | 360, lost | 498.5 | 1 | 17 / 252 (beside 1 init Frame per map) | 6 931 / 67 879 | 498 / 434 | no `TrackingLost`, one map: **missed** |
| *Before (master, 2026-10-06)* | mono 19 maps (82 tracked in 400), stereo 1 map in 200 | | ≈ 4 (mono) | 18 (mono) | – | 62-109 per mono map | 41 mono / 232 stereo | |

Mono initializes at about frame 120 (as before); map 1 then tracks 129 frames
(about 6.5 s) — the user-visible symptom (losing the map within a few frames)
is gone, but the first-map bar is missed. Diagnosis from the decision log: of
129 evaluations on map 1, 1 created a Frame (`c2` via the ratio, `c1b`) and
128 had `c2` false. After Frame 2, `ref_points` stays at 36: the 258
triangulated points have only 2 observations (< `min_obs` 3) and nothing adds
a third until the next Frame, so `inliers < 0.9 · ref_points` holds only when
inliers fall to 32, and they dropped from 41 to 27 (< 30 gate) in one frame
while the camera rotated at 50-70 deg/s (mocap). ORB-SLAM3 avoids the stall
through fusion (`SearchInNeighbors`), sub-project 3. Stereo creates Frames
mostly through `need_close` (215 of 252 in map 2) and is lost once, at
frame ~360, by the motion model (17 matches < 20, zero motion 6 inliers)
during a 120-170 deg/s rotation: a missing reference-Frame / relocalization
fallback (sub-project 4), not map starvation (a Frame had been created on each
of the three previous frames). Mapper cost per `CreateFrame` (temporary
timer, not committed): triangulation is ≥ 98 % of it, about 25 ms per mono
neighbour (brute force, 1500 features); mono 49-76 ms with 2-3 neighbours
(0.75 s expected with 30), stereo 88 ms mean (max 124 ms) with 8.8 neighbours, about 10 ms each. A second stereo run lost map 1 at frame ~354 and map 2 right after its initialization (3 maps): the stereo result varies between runs (thread timing). ATE vs mocap: not measured —
the apps do not write a trajectory (TODO).

## 7. Coding Conventions

**Naming (Google C++ style):**
- Classes, structs, functions, methods: `PascalCase` — `MapPointStorage`, `Add()`, `GetNextEvent()`.
- Trivial accessors may be `snake_case` and mirror the member — `size()`, `positions()`, `normalized_points()`.
- Local variables and parameters: `snake_case`.
- Member variables: `snake_case_` with trailing underscore.
- Parameters passed by non-const reference (outputs): `out_` prefix — `TPoint3DF& out_triangulated`.
- Constants: `kPascalCase` — `kInitialCapacity`, `kDescSize`.
- Template parameters and type aliases: `T` prefix — `TFeatureExtractor`, `TDescriptors`, `TMeasurement`.
- Enums: `enum class` with `PascalCase` values.
- Namespaces: `voyahslam::<module>` (`map`, `events`, `memory`, `features`, `geometry`, …).

**Structure:**
- C++20.
- Include guards `VOYAHSLAM_<PATH>_H` (no `#pragma once`).
- One event type per header; file name is the event name in `snake_case`.
- Every header is self-contained: it compiles when included first and alone.
- No OpenCV in public headers (principle 8): OpenCV usage lives in `.cpp` files
  and private headers under `src/` (e.g. `src/opencv_bindings/`,
  `vizualization/src/opencv_wrapping.h`).
- Link-level note: `voyahslam` is a static library, so its consumers still
  *link* OpenCV's shared libraries. In-tree, CMake propagates that
  automatically. When we add `install()`/package export, the package config must
  do `find_dependency(OpenCV)` itself (or `voyahslam` becomes a shared
  library) so the user still never calls `find_package(OpenCV)`.
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
- Dependencies (`conanfile.txt`): gtest, eigen, stb, spdlog, concurrentqueue,
  yaml-cpp (examples only), g2o (local package, see below).
- **OpenCV is not from Conan:** OpenCV **5.x** built from source and installed
  in `/usr/local` (`find_package(OpenCV 5 ... core imgproc features highgui)`,
  config in `/usr/local/lib/cmake/opencv5`). Every dev machine needs it.
  Build it with a GUI backend for voyahviz (`WITH_GTK=ON`; GTK2 dev files are
  enough). OpenCV 5 renamed `features2d` → `features` and split `calib3d` into
  `calib` / `geometry` / `stereo` (compat headers `features2d.hpp`,
  `calib3d.hpp` exist). Linked as: `voyahslam` → core, imgproc, features;
  `voyahviz` → core (public), imgproc, highgui; tests → all.
- **g2o is a local Conan package:** vendored in `third_party/g2o/` and built with
  `conan create third_party/g2o --build=missing -s build_type=Debug|Release` once per
  machine, before the project's `conan install` (`conanfile.txt` requires
  `g2o/20241228@voyah/stable`). Step-by-step build instructions: `README.md`
  (checked on a fresh `CONAN_HOME` on 2026-10-05).
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
| 2026-10-05 | Scope of the id rule made precise: `FrameId` unique per run; `MapPointId` unique within a map and append-only (`MapPointStorage::Remove` only marks a point dead, the free-list slot reuse is removed; `num_ids()` = next id); point ids restart per map and `(map_id, id)` identifies a point. CLAUDE.md updated to match. | The free list could hand a removed id to a new point (aliasing for lagging replicas, non-contiguous `MapPointsAdded` ranges); restarting per map is safe because every event carries `map_id`. |
| 2026-10-02 | Tracker lag between `NewKeyFrameCandidate` and resulting diffs is accepted. (Since 2026-10-06: between `FrameTracked` and the resulting `KeyFrameCreated`.) | Matches ORB-SLAM3 behaviour; Tracker keeps tracking with its current replica. |
| 2026-10-02 | `ObjectPool` free-list is a lock-free Treiber stack with tagged head; links in a separate index array. | Removes the last mutex from the event path; slot memory never read concurrently with construction. Verified with ThreadSanitizer. |
| 2026-10-02 | `Observable::Notify()` always takes ownership of the event. | One rule for producers regardless of how observers are wired. |
| 2026-10-02 | Pooled objects are default-constructed once and kept alive; `Recycle()` doesn't destroy, reuse calls optional `Reset()`. `Create()` takes no arguments. | Payload buffers (the bulk of allocation) are reused, not just the event shell. |
| 2026-10-02 | Tracker has its own thread and queue; `Track()` only enqueues. | Map diffs and tracking are processed on one thread, so the Tracker's replica needs no locks. |
| 2026-10-02 | Tracker uses two queues: map diffs (inbox) drained first, then measurements. `Track()` takes the measurement by value and enqueues it. | Replica is current before every tracking step; allows dropping stale measurements; one moodycamel queue wouldn't order the two producers anyway. |
| 2026-10-02 | `CameraFrame` → `MonocularMeasurement`, `BinocularCameraFrame` → `BinocularMeasurement`; timestamp stored in the measurement (binocular uses the left's); removed from `Frame` and keyframe events. | Avoids `CameraFrame`/`Frame`/`MonoFrame` confusion; one source of truth for time. |
| 2026-10-02 | Long-term goal: mixed monocular/binocular keyframes in one map. Tracker accepts both measurement types now; binding decided later. | Extension beyond ORB-SLAM3. |
| 2026-10-02 | Configuration lives in the user application; reference app in `examples/full_slam/`. | Library stays free of config-file parsing. |
| 2026-10-03 | `images::TWorkingImage` is 8-bit (`TImageU8Gray`), not float. | FAST/BRIEF only compare intensities, float = 4× memory traffic. GPU backends read 8-bit textures as normalized floats, so float storage isn't needed for a future GPU extractor; the extractor interface (image in, features out) is the GPU boundary. Same feature count; mono 231 → 209 ms. |
| 2026-10-03 | Long-term goal: drop the OpenCV dependency. | User preference; own Gaussian blur already, FAST next. |
| 2026-10-03 | Link a source-built OpenCV 5.1 from `/usr/local` instead of Conan's OpenCV 4.12. | Conan's 4.12 `cv::FAST` resizes its output to 10 000 keypoints on every call (57% of extraction time). With 5.1: mono extraction 231 → 70 ms, stereo ~404 → 149 ms on TUM-VI 1024² (float images). Also gives a GTK GUI backend. |
| 2026-10-03 | Map viewer: `SceneState` (event-driven model) + `MapViewer` (own thread, single inbox, fixed-rate loop); attached by the apps, not by `Slam`. | Emulates the future remote viewer: everything comes from events, so the consuming code is unchanged behind a serializer. Own thread decouples rendering from the processing loop and suits Pangolin. |
| 2026-10-03 | Respect the event boundary now; serialization + transport (HTTP/ROS) later. | Format depends on the transport choice; the boundary is what must not change. |
| 2026-10-03 | No OpenCV types in public headers; OpenCV linked `PRIVATE`; APIs use our types. voyahviz reworked accordingly (`TColorImage`, `Color`). Small libs (Eigen, spdlog, concurrentqueue) may stay public. | User requirement: library users must not need OpenCV headers or `find_package(OpenCV)`; requiring the small libs is acceptable. |
| 2026-10-03 | Visualization in a separate `voyahviz` library (`vizualization/`), allowed to use OpenCV; images via `cv::imshow`, keypoints drawn with `cv::circle`/`cv::line` rather than `cv::drawKeypoints`. | Dev-only tool. Own drawing avoids converting to `cv::KeyPoint` (whose size/octave we don't have) and allows orientation ticks and future colouring by level/tracking state. |
| 2026-10-03 | Measurements drop features whose ray is > 85° off the optical axis (`kMaxRayAngle`), tested in distorted space via `Distort()` of an 85° ray. | z = 1 normalized points can't represent ≥ 90° rays (fisheye); model-agnostic check needs no new distortion-model API. |
| 2026-10-02 | Dataset reader is synchronous (no thread); apps pace with `PlaybackClock` (absolute deadlines). | Measured: load ≈ 10 ms vs extraction ≈ 220 ms per frame — a reader thread can't help; keeps the reader trivially testable. |
| 2026-10-02 | `ObjectPool::Create()` aborts on exhaustion in every build type (not `assert`). | Release builds silently returned `nullptr`. |
| 2026-10-02 | Orchestrator named `Slam<FE>`; components share a non-virtual `EventNode` base for their observable + inbox. | `Slam` is what the user instantiates; `System` is too generic. One base avoids repeating the observer API in three classes. |
| 2026-10-02 | One event type per header, plus `events/events.h` umbrella; `map_events.h` removed. | User preference: easier to navigate and to include only what is used. |
| 2026-10-02 | Event payloads use grow-only `ColumnBuffer` (capacity + count) instead of plain `Eigen::Matrix`. | Eigen reallocates on any size change; grow-only buffers reach a steady state with zero allocations. Chosen over fixed max-size buffers, which waste memory and need a hard cap. |
| 2026-10-03 | Scope A for the first protocol: state machines + component loops + initialization protocol, with geometry behind rule-based stubs (`PoseEstimator`, `MapInitializer`). | Runs the whole pipeline end to end now; the geometry spec replaces the stub bodies in place without touching the protocol. |
| 2026-10-03 | Mono/stereo measurements travel as `std::variant` (`features::TMeasurement<FE>`) inside events instead of a template parameter (OQ 24). | One `EventType` per event; consumers holding `Event*` can pick the concrete type; mixed keyframes stay possible. |
| 2026-10-03 | Idle wait via per-component `WakeSignal` (atomic counter, C++20 wait/notify) + `WorkerThread` (OQ 20). Snapshot is taken before the `running` check. | Lock-free sleep/wake from both `Track()` and queue pushes; the snapshot order avoids a lost `Stop()` wake-up. |
| 2026-10-03 | The Mapper (only map writer) performs initialization from `InitializationMeasurement`s forwarded by the Tracker. | Single-writer principle: the Tracker never creates map objects. |
| 2026-10-03 | Maps are identified by a monotonic `map_id` on every event; consumer rule: ignore older, reset-and-adopt newer, apply equal; 0 exempt. | Detects stale events across map restarts; the initial map's diffs precede `MapCreated`, so a newer id must adopt rather than drop. |
| 2026-10-03 | A component changes its state (and `current_map_id_`) **before** publishing the event that announces it (`MapCreated` -> `Mapping`, `TrackingLost` -> `Lost`). | Whoever observes the event, on any thread, then also observes the new `state()`; publishing first let `state()` lag the protocol. |
| 2026-10-03 | Monocular initialization needs many frames (reference + a later frame); stereo uses one measurement. | Mono needs parallax between views; stereo has depth from one pair. |
| 2026-10-04 | Essential solver: normalized 8-point now, 5-point later (TODO, raised priority), behind an estimator interface that returns several candidates per sample. | 8-point is simple and verifiable; narrow-FOV scenes are ambiguous for it, which 5-point fixes. |
| 2026-10-04 | One generic `Ransac<TEstimator>` per model (H, E), not ORB-SLAM3's shared sample loop. | Reusable for PnP, Sim3, stereo; the models are independent; cost is two sequential runs (parallel run optional, TODO). |
| 2026-10-04 | ORB-SLAM3 truncated-χ² scores with σ in normalized units (σ_px / f). | Calibrated normalized points; keeps scores comparable across estimators and independent of the camera. |
| 2026-10-04 | `RigidTransformation` with hidden representation and `T_ab` (b -> a) convention; replaces the empty placeholder; `TPose` users migrate later (TODO). | 4x4 or quaternion storage can be swapped without touching callers; `T_ac = T_ab * T_bc` reads naturally. |
| 2026-10-04 | Local refinement: optional `Refine` (Levenberg-Marquardt over inliers) in each estimator, 5 rounds in `Ransac`; LM counts only accepted steps (20 accepted / 40 trials). | Minimal-sample models recovered only ~55 % of true inliers with errors of several degrees; counting rejected trials starved the budget. |
| 2026-10-04 | H-vs-E selection uses the COLMAP rule (`inliers_H >= 0.8 x inliers_E`), not the ORB-SLAM3 score ratio. | A calibrated E fits planes as well as general scenes, so the ratio does not separate them (ground truth gave about 0.41). |
| 2026-10-04 | Recovery/accuracy tests use wide-FOV scenes (`half_fov_tan = 1.0`, close to the TUM-VI fisheye); narrow-FOV and near-frontal planar scenes are tested as rejected/weak. | Narrow FOV is ambiguous for the 8-point solver; the weakness is tracked as 5-point priority rather than hidden by loose bounds. |
| 2026-10-04 | Accepted accuracy limits: translation direction < 10° on general scenes (max 9.92° over 21 seeds), rotation < 1°, recovery >= 50 % (narrow) / >= 75 % (wide, 30 % outliers), `num_triangulated >= 0.5 x true inliers`. | Statistics of the scores (noise in both views) and inlier-set lock-in of the refinement (least squares over a hard-thresholded, self-selected inlier set; LM itself converges); bundle adjustment after initialization provides the final accuracy. |
| 2026-10-04 | `ImageFeatures` stores the pyramid level per keypoint (`TLevels`, uint8); each extractor exposes `ScaleFactor(level)` (ORB `scale^level`, SIFT `2^(level/3)/2`). SIFT keypoints are sorted by response, truncated to `number_of_features`, angles in radians. | Scale-aware matching and initialization need the level; SIFT previously returned degrees and uninitialized columns when over-detecting. |
| 2026-10-04 | Matcher `max_distance` is `double` (spec said `uint32`). | SIFT float L2 distances must be configurable; ORB distances convert exactly. |
| 2026-10-04 | Orientation histogram default: 12 bins of 30 degrees, not 30 bins of 12 degrees. | ORB-SLAM3's `HISTO_LENGTH = 30` is applied as factor `1/30` per degree, i.e. 30-degree bins; that is its effective resolution. |
| 2026-10-04 | A failed reconstruction keeps the reference and the matcher's updated `last_matched` positions. | ORB-SLAM3 `mvbPrevMatched` behaviour: the next frame is matched against the tracked positions. |
| 2026-10-04 | Zero triangulated points and a non-finite median depth count as initialization failure. | The Mapper uses the result's keyframes/points directly; an empty or unscalable map must not be created. |
| 2026-10-04 | SIFT radians / strongest-N fix: see the `ImageFeatures` level row above. | Same change; recorded there. |
| 2026-10-04 | Monocular initialization: ORB-SLAM3 matcher + two-view reconstruction, scaled to median depth 1, published as `MapPointsAdded` -> `KeyFrameCreated` (with observations) -> `MapCreated`; per-map point ids; keyframes store `T_cw`. | Replicas need points before the keyframes that observe them; per-map ids keep storage compact and `(map_id, id)` unique. |
| 2026-10-04 | Pangolin map viewer: `SceneState` decodes events into a scene model (newest map wins); `PangolinRenderer` is private to voyahviz. | Same code can later be fed by a deserializer on a remote machine; renderer stays swappable. |
| 2026-10-04 | `Stop()` during `RunOnCurrentThread()` only requests the stop; the loop drains itself on its own thread. | Keeps the loop-thread invariant: the drain touches the scene/renderer, which only the loop thread may do. |
| 2026-10-04 | Closing the window does not end the loop; only `RequestStop()` / `Stop()` do. The loop keeps draining headless. | Producers' event pools must never stall because a user closed a window. |
| 2026-10-04 | A `RequestStop()` made before the loop is entered is preserved (never lost). | A producer finishing first must not leave `RunOnCurrentThread()` blocked forever. |
| 2026-10-04 | On Linux under Wayland with `PANGOLIN_WINDOW_URI` unset, `MapViewer`'s constructor sets it to `x11://`. | Pangolin's Wayland backend blocks in `eglSwapBuffers` on GNOME, stalling the drain and the producers' pools. |
| 2026-10-04 | Pangolin linked `PRIVATE` via `pango_display`. | No Pangolin headers/types for library users (principle 8). |
| 2026-10-04 | Window-creation failure counts as `window_closed()`, so `--viewer` without a display stops playback after the first frame. | The viewer keeps draining headless; an app that asked for a window should not silently play on without one. |
| 2026-10-04 | `MapViewer::RunOnCurrentThread()` besides `Start()` (macOS hedge). | Cocoa requires the GUI on the main thread; the app would then play back on a worker thread. Not used by the apps yet. In this mode other threads end the run with `RequestStop()` only. |
| 2026-10-04 | Apps: `--viewer` opens the window; playback stops when it is closed; after the last frame the app waits for it to be closed. | The map is only worth inspecting once playback ends; closing the window is the user's exit. |
| 2026-10-04 | Stereo matcher follows ORB-SLAM3's fisheye method: brute-force descriptor match (ratio 0.7) plus triangulation checks (positive depth, chi-square, parallax). Epipolar-guided search deferred. | A rectified row search is unsuitable for fisheye images; brute force is simple and verifiable, guided search is an optimization (TODO). |
| 2026-10-04 | Stereo matching runs on the Tracker thread, at dequeue, before the measurement is tracked or forwarded. | The Mapper then receives matched measurements; the Mapper thread stays free for map work and the measurement is matched once. |
| 2026-10-04 | `StereoRig::T_rl` (left -> right, `RigidTransformation`) replaces `R_rl`/`t_rl`; the dataset reader computed the inverse and is fixed. | One documented convention; the old fields were used inconsistently. |
| 2026-10-04 | `StereoMatches` is in its own header (`features/stereo_matches.h`). | Avoids an include cycle between `BinocularMeasurement` and the matcher. |
| 2026-10-04 | Stereo initialization: one identity keyframe; every valid depth becomes a metric map point (ORB-SLAM3); thresholds 500 features / 100 points. | One stereo pair already carries scale and depth; no median normalization. |
| 2026-10-04 | `InitialMapPoint::feature_in_second = -1` means a single-keyframe map. | The result type serves both mono (two keyframes) and stereo (one). |
| 2026-10-04 | Stereo observations are stored per map in an `ObservationStorage<Stereo>` beside the mono one. | Keeps mixed mono/stereo sessions possible. |
| 2026-10-04 | Stereo depth is capped at about 17 m by `max_parallax_cos = 0.99998` at the 0.11 m baseline. | Accepted for now; the limit is a matcher option, tune later. |
| 2026-10-05 | g2o is vendored pristine in `third_party/g2o/src/` and packaged by a local Conan recipe; linked `PRIVATE`; the public API stays on Eigen types. | Reproducible build across machines (no system g2o), no g2o in public headers (principle 8), adaptations live in the recipe so upstream updates are a directory swap. |
| 2026-10-05 | Local Conan package `g2o/20241228@voyah/stable`: `user`/`channel` set in the recipe (not passed on the command line). | A plain `conan create third_party/g2o` yields the reference `conanfile.txt` requires; a missing package gives Conan's "ERROR: Package 'g2o/20241228@voyah/stable' not resolved: Unable to find 'g2o/20241228@voyah/stable' in remotes." error, which the README explains. |
| 2026-10-05 | BundleAdjust works on undistorted normalized points (z = 1) with sigma in normalized units; poses are `T_cw`; analytic Jacobians. | Same convention as the geometry toolkit (no camera model inside the optimizer; works for pinhole and fisheye). Pixel-space residuals deferred (TODO). Superseded later on 2026-10-05: raw-pixel residuals (camera-model rows below). |
| 2026-10-05 | BundleAdjust is two-stage like ORB-SLAM3 LocalBA: Huber pass (5 iterations), deactivate edges with chi2 > 5.991 or depth <= 0, plain pass (default 10 iterations). | A single robust pass drifted (baseline collapse, free scale) with 10 % gross outliers; deactivating outliers and re-converging fixed it. |
| 2026-10-05 | Monocular initialization runs a two-keyframe BA after the two-view reconstruction; outlier points are dropped; fewer than 50 survivors fail the attempt. | Accuracy (the reconstruction is limited by the RANSAC models; see TODO "refinement lock-in"); ORB-SLAM3 does the same (`Optimizer::GlobalBundleAdjustemnt` in `CreateInitialMapMonocular`). |
| 2026-10-05 | Init-time error at low parallax is noise-limited (synthetic seeds initialize at frame 3, baseline 0.15 vs depth 4-8; ML optimum 0.3-2.7 degrees), so tests check convergence to the maximum-likelihood optimum (oracle from ground truth) rather than an absolute error bound. Mean error 2.11 -> 1.34 degrees with BA. | An absolute bound would test the noise, not the optimizer. |
| 2026-10-05 | Init BA uses 5 + 40 iterations and runs on the unscaled problem (baseline 1, depths ~40), before the median-depth rescale (ORB-SLAM3 order). | The plain stage needs about 40 iterations to converge there; LM stops at convergence so runtime is barely affected. Rescaling first would converge faster (TODO, tuning). |
| 2026-10-05 | The root `CMakeLists.txt` picks the concurrentqueue target (`VOYAHSLAM_CONCURRENTQUEUE_TARGET`); `src/CMakeLists.txt` only uses it. The choice is between `moodycamel::concurrentqueue` and `concurrentqueue::concurrentqueue`. | Found by the clean-machine build: the revision of concurrentqueue/1.0.4 that Conan Center serves today declares only `concurrentqueue::concurrentqueue`. |
| 2026-10-05 | Tracking against the map (sub-project 1): `MapReplica` fed by Mapper events; motion model -> zero-motion fallback (window x4) -> local map -> `OptimizePose`; the `PoseEstimator` stub is deleted. | Spec `2026-10-05-tracking-design.md`; keyframes, local BA, relocalization are sub-projects 2-4. |
| 2026-10-05 | (Q) The projection matcher takes the point source as a template parameter; `features/` does not depend on `tracking/`. `PointReference` lives in `map/point_reference.h`; the rotation histogram is shared in `features/rotation_histogram.h`. | No dependency cycle between features and tracking; one histogram implementation for the initialization and projection matchers. |
| 2026-10-05 | (R) The replica does not count events (spec said it would, as `SceneState`). | Nothing consumes the counters; the replica stays a pure map copy. |
| 2026-10-05 | (S) `OptimizePose`: 4 rounds x 10 iterations; every round restarts from the initial pose (ORB-SLAM3); Huber in rounds 0-1; chi2 5.991 mono / 9.488 stereo (4 dof); success >= 3 inliers. | ORB-SLAM3 `PoseOptimization` schedule; the stereo edge has 4 rows (left + right normalized; raw pixels since the camera-model change below). |
| 2026-10-05 | (T) Mono accuracy is limited by the noise-limited initial map at the default 1 deg parallax (about 12 % depth error, about 10 % translation error). Tests use 4 deg parallax for the accuracy bound and a separate default-init robustness test. | Same cause as the init-accuracy row above; tuning is deferred (TODO). |
| 2026-10-05 | Matcher deviations from ORB-SLAM3: the orientation histogram is built from the final claims; points without a reference observation are skipped by the local-map search; no scale-invariance distance check (the predicted level is clamped). | Simpler; the distance check is a TODO. |
| 2026-10-05 | BoW-less fallback: a wide-window (x4) zero-motion projection search replaces `TrackReferenceKeyFrame`. | No vocabulary yet; relocalization / place recognition is sub-project 4. |
| 2026-10-05 | Camera model, approach A: a `geometry::Camera{K, distortion}` value type (`Project`, `ProjectJacobian`, `Unproject`); not camera-model classes (approach B, TODO if ever needed) and not free functions. | Smallest change that gives every consumer one projection; ORB-SLAM3's `GeometricCamera` hierarchy is not needed for one lens model. |
| 2026-10-05 | All five consumers (projection matcher, `OptimizePose`, `BundleAdjust`, stereo matcher gate, initialization matcher windows) compare **raw (distorted) pixels** with σ = `sigma_px · 1.2^level` px; χ² thresholds unchanged (5.991 / 9.488). Rays stay rays (two-view RANSAC, triangulation, the 85° test). Replaces the normalized-plane residual space of the BA row, (S) and the stereo gate (their schedules and thresholds stand). | Spikes on corridor1 (mono, 400 frames, tracked / re-initializations): baseline 32 / 7; KB window + σ 53 / 5; + local-map gate 20 108 / 5; angle-only allowance σ·1.5·(1 + r²) with window ×(1 + r²) 85 tracked, 99 with gate 20; uniform σ 3 effectively disables outlier rejection. One noise model for the library, as ORB-SLAM3. |
| 2026-10-05 | Measurements keep both the distortion model pointer (`camera()`) and the raw keypoints (`distorted_points()`). The model is app-owned and must outlive every measurement, event and `Camera` copy (pointer rule, §6). | Consumers need raw pixels and the model; recomputing raw pixels from undistorted ones would lose precision and cost a `Distort` per use. |
| 2026-10-05 | `DistortionModel` (Dummy, Kannala-Brandt) moved from `features/distortion_models/` to `geometry/distortion_models/` (namespace `geometry::distortion_models`); forwarding headers (with a namespace alias) remain at the old path. | `geometry::Camera` needs the model and `geometry` must stay at the bottom of the dependency stack. |
| 2026-10-05 | (U) `DistortJacobian` for Kannala-Brandt: J = (d/r)·I + (d′(θ)·θ′(r) − d/r)·p̂p̂ᵀ, θ′(r) = 1/(1 + r²), identity for r < 1e-8. | Analytic; verified against central finite differences (the spec draft had an extra 1/r). |
| 2026-10-05 | (V) Pinhole invariance with the old normalized code holds exactly (1e-9) only for fx = fy. `OptimizePose`, `BundleAdjust` and the stereo gate now weight x and y separately in pixels, so χ² changes slightly on fx ≠ fy rigs (e.g. EuRoC). | The old code divided σ by one f; per-axis pixel weighting is the intended behaviour. |
| 2026-10-05 | The calibration-free angle allowance (σ · 1.5 · (1 + r²), window · (1 + r²)) is recorded as a fallback for lenses without a model, not implemented. | Spike 2 measured it within 0.6 % of the KB stretch up to 60° (85 tracked frames vs 53 for KB window + σ); not needed while every lens has a model (TODO). |
| 2026-10-05 | Smoke result of the camera-model branch (run 2026-10-06, corridor1, Release): mono 82 tracked / 18 re-initializations (gate 20: 137 / 14, measurement only); stereo 197/200 local-map successes (198 before), no loss, median 232 inliers. The ≤ 5 re-initializations criterion is missed; nothing was tuned. | The extra re-initializations come from the raw-pixel init matcher accepting more (mostly adjacent-frame) pairs: 19 maps vs 6; reverting only that matcher gives 6 maps / 56 tracked / 9.3 frames per map. Kept for more tracked frames (82 vs 56). Maps still die at the local-map gate without map growth (sub-project 2); init acceptance and gate tuning are in TODO. |
| 2026-10-06 | `FrameTracked` replaces `PositionUpdated` (deleted): every tracked measurement publishes pose, measurement copy, inlier pairs, visible points, reference Frame, local Frames, stage and counts. | One event gives the Mapper everything it needs to decide and the viewer the tracked / visible sets; no candidate protocol. |
| 2026-10-06 | The Mapper decides on Frame creation (`ShouldCreateFrame`, ORB-SLAM3 rule and defaults); it drains `FrameTracked` to the latest per map and counts the skipped ones as backlog (backlog 0 = idle). | The Mapper holds the authoritative observation counts, its own backlog and Frames the replica lacks; reverses the 2026-10-02 split. |
| 2026-10-06 | `NewKeyFrameCandidate` is deleted. | No candidate protocol (one in flight, rejections, timeouts) is needed once the Mapper decides. |
| 2026-10-06 | `KeyFrameCreated` is one atomic transaction (Frame, new points, observations on existing Frames, updated points), used by initialization too (mono 2 events, stereo 1); `MapPointsAdded` is deleted. | The replica applies an insertion in one `ApplyEvent`, so the Tracker never tracks against a half-applied one. |
| 2026-10-06 | The Tracker's local map is built from covisibility: K1 (Frames observing a match), K2 (top 10 covisible per K1 Frame), cap 80; local points are the local Frames' alive points. | ORB-SLAM3 `UpdateLocalKeyFrames` / `UpdateLocalPoints` without a spanning tree; keeps the search bounded as the map grows. |
| 2026-10-06 | Triangulation uses an epipolar test on rays and a raw-pixel χ² (5.991, σ = `sigma_px · level_scale^level`) in each Frame. | Rays work for pinhole and fisheye alike; the residual space matches the rest of the library. |
| 2026-10-06 | Matching for triangulation is brute force for now; the Faiss vs DBoW2/FBoW decision is deferred to sub-project 4. | Smallest working matcher; its cost is measured (smoke: about 25 ms per neighbour) and in TODO. |
| 2026-10-06 | (A) `FrameTracked::reference_frame_id` is `std::optional<FrameId>`; with none (or one the Mapper no longer has) the Mapper uses its newest Frame. (B) `sigma_px` / `level_scale` live in `MapperOptions::triangulation`. (C) The Mapper keeps its own `CovisibilityIndex` with a Mapper-thread-only `covisibility()` accessor. | (A) the Tracker may know none of the observing Frames; (B) one source of truth; (C) needed for `ref_points` and the replica-consistency test. |
| 2026-10-06 | (D, E) `DistinctiveDescriptor`: least median Hamming distance; an even count takes the mean of the two middle distances (ORB-SLAM3 takes the lower), ties go to the lowest index. | A strict winner on small sets; the plan's 1-bit example violated the triangle inequality (test uses 1/9/10). |
| 2026-10-06 | (F, spec amended) The epipolar band is the exact directional 3σ bound `asin(|n·ray_b|) ≤ k·σ·‖(JJᵀ)⁻¹J n‖` with J = `ProjectJacobian` at the unit ray (σ/f at a pinhole centre), replacing the spec's "norm of `ProjectJacobian`". | The spectral norm at z = 1 is up to 5.8× too loose at 80° and too strict along slow fisheye directions. |
| 2026-10-06 | (G, spec amended) Scale consistency compares dist_b / dist_a with level_scale^(level_a − level_b) (ORB-SLAM3 order). | The plan's dist_a / dist_b was inverted and rejected valid multi-level matches (`ScaleConsistencyFollowsOctaveRatio`). |
| 2026-10-06 | (H, spec amended) For `ref_points` / `min_obs` a stereo-matched observation counts 2 (ORB-SLAM3 `MapPoint::AddObservation`). | With a count of 1 a one-Frame stereo map has `ref_points` 0 and creates Frames only through `need_close`. |
| 2026-10-06 | (I) A later `TrackingLost` of the same map in the drained batch also supersedes a `FrameTracked`; all events keep their order. | No Frame is created for a map that is about to be cleared. |
| 2026-10-06 | (J) The mono triangulation unit test triangulates on the 4th Frame. | A noise-free initialization maps every synthetic point; new points duplicate existing ones until fusion. |
| 2026-10-06 | (K) `visible_points` includes already-matched points without re-projection (ORB-SLAM3 `SearchLocalPoints`), so visible ⊇ matched; K2 takes the top 10 neighbours of every K1 Frame (ORB-SLAM3 adds only the first new one). | The viewer and the Mapper see every tracked point as visible; the spec's wording for K2, slightly larger local map. |
| 2026-10-06 | (L) The reference Frame is the Frame sharing the most inliers among **all** observing Frames, not only K1. | Includes Frames reached through local-map matches, a better reference; occasionally a K2 Frame. |
| 2026-10-06 | Viewer colours: tracked points green, visible points blue, other points grey; local-map Frames orange, other Frames blue. | Debugging the local map and the matcher at a glance. |
| 2026-10-07 | Keyframes smoke result (corridor1, Release; "Keyframes and Map Growth"): mono 2 maps in 400 frames, first map 129 tracked frames, median lifetime 135.5, 1 re-initialization (was 82 tracked / 18 re-initializations); stereo 2 maps in 1000 frames (lost once at frame ~360). Both bars (first mono map ≥ 150; stereo no loss) are missed; nothing was tuned. Synthetic end-to-end error bounds loosened to 30 % of the path / 0.3 m. | Mono: the Frame rule stalls without fusion (`ref_points` stuck at 36); stereo: motion-model failure at 120-170 deg/s without a reference-Frame fallback. The synthetic error is noise random-walk without local BA (0 without noise). All in TODO. |

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
15. ~~Tracker threading~~ — decided 2026-10-02: own thread, two queues (§4).
16. ~~Measurement naming and timestamps~~ — decided 2026-10-02:
    `MonocularMeasurement` / `BinocularMeasurement`, timestamp in the measurement.
17. **Mixed mono/binocular keyframes:** `Frame<FE, TCameraFrameType>` fixes one
    type per map; Mapper/LoopCloser are templated only on `FE`. Needs a design
    that lets both keyframe types coexist (goal in §1).
18. ~~Component construction~~ — decided 2026-10-02: configuration (rig,
    extractor settings, pool sizes) is done by the user application; the
    reference application will live in `examples/full_slam/`.
19. **Tracker measurement policy:** track every measurement in order, or skip
    to the newest when falling behind (real-time)? Possibly configurable.
20. ~~Tracker idle wait~~ — resolved 2026-10-03: per-component `WakeSignal`
    (atomic counter, C++20 wait/notify) signalled by `AsyncObserver::Push()` and
    `Tracker::Track()`; `WorkerThread` sleeps on it (§4).
21. **ORB extraction speed.** Was ≈ 210 ms per 1024² image in Release with
    Conan's OpenCV 4.12; with OpenCV 5.1 (no per-call resize in `cv::FAST`) it
    is ≈ 70 ms (see decisions log). ORB-SLAM3 reference numbers are for 512²
    images with CLAHE applied first — not comparable to 1024². With OpenCV 5.1
    the 8-bit working image is ~8% *slower* than float (≈ 76 vs 70 ms mono);
    suspects: per-pixel `std::round` in `BilinearInterpolation`, 8-bit → float
    conversion per tap in `Filter` — re-profile before the extractor rework.
    Profile of the
    4.12 build, 2026-10-03 (perf, DWARF unwinding):
    - **57% `cv::FAST`:** the OpenCV 4.12 we link does `keypoints.resize(10000)`
      (280 KB, value-initialized) on every call before detecting; we call it per
      35-px cell (~1500–2500 calls/image, most cells twice because the dark
      TUM-VI images need the threshold-7 retry). Fresh memory → page faults.
      The allocator churn also makes timings depend on glibc's dynamic
      mmap/trim thresholds (stereo was 2× slower than 2× mono until fixed via
      `GLIBC_TUNABLES`).
    - 17% descriptors incl. our `images::Filter` blur (allocates two images per
      level per frame, per-tap border clamping); rest: allocations, pyramid
      resize, PNG decode.

    **Comparison with ORB-SLAM3** (`ORBextractor.cc`), same algorithm otherwise:
    - Cells: ORB-SLAM3 grows each cell by 6 px so FAST's 3-px border doesn't
      leave blind strips; ours don't overlap → undetectable 6-px strips every
      35 px.
    - Descriptor sampling: ORB-SLAM3 rounds rotated pattern offsets (`cvRound`),
      we truncate (`cast<int>`) → descriptors differ bit-wise (matters for using
      ORB-SLAM3 DBoW2 vocabularies).
    - ORB-SLAM3 keeps each keypoint's pyramid level (`octave`) and size;
      ~~`ImageFeatures` loses the level~~ — done: `ImageFeatures` carries
      `levels()` (used by the initialization matcher).
    - ORB-SLAM3 pads levels with a 19-px reflected border; we have none.
    - ORB-SLAM3 blurs into `uint8` with `cv::GaussianBlur`; our blurred image
      is float. Unused `cv::minMaxLoc` per cell in ours.

    **Plan:** own FAST-9-16 run once per level, corners assigned to cells
    (≥ 20 if any, else ≥ 7) — fixes speed, blind strips, and drops OpenCV from
    detection; preallocated blur/pyramid buffers; round descriptor offsets;
    keep the level in `ImageFeatures`. Then re-profile. Parallel left/right
    extraction after that.
22. **Fisheye rays beyond 90°:** `MonocularMeasurement` stores undistorted
    *normalized* points (z = 1), which cannot represent rays at ≥ 90° off-axis;
    TUM-VI's image corners are there. **Interim (2026-10-03):** features whose
    ray is more than `MonocularMeasurement::kMaxRayAngle` = 85° off-axis are
    dropped at construction. **Long term:** store unit bearing vectors instead of
    z = 1 points (needed for full fisheye FOV and mixed cameras) — decide with
    the Tracker design. Since 2026-10-05 residuals and gates are in raw pixels
    (only the geometry stays on z = 1 rays), so keeping features beyond 85°
    needs bearing vectors only on the ray side (TODO).
23. **Radial-tangential distortion** (EuRoC MAV) is not implemented; the reader
    rejects such calibrations.
24. ~~Mono/stereo variants share an event tag~~ — resolved 2026-10-03:
    measurements travel as `features::TMeasurement<FE>`, a `std::variant`
    inside the events (§6). Mixed keyframes in one map remain OQ 17.
11. ~~Payload buffers are not reused~~ — resolved 2026-10-02 (objects stay alive
    in the pool, payloads in grow-only `ColumnBuffer`s). Related: chunking large
    loop-closure diffs (Q3) would also bound the buffers' peak capacity.
12. ~~Event pool takes a mutex~~ — resolved 2026-10-02 (lock-free Treiber stack).
13. ~~Ownership in `Notify()` depends on wiring~~ — resolved 2026-10-02
    (`Notify()` always takes ownership).
14. **Pool sizing.** Slot memory is now only committed on first use, but the
    index array (4 B/slot) is written up front, and the default `max_count` is
    still 1,000,000. Pools should be sized per event type from the worst-case
    backlog (consumer rule 3 in §6.1).
25. **Event pool lifetime.** Components own the pools of the events they
    publish, but consumers (e.g. the `MapViewer` inbox) can still hold those
    events. The apps declare the viewer before `slam`, so on exception unwinding
    `Slam` is destroyed first — while the viewer thread may still be running
    and draining its inbox (`DrainInbox()` recycles into the Tracker/Mapper
    pools) — and the viewer's destructor drain then recycles into freed pools
    too. The apps now guard against this: a guard declared right after `slam`
    runs `slam.Stop(); viewer.Stop()` on every exit path, so the viewer is
    stopped and fully drained before `~Slam` (as `tests/slam/slam_test.cpp`
    does). This is a caller-side workaround; the library-level design stays
    open: pools that outlive all producers and consumers.

## 10. Status and Next Steps

**Done:**
- Ported core modules from slamlab2 (78 tests).
- Core data layer + event system (merge `91b56aa`).
- Lock-free event pool, `Notify()` ownership, payload buffer reuse (merge `2e10c59`).
- One header per event; `Slam` / `Tracker` / `Mapper` / `LoopCloser` /
  `EventNode` skeleton with wiring; measurement rename + timestamps; Tracker
  measurement queue (branches `refactor/event-per-file`,
  `feat/slam-threads-skeleton` — merge pending).

- `examples/`: `voyahslam_dataset_reader` (EuRoC/TUM-VI), `monocular_full_slam`,
  `binocular_full_slam` (branch `feat/full-slam-example`). Fixed an infinite
  loop in `KannalaBrandtDistortionModel::PrecomputeInitialGuessLookup` for
  fisheye lenses wider than the image.

- Two-view geometry toolkit (branch `feat/two-view-geometry`, merge pending):
  `RigidTransformation`, homography/essential estimators with LM refinement,
  generic `Ransac`, decomposition, `SelectPose`, `TwoViewReconstructor`
  (see "Geometry Toolkit"). Now wired into `MapInitializer`.

- Monocular map initialization (branch `feat/mono-initialization`, merge
  pending): keypoint levels, `FeatureGrid`, `InitializationMatcher`, real mono
  `MapInitializer`, Mapper stores and publishes the initial map with
  observations, `TPose` -> `RigidTransformation` migration (see "Monocular
  Initialization").

- Stereo map initialization (branch `feat/stereo-initialization`, merge
  pending): `StereoRig::T_rl`, ORB-SLAM3 fisheye-style `MatchStereo`, stereo
  matching on the Tracker thread, stereo `MapInitializer`, Mapper stereo
  observation storage (see "Stereo Initialization").

- Pangolin 3D map viewer (branch `feat/pangolin-viewer`, merge pending):
  `SceneState` decoding, `MapViewer` window lifecycle, `PangolinRenderer`,
  apps `--viewer` (see "Map Viewer").

- Tracker/Mapper state machines and initialization protocol (branch
  `feat/tracking-states`, merge pending): `MapId` + `Event::map_id` consumer
  rule, `TMeasurement` variant in events, `InitializationMeasurement` /
  `MapCreated` / `TrackingLost` events, `WakeSignal` + `WorkerThread`
  component threads (OQ 20, 24 resolved), stub `PoseEstimator` /
  `MapInitializer`, end-to-end `Slam` protocol test; apps print component
  states.

- Tracking against the map (branch `feat/tracking`, merge pending):
  `MapReplica`, `OptimizePose`, projection matching, frame-to-map Tracker, the
  `PoseEstimator` stub removed (see "Tracking Against the Map").

- Camera model (branch `feat/camera-model`, merge pending): `DistortionModel`
  moved to `geometry/` with `DistortJacobian`, `geometry::Camera`, measurements
  keep raw keypoints and `camera()`, all five residual / gate consumers in raw
  pixels (see "Camera Model and Raw-Pixel Residuals").

- Keyframes and map growth, tracking sub-project 2 (branch `feat/keyframes`,
  merge pending): `FrameTracked`, the Mapper's Frame decision and creation
  (stereo close points, triangulation against neighbours) as one
  `KeyFrameCreated`, covisibility local map and reference Frame, viewer
  highlights (see "Keyframes and Map Growth").

**Required follow-up:**
- Decide `MapPointStorage` growth / compaction (Open Question 2); ids are
  append-only since 2026-10-05.
- Decide on Open Question 14 (pool sizing per event type).

**Next:**
- ORB extractor plan in Open Question 21 (own FAST, buffers, descriptor
  rounding). Building OpenCV from source to link against
  (in progress, user).
- Tracker measurement policy (19).
- Sub-projects 3-4 of tracking (the user has their own designs; ask before
  designing): local BA, fusion and culling (fixes the mono Frame-rule stall,
  see "Keyframes and Map Growth"), relocalization / reference-keyframe
  tracking via place recognition.
- Event pool lifetime (Open Question 25).
- Then the Mapper, then the LoopCloser.
