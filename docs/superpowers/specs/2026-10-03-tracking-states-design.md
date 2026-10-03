# Tracker/Mapper State Machines and Map Initialization — Design Spec

Date: 2026-10-03
Status: approved in conversation, pending spec review

## Goal

Make the SLAM pipeline run end to end through its first protocol: the Tracker
forwards measurements while there is no map, the Mapper initializes a map from
them, announces it, the Tracker switches to tracking, and on tracking loss the
Mapper starts a new map. Real geometry (two-view initialization, pose
estimation) is **out of scope**: it sits behind two classes with rule-based
stub implementations that a follow-up spec replaces in place.

**Success criteria**

1. Component threads run, sleep when idle and wake immediately on input,
   without locks.
2. The protocol runs end to end in `Slam` with real threads: `MapCreated(1)`
   → `PositionUpdated`… → `TrackingLost(1)` → … → `MapCreated(2)`.
3. Events from a map that is no longer current are ignored everywhere.
4. Mono and stereo measurements travel in the same event types (OQ 24 resolved).
5. Every state machine transition is unit-tested without threads.

**Out of scope:** feature matching, RANSAC, homography/fundamental,
triangulation, real pose estimation, applying map diffs to the Tracker's
replica, keyframe decision, relocalization, LoopCloser logic, multi-map
storage, configuration loading in the apps.

## Decisions (from the conversation)

- Scope A: protocol + state machines + component loops; geometry stubbed.
- OQ 24 → measurement **variant** inside events.
- OQ 20 → per-component **`WakeSignal`** (`std::atomic<uint32_t>` wait/notify).
- Tracker states `WaitingForMap`, `Tracking`, `Lost`; Mapper states
  `Initializing`, `Mapping`. The Mapper (only map writer) does initialization.
- Map identity via `map_id`; initial map published as ordinary diffs followed
  by a `MapCreated` marker; on `TrackingLost` the Mapper starts a new map
  (relocalization / multi-map later).
- Mono initialization takes many frames (stub: ≥ 1 s between reference and
  current); stereo initializes from one measurement.

## 1. Events

### 1.1 `MapId`

`include/voyahslam/map/map_id.h`: strong type like `FrameId`
(`struct MapId { uint32_t value; ... = default comparisons }`). Ids start at 1
and only increase; `MapId{0}` means "not tied to a map".

`Event` gets a public `map::MapId map_id{}` field. Producers must set it on
every publish: pooled events are reused without reset (DESIGN.md §6.1 producer
rule 5), so a stale `map_id` would otherwise leak.

| Event | `map_id` |
|---|---|
| `MapPointsAdded/Updated/Removed`, `KeyFrameCreated`, `KeyFramePoseUpdated`, `MapCreated` | map the Mapper is building |
| `NewKeyFrameCandidate`, `PositionUpdated`, `TrackingLost` | map the Tracker tracks against |
| `LoopDetected` | map the LoopCloser searched |
| `InitializationMeasurement` | `0` (no map yet) |

**Rule for every consumer:** an event with `map_id != 0` and
`map_id != current map` is acknowledged and ignored. A `MapCreated` with a
`map_id` greater than the current one means "drop the old map".

### 1.2 Measurement variant

`include/voyahslam/features/measurement.h`:

```cpp
template <typename FE> using TMonocularMeasurement =
    MonocularMeasurement<typename FE::TDescriptorScalarType, FE::DescriptorSize>;
template <typename FE> using TBinocularMeasurement =
    BinocularMeasurement<typename FE::TDescriptorScalarType, FE::DescriptorSize>;
template <typename FE> using TMeasurement =
    std::variant<TMonocularMeasurement<FE>, TBinocularMeasurement<FE>>;
```

`Tracker`'s aliases and queue use these. `KeyFrameCreated<FE>` and
`NewKeyFrameCandidate<FE>` drop their `TMeasurement` template parameter and
hold `features::TMeasurement<FE> measurement`.

### 1.3 New event types

| Event | Header | Producer → consumers | Payload |
|---|---|---|---|
| `InitializationMeasurement<FE>` | `events/initialization_measurement.h` | Tracker → Mapper (+ other Tracker observers) | `features::TMeasurement<FE> measurement` |
| `MapCreated` | `events/map_created.h` | Mapper → all | `map_id` (base), `map::FrameId frame_id`, `geometry::TPose pose` — keyframe the Tracker continues from |
| `TrackingLost` | `events/tracking_lost.h` | Tracker → all | `map_id` (base), `double timestamp` |

`EventType` gains `InitializationMeasurement`, `MapCreated`, `TrackingLost`;
`events/events.h` includes the new headers. `voyahviz::SceneState::Apply`
gets the three cases (its no-`default` switch flags them).

### 1.4 Initial map publication order

On successful initialization the Mapper publishes, in this order:
`KeyFrameCreated` for each new keyframe, then `MapCreated`. (With the stubs
there are no map points, so no `MapPointsAdded`; real initialization will
publish it before `MapCreated`.) Per-producer FIFO guarantees that a consumer
seeing `MapCreated` has already received the map's contents.

## 2. Waking and Component Threads

### 2.1 `WakeSignal` — `include/voyahslam/events/wake_signal.h`

```cpp
class WakeSignal {
public:
    [[nodiscard]] uint32_t Snapshot() const;  // acquire load
    void Notify();                            // fetch_add (release) + notify_one
    void Wait(uint32_t seen) const;           // returns once value != seen
private:
    std::atomic<uint32_t> counter_{0};
};
```

Header-only (non-template, trivial; the project already keeps `Observable`
header-only). C++20 `atomic::wait` — a futex on Linux; no mutex.

### 2.2 `AsyncObserver` and `Observable`

- `AsyncObserver` gets `void SetWakeSignal(WakeSignal* signal)` (nullable,
  set before producers start) and `void Push(Event* event)` = enqueue, then
  `signal->Notify()` if set.
- `Observable` stores `AsyncObserver*` (not queue pointers) and delivers with
  `Push`. Public API unchanged (`ConnectAsync(AsyncObserver&)`).

### 2.3 `WorkerThread` — `include/voyahslam/events/worker_thread.h`

```cpp
class WorkerThread {
public:
    explicit WorkerThread(WakeSignal& signal);
    ~WorkerThread();                                   // Stop()
    void Start(std::function<bool()> process_pending); // no-op if running
    void Stop();                                       // idempotent; wakes + joins
    [[nodiscard]] bool running() const;
};
```

Loop: `while (running) { uint32_t seen = signal.Snapshot();
if (!process_pending()) signal.Wait(seen); }`. `Stop()` clears `running`,
calls `signal.Notify()`, joins. A notification between `Snapshot()` and
`Wait()` changes the value, so `Wait` returns immediately (no lost wake-up).
Defined in `src/events/worker_thread.cpp`.

### 2.4 Components

`EventNode` owns a `WakeSignal wake_signal_` and attaches it to `inbox_` in
its constructor. Each component (`Tracker`, `Mapper`, `LoopCloser`) owns a
`WorkerThread` bound to it, implements `[[nodiscard]] bool ProcessPending()`
(public, so tests can drive it synchronously), and its `Start()`/`Stop()`
start/stop the worker with `ProcessPending`. `Tracker::Track()` notifies the
wake signal after enqueuing. Each component owns pools for the events it
publishes (256 slots each for now, `Create()` aborts on exhaustion — OQ 14).
Each component logs state transitions at info level via `CLASS_LOGGER`.

## 3. Stubs (replaced in place by the geometry spec)

No virtual functions; the follow-up spec rewrites the bodies.

### `tracking::PoseEstimator<FE>` — `tracking/pose_estimator.h`

```cpp
struct PoseEstimate { bool success; geometry::TPose pose; };
PoseEstimate Estimate(const features::TMeasurement<FE>& measurement,
                      const geometry::TPose& last_pose);
static constexpr size_t kMinTrackedFeatures = 50;
```

Stub: success iff the measurement has ≥ `kMinTrackedFeatures` features
(binocular: left camera); pose = `last_pose`.

### `mapping::MapInitializer<FE>` — `mapping/map_initializer.h`

```cpp
struct InitialKeyFrame { geometry::TPose pose; features::TMeasurement<FE> measurement; };
std::optional<std::vector<InitialKeyFrame>> TryInitialize(features::TMeasurement<FE> measurement);
void Reset();  // drops the reference
static constexpr size_t kMinInitFeatures = 100;
```

Constructed with `double min_mono_init_interval` (seconds). Stub rules:

- Binocular measurement → success: 1 keyframe, identity pose.
- Monocular, no reference → becomes the reference; `nullopt`.
- Monocular with reference: if either has < `kMinInitFeatures` features → the
  current becomes the reference; `nullopt`. Else if
  `current.timestamp - reference.timestamp >= min_mono_init_interval` →
  success: 2 keyframes (reference, current), identity poses; reference
  cleared. Else `nullopt` (keep reference).

### `mapping::MapperOptions` — `mapping/mapper.h`

```cpp
struct MapperOptions { double min_mono_init_interval = 1.0; };
```

`Mapper(MapperOptions options = {})`; `Slam(mapping::MapperOptions mapper_options = {})`
passes it through. The apps keep the defaults (config loading later).

## 4. State Machines

### 4.1 Tracker

`enum class TrackerState { WaitingForMap, Tracking, Lost };`
`[[nodiscard]] TrackerState state() const` (atomic, any thread).
Members: `current_map_id_` (starts `MapId{0}`), `last_pose_`, `PoseEstimator`.

`ProcessPending()` — one pass:

1. Drain the inbox completely:
   - `MapCreated` with `map_id > current_map_id_` → `current_map_id_ = map_id`,
     `last_pose_ = pose`, state → `Tracking`.
   - `MapCreated` with `map_id <= current_map_id_` → ignore.
   - other events with `map_id != 0 && map_id != current_map_id_` → ignore.
   - other events of the current map → replica update (no-op for now).
2. If a measurement is queued, take **one**:
   - `WaitingForMap` / `Lost` → publish `InitializationMeasurement`
     (`map_id = 0`, measurement moved in).
   - `Tracking` → `Estimate()`. Success → `last_pose_ = pose`, publish
     `PositionUpdated` (`map_id`, `frame_id` = `MapCreated`'s frame id,
     `pose`, `timestamp`). Failure → publish `TrackingLost(current_map_id_,
     timestamp)`, state → `Lost`.
3. Return true iff an event or a measurement was processed.

`Lost` behaves like `WaitingForMap` now; relocalization will live there.

### 4.2 Mapper

`enum class MapperState { Initializing, Mapping };`
`[[nodiscard]] MapperState state() const` (atomic).
Members: `MapInitializer`, `next_map_id_` (starts 1), `current_map_id_`
(starts 0), `next_frame_id_` (starts 0, only increases — never reused, across
maps), keyframes `std::vector<map::Frame<FE, features::TMeasurement<FE>>>`.

`ProcessPending()` drains the inbox completely:

- `InitializationMeasurement`:
  - `Initializing` → `TryInitialize(copy of measurement)` (the event is shared
    with other consumers; never move out of it). On success: create a `Frame`
    per keyframe with ids from `next_frame_id_++`, store them,
    `current_map_id_ = next_map_id_++`, publish `KeyFrameCreated` per frame
    then `MapCreated(current_map_id_, last frame id, last frame pose)`, state →
    `Mapping`.
  - `Mapping` → ignore.
- `TrackingLost` with `map_id == current_map_id_` and state `Mapping` → clear
  keyframes (and map points), `initializer.Reset()`, state → `Initializing`.
  Otherwise ignore.
- `NewKeyFrameCandidate`, `LoopDetected` → acknowledge (handled later).
- Returns true iff any event was processed.

### 4.3 LoopCloser

Worker thread + `ProcessPending()` that drains and acknowledges everything.

## 5. Apps and Logging

No wiring changes (viewer stays attached). After the run the apps print the
Tracker's final state. With the stubs on TUM-VI (~1500 features/frame): mono
initializes after ~20 frames (1 s), stereo on frame 1, then every frame yields
`PositionUpdated`; the viewer's processed count becomes non-zero. State
transitions are logged, e.g. `Tracker: WaitingForMap -> Tracking (map 1)`,
`Mapper: Initializing -> Mapping (map 1, 2 keyframes)`.

## 6. Testing

| Area | Tests |
|---|---|
| `WakeSignal` | `Wait(seen)` returns immediately if the value changed; blocks until `Notify()` from another thread |
| `AsyncObserver`/`Observable` | `Push` notifies the attached signal; `Notify` delivers via `Push`; existing event tests updated and green |
| `WorkerThread` | runs `process_pending` until it returns false, then sleeps; wakes on `Notify`; `Stop()` wakes and joins; `Stop()` twice / without `Start()` safe |
| `PoseEstimator`, `MapInitializer` | each stub rule above (mono interval, feature thresholds, reference replacement, stereo, `Reset`) |
| Tracker (`ProcessPending`, no thread) | forwards `InitializationMeasurement` while `WaitingForMap`; `MapCreated` → `Tracking`; good measurement → `PositionUpdated` with `map_id`; 0 features → `TrackingLost` + `Lost`; stale `MapCreated` and other-map events ignored; `Lost` forwards again |
| Mapper (`ProcessPending`, no thread) | stereo initializes from one measurement; mono waits for the interval; too few features replaces the reference; `KeyFrameCreated` precede `MapCreated`; frame ids increase across maps; `TrackingLost` (current) → `Initializing`, next map id 2; stale `TrackingLost` ignored; `InitializationMeasurement` ignored while `Mapping` |
| End to end (`Slam`, threads) | mono measurements → external observer sees `MapCreated(1)` then `PositionUpdated`; a 0-feature measurement → `TrackingLost(1)`; more measurements → `MapCreated(2)`. Also under ThreadSanitizer |
| `SceneState` | counts the three new event types |

Measurements for tests are built from synthetic `ImageFeatures` (N random
points inside a pinhole camera's 85° cone, so none are dropped) with explicit
timestamps.

## 7. Documentation

- `DESIGN.md`: Tracker/Mapper state machines and the initialization protocol
  (§4); `map_id` rule and new events (§6); OQ 20 and OQ 24 resolved; decisions
  log.
- `TODO.md`: real `PoseEstimator`/`MapInitializer` (geometry spec); applying
  map diffs to the Tracker replica; keyframe decision (`NewKeyFrameCandidate`);
  relocalization in `Lost`; per-event-type pool sizes; remove the obsolete
  "Make `FrameId` allocation monotonic" item (done by the Mapper counter).
