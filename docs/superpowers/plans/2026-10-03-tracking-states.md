# Tracker/Mapper State Machines Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the first SLAM protocol end to end — Tracker forwards measurements, Mapper initializes a map (rule-based stubs), announces it with `MapCreated`, Tracker tracks, `TrackingLost` makes the Mapper start a new map — on lock-free component threads.

**Architecture:** `WakeSignal` (atomic wait/notify) + `AsyncObserver::Push` + `WorkerThread` give every component an idle-sleeping loop around a public `ProcessPending()`. Events gain `map_id` and carry `std::variant` measurements. Tracker and Mapper are explicit enum state machines; geometry sits behind `PoseEstimator` / `MapInitializer` stubs.

**Tech Stack:** C++20 (`std::atomic::wait`, `std::variant`), voyahslam events/pools, spdlog, GTest.

**Spec:** `docs/superpowers/specs/2026-10-03-tracking-states-design.md`

## Global Constraints

- Branch `feat/tracking-states`. Code style per DESIGN.md §7 (PascalCase functions/types, `snake_case_` members, `out_` for non-const reference params, `[[nodiscard]]` const accessors, include guards `VOYAHSLAM_<PATH>_H`, one event per header, every header self-contained).
- No locks (no `std::mutex`); waiting only via `WakeSignal`.
- No virtual functions in stubs/state machines; stubs are replaced in place later.
- Templates on `TFeatureExtractor` are declared in headers, defined in `src/`, instantiated with `INSTANTIATE_CLASS` (ORB, SIFT).
- Producers set `Event::map_id` on every publish (pooled events are reused without reset).
- Consumer rule: `map_id == 0` exempt; `< current` ignore; `> current` reset replica, adopt, apply; `== current` apply.
- Values: `PoseEstimator::kMinTrackedFeatures = 50`; `MapInitializer::kMinInitFeatures = 100`; `MapperOptions::min_mono_init_interval = 1.0` s; event pools 256 slots per published type; map ids start at 1; frame ids start at 0 and never repeat across maps.
- Build/test: `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8` (core tests binary: `cmake-build-debug/build/Debug/tests/voyahslam_tests`). New core tests go under `tests/` and into `tests/CMakeLists.txt`. Existing tests stay green.

## Review Focus

1. The Mapper must never move out of a shared `InitializationMeasurement` (other observers — the viewer — read the same event) → test in Task 6: a second observer still sees the full measurement after the Mapper processed it.
2. Initial-map ordering: `KeyFrameCreated(n)` arrives before `MapCreated(n)` at a consumer whose current map is older → must be applied, not dropped (Task 5 test `TrackerTest.InitialMapEventsBeforeMapCreatedAreKept`).
3. Lost wake-up: a `Notify()` landing between `Snapshot()` and `Wait()` → the worker must not sleep (Task 2 test `WorkerThreadTest.NoLostWakeUp`).
4. `Stop()` while the worker sleeps → returns promptly (Task 2 test `WorkerThreadTest.StopWakesSleepingWorker`).
5. Stereo measurement while the Mapper holds a mono reference → stereo initializes immediately and the reference is dropped (Task 4 test `MapInitializerTest.StereoInitializesEvenWithMonoReference`).

---

### Task 1: WakeSignal and AsyncObserver::Push

**Files:**
- Create: `include/voyahslam/events/wake_signal.h`
- Modify: `include/voyahslam/events/async_observer.h`, `include/voyahslam/events/observable.h`
- Test: `tests/events/wake_signal_test.cpp` (add to `tests/CMakeLists.txt`)

**Interfaces:**
- Produces: `events::WakeSignal { [[nodiscard]] uint32_t Snapshot() const; void Notify(); void Wait(uint32_t seen) const; }`; `AsyncObserver::SetWakeSignal(WakeSignal*)`, `AsyncObserver::Push(Event*)`; `Observable` stores `std::vector<AsyncObserver*>` and delivers via `Push`.

- [ ] **Step 1: Failing tests** in `wake_signal_test.cpp`:
  - `WakeSignalTest.WaitReturnsImmediatelyWhenValueChanged`: `seen = s.Snapshot(); s.Notify(); s.Wait(seen);` returns (test would hang otherwise; run with ctest timeout).
  - `WakeSignalTest.WaitBlocksUntilNotify`: thread waits on `Snapshot()`; main sleeps 50 ms, checks an `atomic<bool> woke == false`, then `Notify()`; join; `woke == true`.
  - `AsyncObserverTest.PushNotifiesAttachedSignal`: observer with `SetWakeSignal(&s)`; `Push(event)` → `s.Snapshot()` increased by 1; the event is dequeued by a `Listener`.
  - `AsyncObserverTest.ObservableDeliversThroughPush`: `Observable::ConnectAsync(observer)` + `Notify(event)` → signal incremented.
- [ ] **Step 2: Run** the core test build — expected: compile failure (`wake_signal.h` missing).
- [ ] **Step 3: Implement** per spec §2.1–2.2 (`Snapshot` acquire load; `Notify` `fetch_add(1, release)` + `notify_one`; `Wait` = `counter_.wait(seen, acquire)`). `Observable::Notify` keeps its ownership/ref-count semantics; only the enqueue becomes `observer->Push(event)`.
- [ ] **Step 4: Run** all core tests — expected: all pass (existing event tests unchanged).
- [ ] **Step 5: Commit** `Add WakeSignal; AsyncObserver::Push notifies it`.

### Task 2: WorkerThread

**Files:**
- Create: `include/voyahslam/events/worker_thread.h`, `src/events/worker_thread.cpp` (add to `src/CMakeLists.txt`)
- Test: `tests/events/worker_thread_test.cpp`

**Interfaces:**
- Consumes: `WakeSignal` (Task 1).
- Produces: `events::WorkerThread { explicit WorkerThread(WakeSignal&); ~WorkerThread(); void Start(std::function<bool()> process_pending); void Stop(); [[nodiscard]] bool running() const; }`.

- [ ] **Step 1: Failing tests** (`process_pending` lambdas over atomics):
  - `WorkerThreadTest.RunsUntilNoWorkThenSleeps`: work counter 5 → callback decrements and returns true while > 0; after ≤ 1 s the counter is 0 and the call count stops growing (sample twice 50 ms apart).
  - `WorkerThreadTest.WakesOnNotify`: after idle, set work = 1 and `signal.Notify()` → processed within 1 s.
  - `WorkerThreadTest.NoLostWakeUp`: 1000 iterations of: set work = 1, `Notify()`, wait (≤ 1 s each) until processed — no iteration times out.
  - `WorkerThreadTest.StopWakesSleepingWorker`: idle worker; `Stop()` returns in < 500 ms; `running() == false`.
  - `WorkerThreadTest.StopTwiceAndWithoutStart`: `Stop()` on a never-started worker and twice after start — no crash.
- [ ] **Step 2: Run** — expected: compile failure.
- [ ] **Step 3: Implement** per spec §2.3; `Start()` no-op if running, joins a finished thread before restarting.
- [ ] **Step 4: Run** all core tests — expected: pass. Run `WorkerThread*` under ThreadSanitizer (scratch build with `-fsanitize=thread -O1 -g`, `-DCMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE=PRE_TEST`, run via `setarch $(uname -m) -R`) — expected: no warnings.
- [ ] **Step 5: Commit** `Add WorkerThread (idle wait on WakeSignal)`.

### Task 3: Events — MapId, measurement variant, new event types

**Files:**
- Create: `include/voyahslam/map/map_id.h`, `include/voyahslam/features/measurement.h`, `include/voyahslam/events/{initialization_measurement,map_created,tracking_lost}.h`
- Modify: `events/event_types.h` (`map_id` field + 3 enumerators), `events/key_frame_created.h`, `events/new_key_frame_candidate.h` (single `FE` parameter, `features::TMeasurement<FE> measurement`), `events/events.h`, `tracking/tracker.h` (use `features::TMonocularMeasurement<FE>` etc.), `vizualization/src/scene_state.cpp` (3 cases)
- Test: `tests/events/event_system_test.cpp`, `tests/map/id_types_test.cpp`, `vizualization/tests/scene_state_test.cpp`

**Interfaces:**
- Produces: `map::MapId { uint32_t value; }` (defaulted `==`/`<=>`); `Event::map_id` (`map::MapId{}` default); `features::TMonocularMeasurement<FE>`, `TBinocularMeasurement<FE>`, `TMeasurement<FE>`; `events::InitializationMeasurement<FE>{ features::TMeasurement<FE> measurement; }`, `events::MapCreated{ map::FrameId frame_id; geometry::TPose pose; }`, `events::TrackingLost{ double timestamp; }`, `events::KeyFrameCreated<FE>`, `events::NewKeyFrameCandidate<FE>`; `EventType::{InitializationMeasurement, MapCreated, TrackingLost}`.

- [ ] **Step 1: Failing tests:**
  - `IdTypesTest.MapIdOrdering`: `MapId{1} < MapId{2}`, `MapId{} == MapId{0}`.
  - `EventSystemTest.NewEventTypeTags`: pooled `MapCreated`, `TrackingLost`, `InitializationMeasurement<Orb>` have the matching `type`; default `map_id == MapId{0}`.
  - `EventSystemTest.KeyFrameCreatedHoldsMonoOrStereo`: one `KeyFrameCreated<Orb>` pool; assign a mono measurement → `std::holds_alternative<TMonocularMeasurement<Orb>>`; recycle, reuse, assign binocular → holds binocular.
  - Update existing `KeyFrameCreated`/`NewKeyFrameCandidate` tests to the single-parameter form.
  - `SceneStateTest.CountsNewEventTypes`: applying one each of the 3 new types → each count 1.
- [ ] **Step 2: Run** — expected: compile failure.
- [ ] **Step 3: Implement** per spec §1. New events follow the existing pattern (see `events/map_points_removed.h`): `Event` + `Recyclable<Self, ThreadSafePolicy>`, function-pointer constructor. `InitializationMeasurement` has no `Reset()` (producer overwrites the variant). Every header self-contained.
- [ ] **Step 4: Run** all tests (core + voyahviz) — expected: pass; build has no `-Wswitch` warning.
- [ ] **Step 5: Commit** `Events: MapId, measurement variant, MapCreated/TrackingLost/InitializationMeasurement`.

### Task 4: Stubs — PoseEstimator and MapInitializer

**Files:**
- Create: `include/voyahslam/tracking/pose_estimator.h`, `src/tracking/pose_estimator.cpp`, `include/voyahslam/mapping/map_initializer.h`, `src/mapping/map_initializer.cpp`, `tests/support/test_measurements.h`
- Test: `tests/tracking/pose_estimator_test.cpp`, `tests/mapping/map_initializer_test.cpp`

**Interfaces:**
- Consumes: `features::TMeasurement<FE>` (Task 3).
- Produces: `tracking::PoseEstimate{ bool success; geometry::TPose pose; }`, `tracking::PoseEstimator<FE>::Estimate(const features::TMeasurement<FE>&, const geometry::TPose& last_pose)`; `mapping::InitialKeyFrame<FE>{ geometry::TPose pose; features::TMeasurement<FE> measurement; }`, `mapping::MapInitializer<FE>(double min_mono_init_interval)` with `std::optional<std::vector<InitialKeyFrame<FE>>> TryInitialize(features::TMeasurement<FE> measurement)` and `void Reset()`; test helpers `test::MakeMonocular(size_t num_features, double timestamp)` → `features::TMonocularMeasurement<OrbFeatureExtractor>`, `test::MakeBinocular(size_t num_features, double timestamp)` (pinhole K = diag(500,500,1) with c = (320,240), points within ±100 px of the centre so the 85° cut keeps all, `nullptr` distortion, identity `StereoRig`).

- [ ] **Step 1: Failing tests:**
  - `PoseEstimatorTest.SucceedsWithEnoughFeatures`: 50 features → success, pose == last_pose (non-identity input).
  - `PoseEstimatorTest.FailsWithTooFewFeatures`: 49 → `!success`.
  - `PoseEstimatorTest.UsesLeftCameraForStereo`: binocular 50 → success; 0 → failure.
  - `MapInitializerTest.StereoInitializesImmediately`: binocular 200 → 1 keyframe, identity pose, holds binocular.
  - `MapInitializerTest.MonoWaitsForInterval` (interval 1.0): mono t=0 → nullopt; t=0.5 → nullopt; t=1.0 → 2 keyframes (timestamps 0 and 1.0, in that order).
  - `MapInitializerTest.FewFeaturesReplacesReference`: t=0 (200 f) → nullopt (reference); t=0.6 (50 f) → nullopt (too few → becomes reference); t=1.2 (200 f) → nullopt (reference has < 100 → t=1.2 becomes reference); t=2.2 (200 f) → success, keyframe timestamps 1.2 and 2.2.
  - `MapInitializerTest.ResetDropsReference`: t=0, `Reset()`, t=1.0 → nullopt (became reference).
  - `MapInitializerTest.StereoInitializesEvenWithMonoReference`: mono t=0, binocular t=0.1 → 1 keyframe (binocular); then mono t=0.2 → nullopt (reference was cleared).
- [ ] **Step 2: Run** — expected: compile failure.
- [ ] **Step 3: Implement** per spec §3 (`std::visit` / `std::holds_alternative`; feature count = `points().cols()`, binocular → `left()`).
- [ ] **Step 4: Run** all core tests — expected: pass.
- [ ] **Step 5: Commit** `Add rule-based PoseEstimator and MapInitializer stubs`.

### Task 5: Tracker state machine and thread

**Files:**
- Modify: `include/voyahslam/events/event_node.h` (owns `WakeSignal wake_signal_`, attached to `inbox_` in ctor; protected accessor), `include/voyahslam/tracking/tracker.h`, `src/tracking/tracker.cpp`
- Test: `tests/tracking/tracker_test.cpp`

**Interfaces:**
- Consumes: Tasks 1–4.
- Produces: `tracking::TrackerState { WaitingForMap, Tracking, Lost }`; `Tracker<FE>::{ Start(), Stop(), Track(TMonocularMeasurement), Track(TBinocularMeasurement), [[nodiscard]] bool ProcessPending(), [[nodiscard]] TrackerState state() const, [[nodiscard]] map::MapId current_map_id() const }`.

Test harness: feed the Tracker's inbox from a test `Observable` (stands in for the Mapper, events from test pools with `map_id` set); capture what the Tracker publishes with `tracker.AddObserver(capture)` and a `Listener` on `capture`. No threads (call `ProcessPending()` until it returns false).

- [ ] **Step 1: Failing tests:**
  - `TrackerTest.ForwardsMeasurementsWhileWaitingForMap`: `Track(mono 200 f, t=0)` → captured `InitializationMeasurement` with `map_id == 0` holding the measurement; state `WaitingForMap`.
  - `TrackerTest.InitialMapEventsBeforeMapCreatedAreKept`: publish `KeyFrameCreated(map 1)` then `MapCreated(map 1, frame 1, pose P)` → state `Tracking`, `current_map_id() == MapId{1}`.
  - `TrackerTest.TracksAndPublishesPosition`: in `Tracking` on map 1, `Track(mono 200 f, t=2)` → captured `PositionUpdated` with `map_id 1`, `frame_id 1`, `pose P`, `timestamp 2`.
  - `TrackerTest.LosesTrackingOnFewFeatures`: in `Tracking`, `Track(mono 0 f, t=3)` → captured `TrackingLost` (`map_id 1`, `timestamp 3`); state `Lost`; next `Track(200 f)` → `InitializationMeasurement`.
  - `TrackerTest.IgnoresStaleMapCreated`: after map 2 is current, `MapCreated(map 1)` → still map 2.
  - `TrackerTest.NewMapAfterLoss`: Lost on map 1, then `KeyFrameCreated(2)`, `MapCreated(2)` → `Tracking`, map 2.
  - `TrackerTest.ThreadProcessesTrackedMeasurements`: `Start()`; `Track()` 3 measurements → within 1 s 3 `InitializationMeasurement`s captured; `Stop()`.
- [ ] **Step 2: Run** — expected: compile failure (`ProcessPending`/`state` missing).
- [ ] **Step 3: Implement** per spec §4.1 and the consumer rule; publish pools (256) per published type; `Track()` enqueues then `wake_signal_.Notify()`; `Start()`/`Stop()` drive a `WorkerThread` with `[this] { return ProcessPending(); }`; log transitions via `CLASS_LOGGER(Tracker)` (`"WaitingForMap -> Tracking (map 1)"`).
- [ ] **Step 4: Run** all core tests — expected: pass.
- [ ] **Step 5: Commit** `Tracker: state machine, map_id handling, worker thread`.

### Task 6: Mapper state machine, MapperOptions, LoopCloser loop

**Files:**
- Modify: `include/voyahslam/mapping/mapper.h`, `src/mapping/mapper.cpp`, `include/voyahslam/loop_closing/loop_closer.h`, `src/loop_closing/loop_closer.cpp`, `include/voyahslam/slam/slam.h`, `src/slam/slam.cpp`
- Test: `tests/mapping/mapper_test.cpp`, `tests/loop_closing/loop_closer_test.cpp`

**Interfaces:**
- Consumes: Tasks 1–4; `map::Frame<FE, features::TMeasurement<FE>>`.
- Produces: `mapping::MapperOptions{ double min_mono_init_interval = 1.0; }`; `mapping::MapperState { Initializing, Mapping }`; `Mapper<FE>(MapperOptions options = {})` with `Start()`, `Stop()`, `[[nodiscard]] bool ProcessPending()`, `[[nodiscard]] MapperState state() const`, `[[nodiscard]] map::MapId current_map_id() const`, `[[nodiscard]] size_t num_keyframes() const`; `LoopCloser<FE>::ProcessPending()`; `Slam<FE>(mapping::MapperOptions mapper_options = {})`.

Harness as in Task 5 (test `Observable` stands in for the Tracker; capture observer on the Mapper).

- [ ] **Step 1: Failing tests:**
  - `MapperTest.StereoInitializesFromOneMeasurement`: `InitializationMeasurement(binocular 200 f)` → captured exactly `KeyFrameCreated(map 1, frame 0)` then `MapCreated(map 1, frame 0)`; state `Mapping`; `num_keyframes() == 1`.
  - `MapperTest.MonoInitializesAfterInterval` (options interval 1.0): mono t=0, 0.5 → nothing published; t=1.0 → `KeyFrameCreated` ×2 (frames 0, 1) then `MapCreated(map 1, frame 1)`.
  - `MapperTest.FewFeaturesReplacesReference`: mono t=0 (200), t=0.6 (50), t=1.2 (200) → nothing; t=2.2 (200) → map created.
  - `MapperTest.IgnoresInitializationMeasurementWhileMapping`.
  - `MapperTest.TrackingLostStartsNewMap`: after map 1 (stereo), `TrackingLost(map 1)` → `Initializing`, `num_keyframes() == 0`; next stereo → `KeyFrameCreated(map 2, frame 1)` + `MapCreated(map 2)` (frame ids keep increasing).
  - `MapperTest.IgnoresStaleTrackingLost`: on map 2, `TrackingLost(map 1)` → still `Mapping`.
  - `MapperTest.DoesNotConsumeSharedMeasurement`: a second observer connected to the same test producer; after the Mapper processed the `InitializationMeasurement`, the second observer's event still has 200 features.
  - `LoopCloserTest.AcknowledgesEverything`: 1-slot pool event delivered, `ProcessPending()` → slot returned.
- [ ] **Step 2: Run** — expected: compile failure.
- [ ] **Step 3: Implement** per spec §4.2–4.3; `TryInitialize(copy)`; frame ids from `next_frame_id_` (never reset); `current_map_id_ = next_map_id_++` on success; `Slam` passes options to `Mapper`; `CLASS_LOGGER` transitions (`"Initializing -> Mapping (map 1, 2 keyframes)"`).
- [ ] **Step 4: Run** all tests — expected: pass.
- [ ] **Step 5: Commit** `Mapper: initialization state machine, map ids; LoopCloser loop; MapperOptions`.

### Task 7: End to end, apps, ThreadSanitizer

**Files:**
- Modify: `examples/full_slam/monocular_full_slam.cpp`, `examples/full_slam/binocular_full_slam.cpp`, `tests/slam/slam_test.cpp`

**Interfaces:**
- Consumes: `Slam<FE>(MapperOptions)`, `tracker().state()`, `mapper().state()` (Tasks 5–6).

- [ ] **Step 1: Failing test** `SlamTest.EndToEndInitializeTrackLoseReinitialize` (options interval 0.2; external `AsyncObserver` on `slam.mapper()` and `slam.tracker()`; helper waits ≤ 2 s for a condition): `Start()`; `Track(mono 200, t=0..0.3 step 0.1)` → observe `MapCreated(1)`, then after more frames `PositionUpdated(map 1)`, tracker `Tracking`; `Track(mono 0 f)` → `TrackingLost(1)`; `Track(mono 200 ...)` over ≥ 0.2 s → `MapCreated(2)`, tracker `Tracking` on map 2; `Stop()`.
- [ ] **Step 2: Run** — Tasks 5–6 already provide the behaviour, so the test may pass at once. Prove it can fail: temporarily stop the Mapper publishing `MapCreated` → the test must time out/fail; restore → PASS.
- [ ] **Step 3: Apps** print `tracker state: <name>` and `mapper state: <name>` after the stats (`slam.Stop()` before `viewer.Stop()` unchanged).
- [ ] **Step 4: Run** all tests in Debug and Release; ThreadSanitizer run of `SlamTest.*`, `TrackerTest.ThreadProcessesTrackedMeasurements`, `WorkerThreadTest.*` — expected: pass, 0 warnings. Run `monocular_full_slam … --rate=0 --max-frames=60` — expected: log shows `Mapper: Initializing -> Mapping (map 1, 2 keyframes)` around frame 20, `tracker state: Tracking`, viewer count ≈ 1 MapCreated + 2 KeyFrameCreated + ~40 PositionUpdated + forwarded InitializationMeasurements; binocular: map on frame 1.
- [ ] **Step 5: Commit** `End-to-end SLAM protocol test; apps report component states`.

### Task 8: Documentation

**Files:** `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: DESIGN.md** — §4: Tracker/Mapper state machines + initialization protocol + stubs; §6: `map_id` + consumer rule, measurement variant, new events table rows; OQ 20 and OQ 24 marked resolved (WakeSignal; variant); decisions log rows (scope A with stubs; variant; WakeSignal; Mapper initializes; map_id epochs; mono init takes many frames).
- [ ] **Step 2: TODO.md** — add real `PoseEstimator`/`MapInitializer` (geometry spec); apply map diffs to Tracker replica; keyframe decision (`NewKeyFrameCandidate`); relocalization in `Lost`; remove "Make `FrameId` allocation monotonic" (done); per-event-type pool sizes stays.
- [ ] **Step 3: Commit** `DESIGN/TODO: state machines, map ids, wake signal`.
