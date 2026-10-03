# Map Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `voyahviz::MapViewer` that subscribes to Tracker and Mapper events through one inbox, runs on its own thread, and drains/acknowledges every event into an event-driven `SceneState`, attached in both `full_slam` apps.

**Architecture:** `SceneState<FE>` (no threads, no GUI) applies events — for now it only counts them per `EventType`. `MapViewer<FE>` owns one `events::AsyncObserver` inbox, a thread ticking at a fixed rate (drain → apply → empty `Render()` → sleep to absolute deadline), and a final drain in `Stop()`. Apps wire `viewer.inbox()` to Tracker and Mapper before `slam.Start()` and stop `Slam` before the viewer.

**Tech Stack:** C++20, voyahslam events (`Observable`, `AsyncObserver`, `Listener`, pooled `Recyclable` events), GTest, CMake.

**Spec:** `docs/superpowers/specs/2026-10-03-map-viewer-design.md`

## Global Constraints

- Branch: `feat/map-viewer`. Library `voyahviz` in `vizualization/`, namespace `voyahviz`, includes `<voyahviz/...>`.
- No OpenCV anywhere in this feature (DESIGN.md principle 8).
- `SceneState` and `MapViewer` are templated on `TFeatureExtractor`; declared in headers, defined in `src/*.cpp`, explicitly instantiated with `INSTANTIATE_CLASS` from `<voyahslam/features/instantiation.h>` (ORB + SIFT).
- The viewer uses only event contents — no access to `Slam`, components or shared state.
- Default frame rate: `fps = 30.0`.
- Code style per DESIGN.md §7: PascalCase types/functions, `snake_case` locals, `snake_case_` members, `out_` for non-const reference params, `[[nodiscard]]` on const accessors, include guards `VOYAHVIZ_<NAME>_H`.
- Build/test: `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8`; Release in `build/build/Release`. All existing 155 tests keep passing.

## Review Focus

1. `Stop()` on a viewer that was never started — must not hang or crash, and must still acknowledge queued events.
2. Events published after `Stop()` (producer stopped late) — the destructor must acknowledge them (`~MapViewer` calls `Stop()`, which drains).
3. `Start()` called twice, and `Start()` after `Stop()` (restart) — no second thread, no `std::terminate` from an unjoined thread.
4. Non-positive `fps` — constructor throws `std::invalid_argument`.
5. A slow/late tick (drain takes longer than the period) — no catch-up burst; next deadline is `now + period`. (Behavioral; covered by code review, not a timing test.)

---

### Task 1: SceneState

**Files:**
- Create: `vizualization/include/voyahviz/scene_state.h`, `vizualization/src/scene_state.cpp`
- Modify: `vizualization/CMakeLists.txt` (add source), `vizualization/tests/CMakeLists.txt` (add test file)
- Test: `vizualization/tests/scene_state_test.cpp`

**Interfaces:**
- Produces:
  ```cpp
  template <typename TFeatureExtractor> class SceneState {
  public:
      void Apply(const voyahslam::events::Event& event);
      [[nodiscard]] size_t applied_count(voyahslam::events::EventType type) const;
      [[nodiscard]] size_t total_applied() const;
  };
  ```

- [ ] **Step 1: Write the failing tests** in `scene_state_test.cpp` (use `memory::ObjectPool<…, ThreadSafePolicy>` to create events; recycle them after `Apply`):
  - `SceneStateTest.StartsEmpty`: `total_applied() == 0`, `applied_count(EventType::PositionUpdated) == 0`.
  - `SceneStateTest.CountsAppliedEventsPerType`: apply 2× `PositionUpdated`, 1× `MapPointsAdded<OrbFeatureExtractor>`, 1× `MapPointsRemoved` → counts 2 / 1 / 1, `applied_count(EventType::KeyFrameCreated) == 0`, `total_applied() == 4`.
- [ ] **Step 2: Run** `cmake --build cmake-build-debug/build/Debug -j8 --target voyahviz_tests` — expected: compile failure (`scene_state.h` missing).
- [ ] **Step 3: Implement** `SceneState` in `scene_state.h` / `scene_state.cpp`. Per-type counters (e.g. `std::array<size_t, N>` indexed by the enum's underlying value). `Apply` is a `switch` over every `EventType` enumerator with **no `default`**, so adding an enumerator triggers `-Wswitch` here; every case only increments its counter for now (comment: payload decoding comes with Pangolin). `INSTANTIATE_CLASS(SceneState)` in the `.cpp`. Add `src/scene_state.cpp` to `voyahviz`.
- [ ] **Step 4: Run** the build and `cmake-build-debug/build/Debug/vizualization/tests/voyahviz_tests --gtest_filter='SceneState*'` — expected: 2 tests PASS.
- [ ] **Step 5: Commit** `Add voyahviz::SceneState (event-driven viewer model)`.

### Task 2: MapViewer

**Files:**
- Create: `vizualization/include/voyahviz/map_viewer.h`, `vizualization/src/map_viewer.cpp`
- Modify: `vizualization/CMakeLists.txt`, `vizualization/tests/CMakeLists.txt`
- Test: `vizualization/tests/map_viewer_test.cpp`

**Interfaces:**
- Consumes: `SceneState<FE>::Apply`, `total_applied()` (Task 1); `events::AsyncObserver::GetListener()`, `Listener::GetNextEvent()`.
- Produces:
  ```cpp
  template <typename TFeatureExtractor> class MapViewer {
  public:
      explicit MapViewer(double fps = 30.0);   // throws std::invalid_argument if fps <= 0
      ~MapViewer();                            // Stop()
      void Start();                            // no-op if running; restart after Stop() allowed
      void Stop();                             // idempotent; join, then final DrainInbox()
      [[nodiscard]] voyahslam::events::AsyncObserver& inbox();
      [[nodiscard]] size_t events_processed() const;   // atomic
  };
  ```

- [ ] **Step 1: Write the failing tests** in `map_viewer_test.cpp` (producer = `events::Observable` with `ConnectAsync(viewer.inbox())`; events from 1-slot `ObjectPool<PositionUpdated, ThreadSafePolicy>` unless stated; "slot returned" = `pool.TryCreate() != nullptr`; wait for the viewer with a bounded poll, e.g. up to 2 s on `events_processed()`):
  - `MapViewerTest.ProcessesEventPublishedBeforeStart`: publish 1 event, `Start()`, wait until `events_processed() == 1`, then slot returned (`TryCreate` succeeds) while viewer still running.
  - `MapViewerTest.StopDrainsQueuedEvents`: `Start()`, `Stop()`, publish 1 event, `Stop()` again → `events_processed() == 1`, slot returned.
  - `MapViewerTest.StopWithoutStartDrains`: never `Start()`; publish 1 event; `Stop()` → slot returned, `events_processed() == 1`.
  - `MapViewerTest.DestructorAcknowledgesEventsPublishedAfterStop`: viewer in a scope, `Start()`, `Stop()`, publish 1 event, leave scope → slot returned.
  - `MapViewerTest.ReceivesFromTwoProducers`: two `Observable`s both connected to `viewer.inbox()`, a 16-slot pool; `Start()`; publish 10 from each; wait until `events_processed() == 20`; `Stop()`.
  - `MapViewerTest.StartTwiceAndRestart`: `Start(); Start(); Stop(); Start();` publish 1, wait for `events_processed() == 1`, `Stop()` — no crash.
  - `MapViewerTest.RejectsNonPositiveFps`: `EXPECT_THROW(MapViewer<OrbFeatureExtractor>(0.0), std::invalid_argument)`.
- [ ] **Step 2: Run** the test build — expected: compile failure (`map_viewer.h` missing).
- [ ] **Step 3: Implement** `MapViewer` per the spec's "Thread Lifecycle and Data Flow": members `events::AsyncObserver inbox_`, `SceneState<FE> scene_`, `std::atomic<bool> running_`, `std::atomic<size_t> events_processed_`, `std::thread thread_`, `std::chrono::steady_clock::duration period_`. `Run()`: `while (running_) { DrainInbox(); Render(); sleep_until(next); next += period_; if (next < now) next = now + period_; }`. `DrainInbox()`: local `Listener`, loop `GetNextEvent()` until `nullptr`, `scene_.Apply(*e)`, `++events_processed_`. `Render()`: empty, comment "Pangolin rendering goes here". `Start()` joins a finished thread before spawning (restart). `INSTANTIATE_CLASS(MapViewer)`.
- [ ] **Step 4: Run** `voyahviz_tests --gtest_filter='MapViewer*'` — expected: 7 PASS. Then the full `ctest` — expected: all pass.
- [ ] **Step 5: ThreadSanitizer**: configure a scratch build with `-DCMAKE_CXX_FLAGS="-fsanitize=thread -O1 -g" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"` (Debug Conan toolchain), build `voyahviz_tests`, run `setarch $(uname -m) -R <build>/vizualization/tests/voyahviz_tests --gtest_filter='MapViewer*'` — expected: PASS, no `WARNING: ThreadSanitizer`.
- [ ] **Step 6: Commit** `Add voyahviz::MapViewer (own thread, single inbox, drains events)`.

### Task 3: Attach the viewer in the full_slam apps

**Files:**
- Modify: `examples/full_slam/monocular_full_slam.cpp`, `examples/full_slam/binocular_full_slam.cpp`

**Interfaces:**
- Consumes: `MapViewer<TFeatureExtractor>`, `inbox()`, `events_processed()` (Task 2); `slam.tracker().AddObserver(...)`, `slam.mapper().AddObserver(...)`.

- [ ] **Step 1: Wire** in both apps, in this order: construct `voyahviz::MapViewer<TFeatureExtractor> viewer;` → `slam.tracker().AddObserver(viewer.inbox()); slam.mapper().AddObserver(viewer.inbox());` → `viewer.Start();` → `slam.Start();` … after the loop `slam.Stop(); viewer.Stop();` → print `viewer: <N> events processed` after the stats. Comment the stop ordering (Slam before viewer).
- [ ] **Step 2: Run** `build/build/Release/examples/full_slam/monocular_full_slam ~/Downloads/dataset-corridor1_1024_16 --rate=0 --max-frames=20` and the binocular app — expected: normal stats and `viewer: 0 events processed` (Tracker/Mapper don't publish yet), clean exit.
- [ ] **Step 3: Commit** `Attach MapViewer to Tracker and Mapper in full_slam apps`.

### Task 4: Documentation

**Files:**
- Modify: `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: DESIGN.md** — add to the Visualization section a "Map Viewer" subsection (components, single inbox, lifecycle, stop ordering, `SceneState` fed only by events); add "Remote visualization" (serializing async observer on the SLAM machine → HTTP/ROS → receiver feeding `SceneState` on the viewer machine; images are a separate flow); add the event pointer rule to §6 (events may point only at immutable, run-lifetime data — calibration/config — which the serializer sends by value; everything else by value); add a new Open Question for the shared mono/stereo tag of `KeyFrameCreated` / `NewKeyFrameCandidate`; add the no-filtering note to §6; decisions-log rows for: viewer on own thread, single inbox, respect-boundary-now/serialize-later.
- [ ] **Step 2: TODO.md** — add: event serialization + transport (HTTP/ROS); separate image flow for the camera view; `AsyncObserver` destructor doesn't acknowledge queued events; shared mono/stereo event tag; Pangolin renderer in `MapViewer::Render()` (+ `SceneState` payload decoding).
- [ ] **Step 3: Commit** `DESIGN/TODO: map viewer, remote visualization, event rules`.
