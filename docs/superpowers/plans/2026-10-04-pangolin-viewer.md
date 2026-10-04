# Pangolin Map Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Pangolin 3D window, owned by `voyahviz::MapViewer`, that shows the current map's points, keyframes, current camera and trajectory, all built from SLAM events, so that monocular initialization can be judged visually.

**Architecture:**
- `SceneState` becomes a GL-free scene model that decodes event payloads and applies the multi-map rule.
- `MapViewer` gets `MapViewerOptions` and two run modes: its own thread, or `RunOnCurrentThread()` (the macOS hedge). It also gets window-close semantics.
- A private `PangolinRenderer` draws the model on the thread that runs the loop.
- The apps get `--viewer`.

**Tech Stack:** C++20, Eigen, Pangolin 0.9.3 (installed in `/usr/local`, linked PRIVATE), GoogleTest, CMake/Conan 2.

**Spec:** `docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md`

## Global Constraints

- Branch `feat/pangolin-viewer`. Commit per task. Never merge or push.
- Google style:
  - `PascalCase` types and functions, `snake_case` variables, `snake_case_` members.
  - `out_` prefix for non-const reference parameters.
- No Pangolin or OpenCV type appears in any public header (`include/voyahviz/`, `include/voyahslam/`). Both are linked PRIVATE.
- The thread that runs the viewer loop is the only one that touches `SceneState` and the GL context.
- `voyahviz` is never built for Android. `voyahslam` gains no dependency.
- Build and test:
  - Full: `cmake --build cmake-build-debug/build/Debug -j8 && ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120`.
  - Viewer tests: `cmake-build-debug/build/Debug/vizualization/tests/voyahviz_tests --gtest_filter='<Suite>*'`.
  - All existing tests stay green (298 on master).
- Every new or changed header passes `clang++-18 -std=c++20 -stdlib=libc++ -fsyntax-only` (include/define flags from the `voyahviz_tests` target's `flags.make`).

## Review Focus

1. **A new map that starts without `MapPointsAdded`.** The stereo stub publishes only `KeyFrameCreated` and `MapCreated` for map 2. Expected: any event with a newer `map_id` switches the scene, not only `MapPointsAdded`. Test in Task 1: `NewerMapKeyFrameSwitchesScene`.
2. **`MapPointsAdded` whose `start_id` is beyond the current size (a gap).** Expected: the vectors grow and the gap ids are dead, not alive at (0, 0, 0). Test in Task 1: `AddedWithGapLeavesGapDead`.
3. **`Stop()` from another thread while `RunOnCurrentThread()` is running.** Expected: the loop ends, and the final drain happens on the loop's thread, never concurrently. Test in Task 3: `StopEndsRunOnCurrentThread`, also run under ThreadSanitizer.
4. **`WaitForWindowClose()` with no window, or after the loop has exited.** Expected: it returns, and never hangs. Tests in Task 3: `WaitForWindowCloseReturnsWhenHeadless` and `WaitForWindowCloseReturnsAfterLoopExit`.
5. **Restarting the viewer (`Stop()` then `Start()`) with a window.** Expected: a fresh window. This is manual, in Task 5, because it needs a display.

---

### Task 1: `SceneState` decodes events into a scene model

**Files:**
- Modify: `vizualization/include/voyahviz/scene_state.h`, `vizualization/src/scene_state.cpp`
- Test: `vizualization/tests/scene_state_test.cpp`

**Interfaces:**
- Produces, in `voyahviz`:
  - `enum class TrackingStatus : uint8_t { None, Tracking, Lost };`
  - `SceneState<FE>` const accessors:
    - `voyahslam::map::MapId current_map_id() const`
    - `const std::vector<geometry::TPoint3DF>& point_positions() const`
    - `const std::vector<bool>& point_alive() const`
    - `size_t num_alive_points() const`
    - `const std::map<voyahslam::map::FrameId, geometry::RigidTransformation>& keyframes() const` (T_cw)
    - `const std::optional<geometry::RigidTransformation>& current_pose() const`
    - `TrackingStatus status() const`
    - `const std::vector<geometry::TPoint3DF>& trajectory() const`
  - The existing `applied_count` and `total_applied` stay unchanged.
- Semantics: spec §1. The event table and the multi-map rule apply verbatim. Every event is counted, including ignored and map-0 events.

- [ ] **Step 1: Write failing tests.** Use pooled events, as the existing `ApplyOne` helper does, and set `map_id` explicitly. The existing three tests keep passing unchanged (their events carry map 0, so they are counted only).
  - `PointsAddedUpdatedRemoved`: map 1, `MapPointsAdded` start 0 with 3 points → positions equal, 3 alive. `MapPointsUpdated` ids {1, 7} → id 1 moved, id 7 ignored, size stays 3. `MapPointsRemoved` {0, 9} → id 0 dead, `num_alive_points() == 2`.
  - `AddedWithGapLeavesGapDead`: `MapPointsAdded` start 5 with 2 points into an empty scene → size 7, ids 0–4 dead, ids 5–6 alive.
  - `KeyFramesInsertedAndUpdated`: `KeyFrameCreated` frame 3, then `KeyFramePoseUpdated` frame 3 → one keyframe with the updated `T_cw` (`ToMatrix34().isApprox`).
  - `PositionUpdatedSetsPoseAndTrajectory`: a non-trivial `T_cw` (rotation and translation) → `current_pose` equals it, `status() == Tracking`, and `trajectory().back()` ≈ `T_cw.Inverse().translation()`. Two updates give two trajectory entries.
  - `MapCreatedSetsCurrentPose`: `current_pose` = pose, status Tracking, trajectory unchanged.
  - `TrackingLostKeepsScene`: after points and a pose, `TrackingLost` → status Lost. Points, keyframes and pose stay.
  - `OlderMapIgnored`: scene on map 2. A `MapPointsAdded` for map 1 → no change, but counted.
  - `NewerMapClearsAndAdopts`: map 1 with points, keyframes, pose and trajectory. A `MapPointsAdded` for map 2 → only the new points, no keyframes, no pose, empty trajectory, status None, `current_map_id() == 2`.
  - `NewerMapKeyFrameSwitchesScene`: map 1 with points. A `KeyFrameCreated` for map 2 → points cleared, one keyframe, map 2.
  - `MapZeroCountedOnly`: an `InitializationMeasurement` with map 0 after map 1 → scene unchanged, counted.
- [ ] **Step 2: Run** `voyahviz_tests --gtest_filter='SceneState*'`. Expected: compile failure (no accessors).
- [ ] **Step 3: Implement** spec §1 in `scene_state.{h,cpp}`.
  - Keep the `switch` with no `default` (the -Wswitch guard).
  - The event payload casts use the `FE` template parameter, as `Mapper::ApplyEvent` does.
- [ ] **Step 4: Run** the filter, then the full suite. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "SceneState: decode event payloads into a scene model; multi-map rule"`.

### Task 2: `FrustumInWorld` geometry helper

**Files:**
- Create: `vizualization/include/voyahviz/camera_frustum.h` (header-only)
- Test: `vizualization/tests/camera_frustum_test.cpp` (add it to `vizualization/tests/CMakeLists.txt`)

**Interfaces:**
- Produces:
  ```cpp
  struct CameraFrustum {
      geometry::TPoint3DF centre;
      std::array<geometry::TPoint3DF, 4> corners;  // top-left, top-right, bottom-right, bottom-left in the camera image
  };
  [[nodiscard]] CameraFrustum FrustumInWorld(const geometry::RigidTransformation& T_cw, double size);
  ```
  In camera coordinates the corners are `(x, y, z) = (±w, ±h, size)`, with `w = size · 2/3` and `h = size · 1/2`. That gives 4:3 aspect, and the half-width-to-depth ratio is 2/3. `x−` is left and `y−` is top (y down). All points are mapped by `T_cw.Inverse()`.

- [ ] **Step 1: Write failing tests:**
  - `IdentityPose`: centre (0, 0, 0). Corners (−w, −h, s), (w, −h, s), (w, h, s), (−w, h, s) for s = 0.1.
  - `NonTrivialPose`: R about an axis with ‖t‖ > 0. `centre ≈ T_cw.Inverse().translation()`, and `T_cw.Transform(corner)` gives back the camera-frame corners (1e-12).
- [ ] **Step 2: Run** `--gtest_filter='CameraFrustum*'`. Expected: compile failure.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Expected: pass.
- [ ] **Step 5: Commit** `git commit -m "voyahviz: camera frustum geometry helper"`.

### Task 3: `MapViewer` options, run modes, window lifecycle; Pangolin skeleton

**Files:**
- Modify: `vizualization/include/voyahviz/map_viewer.h`, `vizualization/src/map_viewer.cpp`, `vizualization/CMakeLists.txt`
- Create: `vizualization/src/pangolin_renderer.h`, `vizualization/src/pangolin_renderer.cpp` (private)
- Test: `vizualization/tests/map_viewer_test.cpp`

**Interfaces:**
- Consumes: `SceneState` (Task 1).
- Produces: `MapViewerOptions` and the `MapViewer` API exactly as in spec §2 (`fps`, `show_window`, `width`, `height`, `camera_size`; `Start`, `Stop`, `RunOnCurrentThread`, `RequestStop`, `window_closed`, `WaitForWindowClose`, `inbox`, `events_processed`). The `MapViewer(double fps)` constructor is replaced by `explicit MapViewer(MapViewerOptions options = {})`.
- Produces (private, used by Task 4):
  ```cpp
  template <typename FE> class PangolinRenderer {
  public:
      PangolinRenderer(const MapViewerOptions& options);  // creates and binds the window; throws on failure
      ~PangolinRenderer();                                // destroys the window
      // Draws one frame; returns false once the user closed the window.
      bool Draw(const SceneState<FE>& scene, size_t events_processed);
  };
  ```
  In this task, `Draw` only clears the screen, calls `pangolin::FinishFrame()`, and returns `!pangolin::ShouldQuit()`.
- Loop semantics:
  - Both modes run one private loop function: `Step()` paced to `fps`, until stopped.
  - **Renderer lifetime:** the renderer is created at loop entry only when `show_window` is set. On creation failure, log the error and set `window_closed_ = true`. The renderer is destroyed when `Draw` returns false (set `window_closed_` and notify the `WakeSignal`) and at loop exit.
  - **Final drain:** the loop does it on its own thread before returning. `Stop()` from another thread while `RunOnCurrentThread()` runs only requests the stop. `RunOnCurrentThread()` performs its own final drain.
  - **`Start()`:** joins its thread, then drains (as today).
  - **Mode conflict:** `RunOnCurrentThread()` while `Start()` is active throws `std::logic_error`. `Start()` while `RunOnCurrentThread()` is active is a no-op that logs a warning.
  - **`WaitForWindowClose()`:** returns immediately if `!show_window`, if `window_closed_` is set, or if no loop is running. Otherwise it waits on a `WakeSignal` that is notified on window close and on loop exit. Document that it supports one waiter at a time.
  - Use `CLASS_LOGGER(MapViewer)` (voyahslam logging).
- CMake: `find_package(Pangolin 0.9 REQUIRED)`, then `target_link_libraries(voyahviz PRIVATE ${Pangolin_LIBRARIES})` (or the `pango_*` targets Pangolin exports). No public include.

- [ ] **Step 1: Write failing tests** (all headless):
  - Existing tests: replace `TViewer viewer(fps)` with `TViewer viewer(MapViewerOptions{.fps = …})`. `RejectsNonPositiveFps` keeps its expectation.
  - `RunOnCurrentThreadStopsOnRequestStop`: start `RunOnCurrentThread()` on a `std::thread`, publish an event, wait until `events_processed() >= 1`, call `RequestStop()`, join. Expected: returns, and the event was processed.
  - `StopEndsRunOnCurrentThread`: same setup, but call `Stop()` from the test thread instead of `RequestStop()`. Expected: the runner thread returns, and events published before the stop are acknowledged (the pool slot is back, as in the existing tests' check).
  - `RunOnCurrentThreadWhileStartedThrows`: `Start()`, then `EXPECT_THROW(viewer.RunOnCurrentThread(), std::logic_error)`.
  - `WaitForWindowCloseReturnsWhenHeadless`: a never-started viewer, and a started headless viewer. Both calls return. `window_closed()` is false.
  - `WaitForWindowCloseReturnsAfterLoopExit`: a headless viewer started and stopped → returns.
- [ ] **Step 2: Run** `voyahviz_tests --gtest_filter='MapViewer*'`. Expected: compile failure.
- [ ] **Step 3: Implement** the API and the skeleton renderer. The CMake change goes in the same commit.
- [ ] **Step 4: Run** the filter, the full suite, and the clang-18 syntax check of `map_viewer.h`. Expected: green. ThreadSanitizer is deferred to Task 5.
- [ ] **Step 5: Commit** `git commit -m "MapViewer: options, RunOnCurrentThread, window lifecycle; Pangolin skeleton"`.

### Task 4: Render the scene

**Files:**
- Modify: `vizualization/src/pangolin_renderer.{h,cpp}`

**Interfaces:**
- Consumes: `SceneState` accessors (Task 1), `FrustumInWorld` (Task 2), `PangolinRenderer` (Task 3), `MapViewerOptions::camera_size`.

- [ ] **Step 1: Implement** spec §3:
  - **View and camera:**
    - `pangolin::OpenGlRenderState` with `ProjectionMatrix(width, height, 500, 500, width/2, height/2, 0.01, 1000)`.
    - `ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)`.
    - Mouse control: `Handler3D`.
    - Display bounds leave room for a 175 px panel on the left.
  - **Panel:** `pangolin::CreatePanel("menu")`, with these `pangolin::Var`s:
    - `bool`: "Show points", "Show keyframes", "Show trajectory" (all true), "Follow camera" (false).
    - `std::string` read-only: "Map", "Points", "Keyframes", "Status", "Events".
  - **Drawn items:**
    - Points: black, `glPointSize(2)`, alive points only.
    - Keyframes: blue line frustums. Each frustum draws the 4 centre→corner edges and the rectangle.
    - Current camera: green, or red when `status == Lost`.
    - Trajectory: grey `GL_LINE_STRIP`.
    - Background: white.
  - **Follow camera:** set the render state's model-view to follow `current_pose()` (or the last keyframe) using `pangolin::OpenGlRenderState::Follow` with the camera's `T_wc` as an `OpenGlMatrix`.
- [ ] **Step 2: Build** the whole tree and run the full suite. Expected: green (no new unit tests; drawing needs a display).
- [ ] **Step 3: Quick manual check**, if `DISPLAY` is available: a throwaway local program, not committed, or wait for Task 5's app flag. Record what you saw in the report.
- [ ] **Step 4: Commit** `git commit -m "PangolinRenderer: points, keyframes, current camera, trajectory, panel"`.

### Task 5: Apps `--viewer`, ThreadSanitizer, docs, manual verification

**Files:**
- Modify: `examples/full_slam/full_slam_common.h`, `examples/full_slam/monocular_full_slam.cpp`, `examples/full_slam/binocular_full_slam.cpp`, `docs/DESIGN.md`, `docs/TODO.md`

- [ ] **Step 1: Apps.**
  - `FullSlamOptions` gets `bool viewer = false`, parsed from `--viewer`. Add it to the usage text: `--viewer        open the 3D map viewer (Pangolin)`.
  - Construct `MapViewer` with `MapViewerOptions{.show_window = options.viewer}`.
  - The frame loop breaks when `viewer.window_closed()` is true.
  - After the loop, if `options.viewer && !viewer.window_closed()`: log `Playback finished — close the 3D viewer to exit`, then call `viewer.WaitForWindowClose()`.
  - The existing exit guard (`slam.Stop(); viewer.Stop();`) is unchanged.
- [ ] **Step 2: ThreadSanitizer.**
  - Use a scratch build with `-fsanitize=thread -O1 -g` and `-DCMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE=PRE_TEST`, with the Debug Conan toolchain.
  - Run `setarch $(uname -m) -R <build>/vizualization/tests/voyahviz_tests --gtest_filter='MapViewer*:SceneState*'`.
  - Expected: pass, with no `WARNING: ThreadSanitizer`.
- [ ] **Step 3: clang-18 + libc++ syntax check** of all sources. Expected: clean.
- [ ] **Step 4: Manual verification.** Use a Release build, the dataset at `/home/vahagn/Downloads/dataset-corridor1_1024_16`, and a display. Record the outcomes in the report.
  - `monocular_full_slam <dataset> --rate=0 --viewer --max-frames=200` → after about frame 121 the window shows 2 blue frustums, the points and a green current camera.
  - Close the window mid-run → playback stops and the app exits cleanly.
  - Run to `--max-frames` → the log asks you to close the window, the app waits, and closing it exits.
  - `binocular_full_slam <dataset> --viewer --max-frames=20` → 1 frustum, no points.
  - Without a display (`DISPLAY= monocular_full_slam … --viewer --max-frames=5`) → an error is logged and the app finishes without hanging.
  - If no display is available to the executor, state that clearly; the user verifies.
- [ ] **Step 5: Docs.**
  - `DESIGN.md`:
    - A viewer section: `SceneState` as the scene model built from events and its multi-map rule; the loop-thread invariant; `Start()` / `RunOnCurrentThread()` and the macOS rationale; window-close semantics; Pangolin as a private dependency.
    - Decisions-log rows dated 2026-10-04.
  - `TODO.md`:
    - Remove "Pangolin renderer in `MapViewer::Render()` …" and the window-close/`Stop()` part of the MapViewer hardening item. Keep the rest.
    - Add: the macOS setup (playback on a worker thread, viewer and image window on the main thread), the 2D initialization overlay, visual extras (covisibility edges, colouring by observations, video recording), and "install Pangolin 0.9 on the second development machine".
- [ ] **Step 6: Commit** `git commit -m "Apps: --viewer; docs for the Pangolin map viewer"`.
