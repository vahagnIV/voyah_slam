# Pangolin Map Viewer — Design Spec

Date: 2026-10-04
Status: approved in conversation, pending spec review

## Context and Scope

`voyahviz::MapViewer` (spec `2026-10-03-map-viewer-design.md`) already runs on
its own thread and drains Tracker and Mapper events into `SceneState`. Today,
`SceneState` only counts events and `Render()` is empty. Monocular
initialization is now real. This spec adds a 3D view of the map, so that
initialization can be judged visually before tuning on TUM-VI 512.

**In scope**

1. `SceneState` decodes event payloads into a scene model: map points,
   keyframe poses, the current camera and the trajectory. It applies the
   multi-map rule.
2. A Pangolin renderer draws that model in a window, with a side panel.
3. `MapViewer` gets options, two run modes (its own thread, or the caller's
   thread — the macOS hedge), and window-close semantics.
4. The full_slam apps get a `--viewer` flag. Closing the window ends the run.
   When playback finishes, the window stays open until it is closed.

**Out of scope (→ TODO.md):**

- macOS run setup. The apps would move playback to a worker thread and drive
  the viewer from the main thread. The OpenCV image window would also need
  the main thread.
- A 2D overlay of initialization matches and reprojections.
- Event serialization and a remote viewer (already in TODO).
- Covisibility edges, colouring points by observation count, and video
  recording.

**Success criteria**

1. `SceneState` unit tests cover each event's effect and the multi-map
   switch.
2. The headless `MapViewer` tests stay green, including under
   ThreadSanitizer, and the new run-mode tests pass.
3. `monocular_full_slam <corridor1_1024> --rate=0 --viewer` shows the initial
   map: two keyframe frustums, the points and the current camera. Closing the
   window stops playback. If playback finishes first, the window stays open
   until it is closed.
4. `binocular_full_slam … --viewer` shows the stereo stub's single keyframe
   and no points.

## Decisions (from the conversation)

- The view is 3D only. The OpenCV image window stays as it is.
- Option A: a GL-free `SceneState` scene model, drawn by a private Pangolin
  renderer on the thread that runs the viewer loop. There is no public
  renderer interface (YAGNI).
- macOS hedge: the loop can also run on the caller's thread through
  `RunOnCurrentThread()`. Cocoa requires windows and GUI events on the main
  thread.
- Closing the window ends the run, but the app decides, not the library. The
  window stays open after playback until it is closed.

## 1. `SceneState` — the scene model

`SceneState<FE>` (`include/voyahviz/scene_state.h`) has no GL and no threads.
Only the thread that runs the viewer loop touches it.

State:

- `map::MapId current_map_id` (initially 0).
- Map points: `std::vector<geometry::TPoint3DF> point_positions` and
  `std::vector<bool> point_alive`, both indexed by `MapPointId::value`. Ids
  restart at 0 per map, so the storage is dense.
- Keyframes: `std::map<map::FrameId, geometry::RigidTransformation>` (T_cw);
  `FrameId` is ordered (`operator<=>`).
- Current camera: `std::optional<geometry::RigidTransformation> current_pose`
  (T_cw), plus `TrackingStatus status` (`None`, `Tracking`, `Lost`).
- Trajectory: `std::vector<geometry::TPoint3DF>` of camera centres for the
  current map.
- The existing event counters (`applied_count`, `total_applied`).

How each event is applied (after the map rule):

| Event | Effect |
|---|---|
| `MapPointsAdded` | Grow both vectors to `start_id + count`, set the positions and mark them alive. |
| `MapPointsUpdated` | Overwrite the position of each known id. Out-of-range ids are ignored. |
| `MapPointsRemoved` | Set `alive = false` for each known id. |
| `KeyFrameCreated`, `KeyFramePoseUpdated` | Insert or overwrite the keyframe's `T_cw`. |
| `MapCreated` | `current_pose = pose`, `status = Tracking`. |
| `PositionUpdated` | `current_pose = pose`, `status = Tracking`. Append `pose.Inverse().translation()` to the trajectory. |
| `TrackingLost` | `status = Lost`. The scene stays as it is. |
| `InitializationMeasurement`, `NewKeyFrameCandidate`, `LoopDetected` | Counted only. |

Every event is counted, whatever its effect.

Multi-map rule (the same rule as the other consumers):

- `map_id == 0`: no map rule applies; the event is counted only.
- `map_id < current_map_id`: ignored (still counted).
- `map_id > current_map_id`: clear the points, keyframes, trajectory and
  current camera (`status = None`), adopt the new id, then apply the event.
- `map_id == current_map_id`: apply the event.

`MapPointsAdded` is published before the keyframes and `MapCreated`, so it is
the event that triggers the switch.

Const accessors: `current_map_id()`, `point_positions()`, `point_alive()`,
`num_alive_points()`, `keyframes()`, `current_pose()`, `status()`,
`trajectory()`, plus the existing counters.

## 2. `MapViewer` — options, run modes, window lifecycle

```cpp
struct MapViewerOptions {
    double fps = 30.0;          // must be > 0 (std::invalid_argument otherwise)
    bool show_window = false;   // false = headless: drain + scene model only
    int width = 1280;
    int height = 800;
    double camera_size = 0.08;  // frustum depth in map units (median depth = 1 at init)
};

explicit MapViewer(MapViewerOptions options = {});
void Start();                    // the loop on the viewer's own thread (Linux default); no-op if running
void Stop();                     // RequestStop, join, drain (unchanged semantics)
void RunOnCurrentThread();  // the same loop on the caller's thread; blocks until RequestStop() or Stop() (closing the window does not end it); throws std::logic_error if Start() is active
void RequestStop();              // thread-safe; ends either loop
[[nodiscard]] bool window_closed() const;  // thread-safe
void WaitForWindowClose();       // blocks until closed; returns immediately when headless
[[nodiscard]] voyahslam::events::AsyncObserver& inbox();
[[nodiscard]] size_t events_processed() const;
```

- **Loop.** One iteration is a private `Step()`: drain the inbox into
  `SceneState`, then draw one frame if a window is open. The loop paces
  itself to `fps`, with no catch-up burst (as today).
- **Invariant.** Only the thread running the loop touches `SceneState` and
  the GL context. The renderer (window and context) is created at loop entry
  and destroyed at loop exit, on that thread. `Stop()`'s final drain runs
  after the join, as today.
- **Window close.** When the user closes the window, `window_closed()`
  becomes true. The renderer is destroyed, and the loop continues headless
  (draining only) until `RequestStop()` or `Stop()`, so producers' event
  pools never stall.
- **`WaitForWindowClose()`** waits on a `WakeSignal`, which is notified when
  the window closes and when the loop exits.
  - It returns immediately in three cases: `show_window == false`, window
    creation failed, or the window is already closed.
  - It also returns when the loop exits for any other reason, so it can
    never hang.
- **Window creation failure** (for example no `DISPLAY`): the exception from
  Pangolin is caught and logged, `window_closed()` becomes true, and the loop
  runs headless.
- Closing the window never calls `Stop()` from inside the loop, so the
  thread can never try to join itself.

## 3. Rendering — `src/pangolin_renderer.{h,cpp}` (private)

- **Build.** `find_package(Pangolin 0.9 REQUIRED)` in
  `vizualization/CMakeLists.txt`, linked PRIVATE. No Pangolin type appears in
  `include/voyahviz/`. voyahviz is a desktop development tool and is not
  built for Android.
- **World and view.**
  - The world frame is camera 1 of the map: x right, y down, z forward.
  - The initial view looks at the first camera from behind and above, with
    up = −y (as in ORB-SLAM3's viewer).
  - Mouse control uses `pangolin::Handler3D` (orbit, pan, zoom).
- **Drawn items** (legacy GL through Pangolin):

  | Item | Look |
  |---|---|
  | Alive map points | black, point size 2 |
  | Keyframes | blue wireframe frustums |
  | Current camera | green frustum; red when `status == Lost` |
  | Trajectory | thin grey line strip through the camera centres |

- **Frustum.** A pyramid from the camera centre to a rectangle at depth
  `camera_size`, with 4:3 aspect (a nominal shape, not the real field of
  view). A GL-free helper computes it:
  `CameraFrustum FrustumInWorld(const RigidTransformation& T_cw, double size)`
  returns `{centre, corners[4]}` in world coordinates
  (`include/voyahviz/camera_frustum.h`, header-only or `src/` with a test).
- **Side panel** (`pangolin::Var`):
  - Checkboxes: "Show points", "Show keyframes", "Show trajectory" (all on),
    and "Follow camera" (off). With "Follow camera" on, the view follows the
    current camera, or the last keyframe if there is no current camera.
  - Read-only text: map id, alive points, keyframes, status, events
    processed.

## 4. Apps — `examples/full_slam`

- A new flag `--viewer` turns on `show_window`. Without it, the viewer runs
  headless, as today. `--show` (the OpenCV image window) stays independent.
- The playback loop breaks as soon as `viewer.window_closed()` is true.
- When playback ends normally and the window is still open, the app logs
  `Playback finished — close the 3D viewer to exit` and calls
  `viewer.WaitForWindowClose()`. The existing exit path then runs:
  `slam.Stop()` followed by `viewer.Stop()`.

## 5. Testing

| Area | Tests |
|---|---|
| `SceneState` | Each event's effect, using hand-built pooled events: points added, updated, removed (including out-of-range ids), keyframe inserted and updated, `MapCreated` and `PositionUpdated` setting the current pose and the trajectory centre (`T_cw` inverse), `TrackingLost` setting the status. Older map ignored. Newer map clears the scene and adopts the id. Map 0 counted only. Counters unchanged. |
| `FrustumInWorld` | Identity pose gives centre 0 and corners at z = size with 4:3 aspect. A non-trivial `T_cw` gives centre = `T_cw.Inverse().translation()`, and corners are consistent with `T_cw.Inverse()`. |
| `MapViewer` (headless) | Existing tests are updated to `MapViewerOptions`. New tests: `RunOnCurrentThread` stops on `RequestStop()` from another thread and processes events. `RunOnCurrentThread` while `Start()` is active throws. `WaitForWindowClose()` returns immediately when headless. `window_closed()` is false when headless. |
| ThreadSanitizer | `MapViewer*` and `SceneState*`. |
| Manual (last plan task) | Apps with `--viewer` on corridor1_1024 meet success criteria 3 and 4. Record the outcome. |

The GL drawing itself is not unit-tested, because it needs a display.

## 6. Documentation

- `DESIGN.md`:
  - A viewer section: `SceneState` as the scene model built from events and
    its multi-map rule; the loop-thread invariant; the two run modes and the
    macOS rationale; window-close semantics; Pangolin as a private
    dependency.
  - Decisions-log rows for the hedge and for "closing the window ends the
    run, and the app decides".
- `TODO.md`:
  - Remove the Pangolin renderer item, and the hardening point about
    `Stop()` on window close.
  - Add: the macOS setup (playback on a worker thread, viewer and image
    window on the main thread), the 2D initialization overlay, the visual
    extras, and installing Pangolin 0.9 on the second development machine.
