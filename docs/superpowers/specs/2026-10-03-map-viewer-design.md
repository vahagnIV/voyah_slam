# Map Viewer — Design Spec

Date: 2026-10-03
Status: approved in conversation, pending spec review

## Context and Goal

voyahslam's threads communicate only through events (DESIGN.md §2, §6). The
visualizer is another consumer of those events. In the long run it runs **on a
different machine**: on the SLAM machine an async observer serializes events
and sends them over HTTP or ROS; on the viewer machine a receiver rebuilds the
events and feeds the visualizer.

This step builds the in-process version of that consumer, structured so the
part that consumes events is unchanged when it moves to the remote side:

- `MapViewer` subscribes (one `AsyncObserver` inbox) to the Tracker and the
  Mapper in the `full_slam` example apps, runs on its own thread, and drains
  and acknowledges every event.
- `SceneState` is the event-driven model the viewer will draw. For now it only
  recognizes and counts events.
- Rendering is an empty hook; Pangolin (3D map points, keyframes, current
  pose) comes later.

**Success criteria**

1. Both `full_slam` apps run with a `MapViewer` attached to Tracker and Mapper.
2. Every event delivered to the viewer is acknowledged (pool slots come back),
   including events still queued when the viewer stops.
3. `SceneState` depends only on event contents — no access to `Slam`,
   components or shared memory — so it can be fed by a deserializer later.
4. A clear place exists for the Pangolin renderer.

**Out of scope:** serialization and transport, Pangolin rendering, the camera
image view (stays the main-thread `--show` tool; acquisition and feature
extraction are outside the SLAM logic, so images get a separate flow later),
fixing the event-system issues listed under Known Issues.

## Constraints Carried From the Conversation

- **Viewer-only data:** the viewer uses nothing but event contents.
- **Pointer rule for events:** an event may point only at data that never
  changes and lives for the whole run (calibration, configuration — e.g. a
  `DistortionModel` or `StereoRig`). The serializer turns such references into
  values (inline, or sent once and referenced). Everything mutable or
  short-lived travels by value.
- **No OpenCV in voyahviz public headers** (DESIGN.md principle 8). The viewer
  uses no OpenCV at all.
- **Decisions taken:** viewer on its own thread (option A); respect the event
  boundary now, serialize later (option A); structure = `SceneState` +
  `MapViewer` (approach A) with a single inbox.

## Components

All in `vizualization/` (library `voyahviz`, namespace `voyahviz`).

### `SceneState<TFeatureExtractor>` — `include/voyahviz/scene_state.h`

Plain data + logic. No threads, no GUI, no synchronization (only the viewer
thread touches it).

```cpp
template <typename TFeatureExtractor>
class SceneState {
public:
    // Updates the scene from one event. Dispatches on event.type.
    void Apply(const voyahslam::events::Event& event);

    // Number of applied events of `type`.
    [[nodiscard]] size_t applied_count(voyahslam::events::EventType type) const;
    // Number of applied events of all types.
    [[nodiscard]] size_t total_applied() const;
};
```

- Templated on `TFeatureExtractor` because Mapper events are
  (`MapPointsAdded<FE>`, `MapPointsUpdated<FE>`); reading their payloads later
  needs the concrete type. Defined in `src/scene_state.cpp`, explicitly
  instantiated with `INSTANTIATE_CLASS` (ORB, SIFT).
- Now: a `switch` over all `EventType` values that increments a per-type
  counter. No `static_cast` to concrete types yet.
- Later (Pangolin step, not now): map point positions in a vector indexed by
  `MapPointId` with a valid flag (ids are never reused), keyframe poses by
  `FrameId`, current pose and trajectory.

### `MapViewer<TFeatureExtractor>` — `include/voyahviz/map_viewer.h`

Owns the inbox, the thread, the `SceneState`, and the render hook.

```cpp
template <typename TFeatureExtractor>
class MapViewer {
public:
    explicit MapViewer(double fps = 30.0);
    ~MapViewer();  // calls Stop()
    MapViewer(const MapViewer&) = delete;
    MapViewer& operator=(const MapViewer&) = delete;

    void Start();
    void Stop();

    // Connect producers with AddObserver(viewer.inbox()) before they start.
    [[nodiscard]] voyahslam::events::AsyncObserver& inbox();
    // Safe to read from any thread.
    [[nodiscard]] size_t events_processed() const;

private:
    void Run();
    void DrainInbox();  // apply + acknowledge everything queued
    void Render();      // empty until Pangolin
};
```

- Members: `AsyncObserver inbox_`, `SceneState<FE> scene_`,
  `std::atomic<bool> running_`, `std::atomic<size_t> events_processed_`,
  `std::thread thread_`, frame period from `fps`.
- Defined in `src/map_viewer.cpp`, `INSTANTIATE_CLASS` (ORB, SIFT).

## Thread Lifecycle and Data Flow

```
Tracker ─┐ AddObserver(viewer.inbox())
         ├──► inbox (one AsyncObserver, moodycamel queue)
Mapper  ─┘                │
                          ▼  viewer thread
              Run(): while running_:
                       DrainInbox()  → Listener::GetNextEvent() until null
                                       scene_.Apply(*e); ++events_processed_
                                       (Listener acknowledges previous event)
                       Render()      → no-op
                       sleep_until(next_tick); next_tick += period
                       (if already late: next_tick = now + period, no catch-up)
```

- `DrainInbox()` creates a local `Listener` per call; its destructor
  acknowledges the last event of the drain, so nothing stays borrowed between
  ticks.
- `Start()`: no-op if already running; sets `running_` and spawns `Run()`.
- `Stop()`: idempotent. Clears `running_`, joins the thread, then calls
  `DrainInbox()` once more on the calling thread so events queued before the
  producers stopped are acknowledged (otherwise their pool slots never return).
- **Required ordering in apps:** wire before `slam.Start()`; stop `Slam` before
  the viewer (`slam.Stop(); viewer.Stop();`) so nothing arrives after the final
  drain.
- Per-producer FIFO is preserved by the queue; no ordering between Tracker and
  Mapper events is needed by the viewer.
- No busy-waiting: the loop sleeps to absolute deadlines (like
  `PlaybackClock`), so it doesn't depend on Open Question 20 (lock-free idle
  wait).

## App Integration (`examples/full_slam/`)

Both `monocular_full_slam` and `binocular_full_slam`:

```cpp
voyahviz::MapViewer<TFeatureExtractor> viewer;
slam.tracker().AddObserver(viewer.inbox());
slam.mapper().AddObserver(viewer.inbox());
viewer.Start();
slam.Start();
... dataset loop ...
slam.Stop();
viewer.Stop();
std::cout << "viewer: " << viewer.events_processed() << " events processed\n";
```

The viewer is always attached (cheap while rendering is a no-op); a flag can
come with Pangolin. `--show` is unchanged. Until Tracker/Mapper publish, the
count is 0.

## Testing

Headless gtests in `vizualization/tests/`, using real events and pools from
voyahslam and an `Observable` as a stand-in producer.

- **SceneState:** applying events of several types increments the matching
  `applied_count` and `total_applied`.
- **MapViewer:**
  - An event published (via `Observable` → `viewer.inbox()`) before `Start()`
    is processed after `Start()`; a **1-slot pool** proves acknowledgement
    (`TryCreate()` succeeds again only when the slot returned).
  - `Stop()`'s final drain acknowledges an event published right before
    `Stop()`.
  - Fan-in: the viewer connected to two `Observable`s receives events from both
    (`events_processed()` == total published).
  - Destructor of a running viewer stops it cleanly; `Stop()` twice is safe.
- Concurrency tests also run under ThreadSanitizer (as for the event pool).

## Known Issues Surfaced (recorded, not fixed here)

1. **Shared event tag for mono/stereo variants.** `KeyFrameCreated<FE,
   TMeasurement>` and `NewKeyFrameCandidate<FE, TMeasurement>` use one
   `EventType` for both measurement types, so a consumer holding `Event*` can't
   pick the concrete type. Options: separate tags or a measurement-kind field.
   Related to mixed keyframes (DESIGN.md OQ 17). → new open question.
2. **No subscription filtering.** An `Observable` delivers every event to every
   observer, so the viewer also receives `NewKeyFrameCandidate` (with the full
   measurement) and just acknowledges it. Fine in-process; the future
   serializing observer must choose what to send.
3. **`AsyncObserver` destruction leaks queued events.** Events left in its
   queue are never acknowledged, so their pool slots never return. General
   event-system issue → `TODO.md`.

## Documentation Changes (part of the implementation)

- `DESIGN.md`: remote visualization architecture (serializing observer →
  transport → receiver feeding `SceneState`); the event pointer rule; a Map
  Viewer section (components, lifecycle, stop ordering); open question for the
  shared mono/stereo event tag; the no-filtering note; decisions log entries.
- `TODO.md`: event serialization + transport (HTTP/ROS); separate image flow
  for the camera view; `AsyncObserver` destructor leak; shared event tag;
  Pangolin renderer.
