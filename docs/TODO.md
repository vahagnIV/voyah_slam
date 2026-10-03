# TODO

Concrete pending work. Reasoning and background live in `docs/DESIGN.md`
(referenced as "OQ n" = Open Question n). Remove items when done; add new ones
as soon as they come up.

## Performance

- [ ] **Profile 8-bit vs float working images.** With OpenCV 5.1, 8-bit is
      ~8% slower than float (≈ 76 vs 70 ms per mono 1024² frame). Suspects:
      per-pixel `std::round` in `BilinearInterpolation`, 8-bit → float
      conversion per tap in `images::Filter`. Profile both variants before the
      extractor rework. (OQ 21)
- [ ] Benchmark on TUM-VI **512×512** (`dataset-corridor1_512_16`, being
      downloaded) — the resolution ORB-SLAM3's configs and timings use.
- [ ] Optionally build ORB-SLAM3 against OpenCV 5.1 with `REGISTER_TIMES` and
      run it on the 512 dataset as a reference.

## ORB extractor (OQ 21)

- [ ] Own FAST-9-16, run once per pyramid level; assign corners to cells
      (≥ 20 if any, else ≥ 7). Removes OpenCV from detection and the blind
      strips at cell borders.
- [ ] Preallocate blur and pyramid buffers (no per-frame image allocations).
- [ ] Round rotated descriptor pattern offsets (ORB-SLAM3 uses `cvRound`; we
      truncate).
- [ ] Keep each keypoint's pyramid level (and size) in `ImageFeatures`.
- [ ] Check border handling (ORB-SLAM3 pads levels by 19 px).
- [ ] Remove the unused `cv::minMaxLoc` per cell.
- [ ] Parallel left/right extraction for stereo.

## Dataset / preprocessing

- [ ] Contrast equalization (own CLAHE) as an app-side preprocessing step —
      ORB-SLAM3 applies CLAHE(3.0, 8×8) to every TUM-VI image.
- [ ] Radial-tangential distortion model for EuRoC MAV. (OQ 23)

## Geometry

- [ ] Bearing vectors instead of z = 1 normalized points (full fisheye FOV);
      interim 85° cut-off in place. Decide with the Tracker design. (OQ 22)
- [ ] Stereo matching in `BinocularMeasurement` (depths / right indices are
      placeholders).

## Map & events

- [ ] `MapPointStorage`: stop reusing freed slots (ids are never reused);
      decide storage-growth strategy. (OQ 2)
- [ ] Make `FrameId` allocation monotonic.
- [ ] Size event pools per event type. (OQ 14)
- [ ] `NewKeyFrameCandidate`: add the tracked `(map_point_id, feature_idx)`
      observations.

## Threads

- [ ] Component loops: threads, lock-free idle wait (OQ 20), Tracker
      measurement policy (OQ 19).
- [ ] Design the Tracker, then the Mapper, then the LoopCloser.

## Visualization

- [ ] Pangolin renderer in `MapViewer::Render()`; decode event payloads in
      `SceneState` (map points, keyframe poses, current pose, trajectory).
- [ ] Event serialization + transport (HTTP or ROS) for a remote viewer: a
      serializing async observer on the SLAM side, a receiver feeding
      `SceneState` on the viewer side.
- [ ] Separate image flow for the camera view (images/keypoints are outside the
      SLAM events).
- [ ] `MapViewer` hardening (deferred review minors, 2026-10-03):
      `Start()`/`Stop()` aren't safe to call concurrently or from the viewer
      thread (matters once a Pangolin window-close calls `Stop()`); `Start()`
      leaves `running_` true if thread creation throws; reject non-finite /
      extreme `fps`, and wake `Stop()` immediately (low fps delays it by a
      period); exceptions from future `Apply()`/`Render()` would terminate.
- [ ] `SceneState`: counter array sized from the last `EventType` enumerator;
      add a count sentinel / `static_assert` and bounds-check types (needed once
      events are deserialized from the network).

## Event system

- [ ] `AsyncObserver` destruction doesn't acknowledge events still queued, so
      their pool slots never return. Drain on destruction.
- [ ] Mono/stereo variants of `KeyFrameCreated` / `NewKeyFrameCandidate` share
      one `EventType`. (OQ 24)

## Environment

- [ ] Second dev machine: build OpenCV 5.x from source into `/usr/local` with
      `WITH_GTK=ON` (Conan no longer provides OpenCV).
