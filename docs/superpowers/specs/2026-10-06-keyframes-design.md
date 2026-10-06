# Keyframes and Map Growth (Tracking Sub-project 2) — Design Spec

Date: 2026-10-06
Status: approved in conversation, pending spec review

## Context and Scope

After the camera-model fix (merged 2026-10-06), mono tracking on corridor1
still loses its map within a few frames. Every map dies at the 30-inlier
local-map gate because nothing ever grows the 62–109-point initial map. In
this codebase a `Frame` is what ORB-SLAM3 calls a KeyFrame: only keyframes
become `Frame` objects. A per-image observation is a *measurement*. Event
names keep the original term (`KeyFrameCreated`).

**User symptom this spec must fix:** "the tracker keeps on losing its
position even in the beginning … we should be able to at least track while
the camera is in the initial room".

**In scope:**

- the Tracker reports every tracked measurement in a rich `FrameTracked` event;
- the Mapper decides when to create a Frame, creates it, adds observations
  and grows the map (stereo close points, triangulation against neighbour
  Frames);
- one atomic `KeyFrameCreated` transaction per Frame insertion, including
  initialization;
- the Tracker's local map built from covisibility, and its reference Frame;
- the viewer draws tracked and visible points.

**Out of scope:**

- **Sub-project 3:** local BA, map point fusion, culling of points and
  Frames, and `ObservationsRemoved`.
- **Sub-project 4:** relocalization, place recognition (DBoW2/FBoW vs Faiss)
  and the post-relocalization Frame guard.
- Parameter tuning on TUM-VI, which happens at the very end.

## 1. Who decides: the Mapper

The Tracker publishes everything it knows about each tracked measurement. The
Mapper, the only map writer, decides whether the measurement becomes a Frame.
Each side holds part of the information:

- **The Tracker** has the tracking quality: matches, inliers and the visible
  points.
- **The Mapper** has the authoritative map:
  - observation counts for the reference ratio;
  - its own backlog;
  - Frames the Tracker's replica has not received yet.

This reverses the current DESIGN split, in which the Tracker decides when a
keyframe *might* be needed and publishes `NewKeyFrameCandidate`. The
candidate protocol (one candidate in flight, rejections, timeouts) is not
needed. The same event also gives the viewer the visible and tracked points
for debugging.

## 2. Events

### 2.1 `FrameTracked<FE>` (Tracker → Mapper, viewer)

It replaces `PositionUpdated`, and `position_updated.h` is deleted. It is
published for every successfully tracked measurement, and it is pooled like
every other event.

| Field | Content |
|---|---|
| `map_id` (base), `timestamp` | as `PositionUpdated` |
| `pose` | T_cw after `OptimizePose` |
| `measurement` | `features::TMeasurement<FE>`, copied (binocular: stereo matches set) |
| `observed_features`, `observed_points` | (feature index, `MapPointId`) pairs, **inliers only** |
| `visible_points` | local-map point ids that pass the projection gate (in front, inside the image, viewing angle), matched or not |
| `reference_frame_id` | the K1 Frame that shares the most inlier points (§5.3) |
| `local_frames` | K1 ∪ K2 Frame ids (§5.2) |
| `stage` | `TrackingStage::{MotionModel, ZeroMotion, LocalMap}`: the step that produced the final pose |
| `num_matches`, `num_inliers` | before and after the final `OptimizePose` |

**Measurement copy.** The measurement is copied into the pooled event, as
`InitializationMeasurement` and `KeyFrameCreated` already do. That is about
80 KB per frame. Sharing it immutably is a TODO.

### 2.2 `KeyFrameCreated<FE>`: one atomic map transaction

Each `KeyFrameCreated` carries everything one Frame insertion changes. The
Tracker's replica applies it in one `ApplyEvent` call, so the Tracker never
tracks against a half-applied insertion.

| Part | Fields |
|---|---|
| Frame | `frame_id`, `pose`, `measurement`, `observed_features` / `observed_points` (the Frame's own observations; existing fields) |
| New points | `new_points_start_id`, `new_positions`, `new_descriptors`, `new_normals` (`ColumnBuffer`s, the layout of today's `MapPointsAdded`); contiguous ids |
| Observations on existing Frames | `added_observations`: `(frame_id, feature_index, map_point_id)`; stereo Frames also `depth`, `right_index` (−1 / −1.0 when absent) |
| Updated points | `updated_ids`, `updated_normals`, `updated_descriptors` (today's `MapPointsUpdated` layout) |

- **Replica order:** new points, then updated points, then observations on
  existing Frames, then the new Frame.
- **Initialization uses the same event.**
  - Mono: event 1 carries Frame 1 with all initial points and Frame 1's
    observations. Event 2 carries Frame 2 and its observations. `MapCreated`
    follows.
  - Stereo: one event, then `MapCreated`.
- **Deleted:** `MapPointsAdded` (no remaining user) and `NewKeyFrameCandidate`.
- **Kept for changes that create no Frame:** `MapPointsUpdated`,
  `MapPointsRemoved`, `KeyFramePoseUpdated` (sub-project 3 and later).
- `Reset()` clears every part and keeps capacity.

## 3. Mapper: deciding

### 3.1 Inbox handling

- `FrameTracked` is considered only in `Mapping` and only for the current map
  (the existing map_id rule).
- **Drain to latest.** A `FrameTracked` is skipped when a newer
  `FrameTracked` for the same map is already in the drained batch. All other
  events keep their order. A `TrackingLost` after a `FrameTracked` clears
  the map, and no Frame is created.
- **Backlog** = the number of `FrameTracked` events skipped since the last
  evaluated one. Backlog 0 means the Mapper is idle.

### 3.2 Signals

| Signal | Definition |
|---|---|
| `ref_points` | points observed by `reference_frame_id` (in the Mapper's map) with ≥ `min_obs` observations; `min_obs` = 2 while the map has ≤ 2 Frames, else 3; a stereo-matched observation counts 2 (amended: ruling H, ORB-SLAM3 `AddObservation`; with a count of 1 a one-Frame stereo map has `ref_points` 0) |
| `since_last` | `timestamp − timestamp of the last Frame` |
| `inliers` | `num_inliers` |
| `close_tracked`, `close_untracked` | stereo only: left features with a stereo match and depth < `close_depth_baselines × baseline`, inside / outside the observed pairs |
| `need_close` | stereo only: `close_tracked < close_tracked_max && close_untracked > close_untracked_min` |

If `reference_frame_id` is no longer in the Mapper's map, the newest Frame
is used instead.

### 3.3 Rule

`bool ShouldCreateFrame(const KeyFrameSignals&, const KeyFrameOptions&)` is a
pure function:

```
c1a = since_last ≥ max_interval
c1b = backlog == 0
c1c = stereo && (inliers < weak_ratio · ref_points || need_close)
ratio = mono ? ratio_mono : (num_frames < 2 ? ratio_stereo_early : ratio_stereo)
c2  = (inliers < ratio · ref_points || need_close) && inliers > min_inliers
create = (c1a || c1b || c1c) && c2
```

`MapperOptions::keyframe` (`KeyFrameOptions`) holds the ORB-SLAM3 defaults:

| Option | Default |
|---|---|
| `max_interval` | 1.0 s |
| `ratio_mono` | 0.9 |
| `ratio_stereo` | 0.75 |
| `ratio_stereo_early` | 0.4 |
| `weak_ratio` | 0.25 |
| `close_depth_baselines` | 40 |
| `close_tracked_max` | 100 |
| `close_untracked_min` | 70 |
| `min_inliers` | 15 |

Each evaluation is logged at debug level with all its signals and the result.

ORB-SLAM3 has a busy-LocalMapping branch (interrupt BA; stereo allowed if the
queue is < 3). It does not apply yet, because nothing long runs in the Mapper.
Interruptible BA is a sub-project 3 TODO.

## 4. Mapper: creating a Frame

`CreateFrame(const FrameTracked&)` runs on the Mapper thread and builds one
`KeyFrameCreated`.

1. **Frame.** It takes `id = next_frame_id_++` and builds the Frame from the
   event's pose and the copied measurement.
2. **Tracked observations.** For each observed pair whose point is alive in
   the Mapper's map:
   - It adds the observation: mono into `observations_`, stereo into
     `stereo_observations_` with depth and right index when stereo-matched.
   - It recomputes the point's normal (the mean unit viewing direction from
     the observing camera centres) and its representative descriptor (the
     observation descriptor with the least median Hamming distance to the
     others). Changed points go into the updated part.
   - The Mapper keeps a **per-point observation index** (point → observation
     rows) so that this and covisibility need no full scan.
3. **Stereo close points** (binocular Frames only):
   - Untracked left features with a stereo match are sorted by depth.
   - A point is created for each with depth < `close_depth`, and at least for
     the `min_close_points` (100) closest.
   - The position is the left camera's back-projection at the stereo depth,
     in world.
   - Each point gets one stereo observation, and its reference is (new
     Frame, feature).
4. **Triangulation against neighbour Frames:**
   - **Neighbours:** the top `neighbours_mono` (30) / `neighbours_stereo`
     (10) Frames by covisibility (shared points); all Frames while there are
     fewer. A neighbour is skipped when the baseline is too small: mono
     baseline / median scene depth of the neighbour < 0.01; stereo baseline <
     the rig baseline.
   - **Matching:**
     - Brute force over features with no observation in either Frame.
     - Descriptor distance ≤ `TH_LOW` (50), best/second ratio 0.6.
     - Epipolar test on **rays**: the angle between the second Frame's ray
       and the epipolar plane (normal n) is ≤ 3σ_level, converted to an angle
       by the exact directional bound `asin(|n·ray_b|) ≤ 3·σ_level·‖(JJᵀ)⁻¹J n‖`
       with J = `Camera::ProjectJacobian` at the unit ray (amended: ruling F;
       was "the norm of `ProjectJacobian`", up to 5.8× too loose at 80°).
     - Rotation-histogram consistency (12 bins of 30°, the init matcher's).
     - One-to-one.
   - **Acceptance:**
     - DLT triangulation on rays.
     - Ray parallax cos < 0.9998. If either feature has a stereo depth whose
       own parallax (the rig baseline seen from that depth) is larger than
       the two-view parallax, the point is instead placed from that stereo
       depth (ORB-SLAM3's stereo fallback). The remaining checks apply
       unchanged.
     - Positive depth in both.
     - Reprojection χ² < 5.991 in raw pixels through each Frame's `Camera`,
       with σ = `sigma_px · level_scale^level`.
     - Scale consistency: the distance ratio dist_b / dist_a must lie within
       the octave ratio `level_scale^(level_a − level_b)` · 1.5·`level_scale`
       and that ratio / (1.5·`level_scale`) (amended: ruling G, ORB-SLAM3
       order; the ratio direction was unspecified).
   - **Result:** each new point has two observations, the new Frame's
     (Frame part) and the neighbour's (`added_observations`). Its reference
     is (new Frame, feature).
5. **Publish** the single `KeyFrameCreated`.

`MapperOptions` gains `sigma_px` and `level_scale`, with the same defaults
and meaning as `TrackerOptions`, plus the triangulation options above
(`neighbours_mono`, `neighbours_stereo`, `min_close_points`, `th_low`,
`nn_ratio`, `min_parallax_cos`). The Mapper validates every incoming point
id with `IsAlive`, because the Tracker's replica may be behind.

**Mixed Frames.** Each Frame stores its observations in the storage that
matches its type. Triangulation between a mono and a stereo Frame uses only
the stereo Frame's left image.

**Knowingly deferred.** Without fusion and culling, duplicate and poor points
accumulate. Tracking tolerates them through the pose outlier rejection.

## 5. Tracker

### 5.1 Replica

- It applies `KeyFrameCreated` atomically (§2.2).
- It keeps the per-Frame observations (existing) and a new per-point list of
  observing Frames.
- Covisibility between two Frames is the number of shared points, computed on
  demand.
- `PointReference` is set from the creating Frame, as today.
- Invariant after every applied event: every observation refers to an
  existing point and Frame, and the two indexes agree.

### 5.2 Local map

The local map is rebuilt for every measurement after the motion-model /
zero-motion step (ORB-SLAM3 `UpdateLocalKeyFrames` / `UpdateLocalPoints`,
without a spanning tree).

- **K1:** the Frames that observe any current match.
- **K2:** for each K1 Frame, its top `local_neighbours` (10) covisible Frames
  not yet included.
- **Cap:** `max_local_frames` (80), K1 first, ordered by shared-point count.
- **Local points:** the alive points observed by the local Frames.

`SearchFromLocalMap` runs over the local points only, through a filtered
point source (the matcher is unchanged). With only the initial map, local
points equal all points, which is today's behaviour.

### 5.3 Reference Frame and the event

- **Reference Frame:** after the final `OptimizePose`, the K1 Frame with the
  most inlier points. `map_frame_id_` is removed.
- **`visible_points`:** the local points that pass the projection gate.

### 5.4 Fast pickup of new points

When a `KeyFrameCreated` arrives whose measurement timestamp equals
`last_frame_`'s, its observations are merged into `last_frame_.matches`. The
next motion-model search then uses the Mapper's new points at once.

### 5.5 Unchanged

- motion model, zero-motion fallback, the 30-inlier local-map gate,
  `OptimizePose`;
- `TrackingLost` and `InitializationMeasurement`.

## 6. Viewer

- `SceneState` consumes `FrameTracked` (pose and trajectory; the current
  tracked and visible point sets; local Frames) and the combined
  `KeyFrameCreated` (points, updated points, Frames).
- Rendering:
  - tracked inliers in a highlight colour;
  - visible but unmatched points in a second colour;
  - local Frames in a distinct frustum colour;
  - everything else grey.
- Only `SceneState` is unit-tested.

## 7. Testing

**Unit tests (synthetic, deterministic):**

- `ShouldCreateFrame`: table-driven over c1a, c1b, c1c, c2, `need_close`,
  the `min_inliers` floor, the early stereo ratio. `ref_points` with
  `min_obs` 2 → 3.
- **Mapper inbox:**
  - 3× `FrameTracked` gives one evaluation, on the newest, with backlog 2.
  - `FrameTracked` followed by `TrackingLost` gives no Frame.
  - An old map's `FrameTracked` is ignored.
- **Frame creation:**
  - **Tracked observations:** they are added, and normals and descriptors
    are recomputed. Dead points are skipped.
  - **Stereo close points:** positions within 1e-9 on a noise-free scene,
    with at least the 100 closest kept.
  - **Triangulation, pinhole and KB fisheye (rays up to 80°):**
    - the new points match ground truth;
    - low-parallax pairs and pairs failing the raw-px χ² test are rejected;
    - the epipolar test on rays accepts true peripheral pairs;
    - the neighbour's half of each new point is in `added_observations`.
- **Replica:**
  - The invariant holds after every event.
  - Atomic application: no tracking between parts, enforced by one event.
  - Initialization: mono uses 2 events, stereo 1.
- **Tracker:**
  - K1/K2 selection, the 80 cap and the reference Frame, on a hand-built
    covisibility map.
  - The local point set equals the local Frames' points.
  - The `last_frame_` merge.
  - `FrameTracked` fields: inlier pairs only, `visible_points` ⊇ the
    observed points, `stage`.
- **End to end (`Slam`, synthetic):**
  - The camera translates and turns 90°, leaving the initial view entirely.
  - Mono and stereo never lose tracking.
  - Frames > 2 and points > the initial count.
  - The trajectory error stays within the noise-derived bound
    (`tests/support/trajectory_error.h`).
- **ThreadSanitizer:** `Tracker*:Mapper*:Slam*`, no warnings.

**Smoke acceptance** (corridor1 1024, Release), stated against the user
symptom:

| Run | Must | Report only |
|---|---|---|
| Mono, 400 frames | The first map survives **≥ 150 consecutive tracked frames** after its initialization; the median map lifetime is **≥ 50 frames** (today about 4) | Frames, points, re-initializations, ATE vs mocap (Sim3) |
| Stereo, 1000 frames | **No tracking loss**, one map | Frames created, points, ATE |

A miss is reported with a diagnosis, and nothing is tuned silently.

## 8. Docs

**DESIGN.md decisions log (2026-10-06):**

- `FrameTracked` replaces `PositionUpdated`.
- The Mapper decides on Frame creation (drain to latest; backlog as
  idleness).
- `NewKeyFrameCandidate` is deleted.
- `KeyFrameCreated` is one atomic transaction, used by initialization too;
  `MapPointsAdded` is deleted.
- The local map is built from covisibility (K1/K2, cap 80).
- Triangulation uses an epipolar test on rays and a raw-px χ².
- Matching for triangulation is brute force for now, with the Faiss vs
  DBoW2/FBoW decision deferred to sub-project 4.

The Tracker, Mapper and event-table sections of DESIGN.md are updated to
match.

**TODO.md:**

- `ObservationsRemoved` (sub-project 3)
- immutable shared measurement in events (`shared_ptr<const>`)
- Faiss vs DBoW2/FBoW spike for place recognition (sub-project 4)
- measure triangulation matching cost; accelerate if needed
- interruptible BA and the busy-Mapper branch (sub-project 3)
- fusion, point and Frame culling, local BA (sub-project 3)
- the relocalization Frame guard (sub-project 4)
