# Map Quality: Fusion, Culling, Local BA — Design Spec

Date: 2026-10-07
Status: approved in conversation, pending spec review
Roadmap: step 1 in `docs/ROADMAP.md` (tracking sub-project 3)

## Context and Scope

After keyframes and map growth, mono tracking on corridor1 keeps one map for
129 frames and then dies at the 30-inlier local-map gate.

**Cause.**
- New Frames stop being created because triangulated points have only 2
  observations, while `ref_points` counts points with ≥ 3 observations. It
  stays near 36.
- Nothing merges duplicates, removes bad points or redundant Frames, or
  refines the map.
- ORB-SLAM3 handles all of this in LocalMapping.

**Goal.** The map stays healthy as it grows:
- fusion merges duplicates and lets points reach ≥ 3 observations;
- culling removes poor new points and redundant Frames;
- local BA refines poses and points around each new Frame.

**Constraints.**
- No locks on map data. The Mapper is the only writer.
- Every Mapper event is an atomic transaction.
- Residuals in raw pixels.
- Tests run under `check` / `check_all`.

**Success, against the user's symptom.**
- Mono, 400 frames of corridor1: one map from the first initialization to
  the end.
- Stereo, 1000 frames: no regression.

**Out of scope.**
- Global BA, loop closing and map merging: roadmap steps 4–5.
- A separate BA thread. It is reserved for global BA.
- Relocalization: roadmap step 3.

## 1. Pipeline

For each created Frame, in the Mapper thread (amended: ruling H, ORB-SLAM3
LocalMapping order; the original "culling before creation" judged fresh
points one Frame short):

1. **The new Frame and its tracked observations** (`ProcessNewKeyFrame`).
2. **Recent-point culling** (§4.1).
3. **New points**, as today: stereo close points, triangulation.
4. **Fusion** (§3).
5. **Local BA** (§5). It is interrupted only by an urgent `FrameTracked`.
6. **Frame culling** (§4.2).

Two transactions are published per created Frame:
- **T1** after steps 1–4: the new Frame, its observations, culled points, new
  points and fusion.
- **T2** after steps 5–6: BA results and culled Frames.

## 2. `MapTransaction<FE>` — the only Mapper event type

It replaces `KeyFrameCreated`, `MapCreated`, `MapPointsUpdated`,
`MapPointsRemoved` and `KeyFramePoseUpdated`, which are deleted. The
Tracker's own events (`FrameTracked`, `TrackingLost`,
`InitializationMeasurement`) are unchanged. Every part may be empty.

| Part | Contents |
|---|---|
| map | `map_id`; `new_map` flag; `start_frame` (the Frame to start tracking from, when `new_map`) |
| Frames added | id, T_cw, measurement, own observations (feature, point) |
| points added | contiguous ids from `new_points_start_id`; positions, normals, descriptors |
| observations added | (frame, feature, point, depth, right_index); depth −1 / right −1 when absent |
| observations removed | (frame, point) |
| points updated | id, position, normal, descriptor |
| points replaced | (loser, winner) |
| points removed | ids |
| Frame poses updated | id, T_cw |
| Frames removed | ids; their observations are removed implicitly |

**Apply order** (Mapper-side writes and every consumer):

1. new map;
2. add Frames;
3. add points;
4. add observations;
5. update points;
6. update poses;
7. remove observations;
8. remove points;
9. remove Frames.

After every transaction:
- every observation refers to an existing Frame and an alive point;
- the covisibility index agrees with the observations;
- every alive point has ≥ 1 observation.

(amended: ruling C) **Every point the Mapper removes is listed in
`removed_points`**, including implicit ones — points left with no observation
after observation removals, Frame culling or BA outliers. Consumers without
observation data (the viewer) rely on it.

**Point references.** When the reference observation of a point (used for
scale prediction) is removed, the new reference is the remaining observation
with the smallest Frame id. Mapper and replica apply the same rule.

**Initialization.**
- Mono: one transaction with `new_map`, both Frames and all points.
- Stereo: one transaction with `new_map`, the one Frame and all points.

**Ids** are never reused, as now. `Reset()` clears every part and keeps
capacity.

## 3. Fusion (ORB-SLAM3 `SearchInNeighbors` / `Fuse`)

**Targets.**
- The new Frame's top covisible Frames: 30 for mono, 10 for stereo.
- Plus each target's top 20 covisible Frames.
- Duplicates and the new Frame are excluded.

**Passes.**
- **Forward:** the new Frame's points are projected into each target.
- **Backward:** the targets' points that the new Frame does not observe are
  projected into the new Frame.

**Gate for one point in one Frame:**
- the Frame does not already observe the point;
- depth > 0 through the Frame's `Camera`, and the projection is inside the
  image;
- ray ≤ 85°;
- cos(view, normal) ≥ 0.5;
- the distance is within the reference-implied scale range ±20 %, which gives
  the predicted level;
- candidates are found in the grid within radius 3 · 1.2^level raw px, at the
  predicted level or one below;
- raw-px χ² < 5.991 (mono) or < 7.815 (stereo, with right x);
- best descriptor distance ≤ 50.

**On a match:**
- **The feature has no point:** add an observation (with depth and right
  index for stereo).
- **The feature has a different point:** merge.
  - The winner is the point with more observations, weighted with stereo
    counting 2 (Ruling H); the lower id wins a tie.
  - The loser's observations move to the winner. In Frames that already
    observe the winner, the loser's observation is dropped.
  - The loser is removed and recorded in "points replaced".

Touched points get their normal and descriptor recomputed.

**Code split.** `features::FuseMatches` (in algs) is a pure function that
returns its decisions. The Mapper applies them. It reuses `ProjectionGate`
and `FeatureGrid`.

**Tracker.** On "points replaced" the Tracker remaps `last_frame_.matches`
from loser to winner.

## 4. Culling

### 4.1 Recent points (ORB-SLAM3 `MapPointCulling`)

**Recent points** are those created in the last 3 Frames. The Mapper counts,
per point, `visible` (in `visible_points`) and `found` (in
`observed_points`). Every `FrameTracked` of the current map updates the
counters, including the ones drain-to-latest skips.

A recent point is removed if either holds:
- `found / visible < 0.25`;
- 2 Frames after its creation, its weighted observations are ≤ 2 (mono) or
  ≤ 3 (stereo).

After 3 Frames a point leaves the recent list.

### 4.2 Frames (ORB-SLAM3 `KeyFrameCulling`)

**Candidates** are the new Frame's covisible Frames. The map's first Frame
and the new Frame are never removed.

A Frame is redundant when ≥ 90 % of its points (stereo: only close points
count) are observed by ≥ 3 other Frames at the same or a finer level (level
≤ its level + 1).

**Removing a Frame:**
- drop its observations;
- remove points left with < 2 observations;
- reassign point references (§2).

## 5. Local BA

**Problem.**
- **Optimized Frames:** the new Frame plus covisible Frames with ≥ 15 shared
  points, capped at 20 by shared count.
- **Points:** all points observed by the optimized Frames.
- **Fixed Frames:** other Frames observing those points. The map's first
  Frame is always fixed when present.

**`BundleAdjust` gains:**
- An optional right-image measurement per observation, plus the rig
  (`std::optional<optimization::StereoCameras>` per Frame), so stereo
  observations use full stereo residuals.
- (amended: rulings A, D) An optional `std::function<bool()> should_stop`
  (null = never), evaluated once before the first iteration and after every
  LM iteration. A stop before iterating leaves the poses unchanged. A flag
  could not work: the Mapper thread itself runs BA, so nobody would set it;
  the Mapper's callback reads the inbox's urgent counter.
- `BaResult` reports `interrupted` and `stage_reached`:
  - 0 = stopped in the robust stage;
  - 1 = stopped in the plain stage;
  - 2 = finished.
  (amended: ruling E) A stop right after the last robust iteration but before
  the outlier deactivation reports stage 0 (outliers kept).

Init BA passes no stop callback and behaves exactly as before.

(amended: rulings I, J) The cap of 20 optimized Frames counts the new Frame.
BA is skipped when no free Frame has an observation or when no Frame would be
fixed (ORB-SLAM3's gauge guard); transaction 2 is still published.

**Iterations:** 5 robust, then 10 plain.

**Applying the result.** Poses and points are always applied. Outlier
observations (χ² > 5.991 mono / 7.815 stereo, or depth ≤ 0) are removed only
when `stage_reached ≥ 1`. Points left with no observations are removed.

**Urgent interruption.**
- The Tracker sets `FrameTracked::urgent` when the final inlier count is
  below `urgent_inlier_factor` (1.5) × `min_local_map_inliers`.
- The Mapper's inbox keeps an atomic count of urgent events: incremented on
  enqueue, decremented when drained. This is event infrastructure, not map
  data.
- The stop callback returns true while the count is > 0, so BA stops within
  one iteration (amended: ruling A). Only `AsyncObserver::TryPop` (and a
  `Listener`) decrement the count; popping from the raw `queue()` bypasses
  it.
- Ordinary `FrameTracked` events never stop BA; drain-to-latest absorbs the
  backlog.

## 6. Options

`MapperOptions::map_quality` holds all thresholds above, with ORB-SLAM3
defaults:

| Group | Values |
|---|---|
| Fusion | neighbours (30 mono / 10 stereo), second order 20, radius factor 3, χ² mono 5.991, χ² stereo 7.815, descriptor distance 50, scale slack 0.2, min cos 0.5 |
| Point culling | window 3 Frames, found ratio 0.25, observations 2 mono / 3 stereo |
| Frame culling | redundancy 0.9, other observers 3 |
| Local BA | min shared 15, max optimized 20, iterations 5 + 10 |

`TrackerOptions::urgent_inlier_factor` = 1.5.

## 7. Testing

**`voyah_algs`:**
- `FuseMatches`: planted duplicates are found; each gate rejects (behind,
  outside, angle, scale, χ², descriptor).
- `BundleAdjust`:
  - stereo edges pass a finite-difference Jacobian check;
  - pinhole stereo matches a hand-built mono-plus-right equivalent;
  - a pre-set stop flag gives `interrupted` and stage 0;
  - a flag set mid-run stops within one iteration;
  - init BA results are unchanged.
- Culling rules as pure functions, tested at their boundaries: 0.25,
  observation counts, 90 %, the first-Frame exemption.

**`voyah_slam`:**
- Mapper:
  - two transactions per Frame, each with the right parts;
  - a merge moves observations and removes the loser;
  - a reference is reassigned when its Frame is culled;
  - the first Frame is never culled;
  - counters are updated from skipped events;
  - an urgent event stops BA.
- Replica:
  - applies every part in the fixed order;
  - the invariant (§2) holds after every transaction;
  - removed points and Frames leave no trace, covisibility included.
- Tracker: `last_frame_` remap on replaced points; the `urgent` flag; the
  urgent counter.
- Viewer: `SceneState` applies transactions, including removals.

**Integration:**
- A synthetic duplicate-producing sequence ends with its duplicates fused and
  observation counts ≥ 3.
- Synthetic mono and stereo leave-the-view runs never lose tracking. Their
  error bounds tighten from 30 % / 0.3 m to the values measured with local
  BA × 1.5.

**ThreadSanitizer** on the slam and integration binaries.

**Smoke on corridor1 (Release):**

| Run | Must | Report |
|---|---|---|
| Mono 400 | One map from the first initialization to the end; Frames created throughout (no stall) | Frames created/culled, points created/culled/fused, local BA ms per Frame and interrupts, Mapper ms per pass, ATE |
| Stereo 1000 | No tracking loss, one map | same |

**Trajectory output.**
- Both apps gain `--trajectory_out <file>`, writing poses in TUM format.
- `tools/ate.py` aligns the output to the corridor1 mocap ground truth (Sim3
  for mono, SE3 for stereo) and reports ATE. Report only.

## 8. Docs

- **`DESIGN.md`:**
  - the pipeline and `MapTransaction` with its apply order;
  - fusion, culling, local BA and the urgent interruption;
  - the decisions log (2026-10-07): all Mapper events are transactions; two
    transactions per Frame; local BA in the Mapper thread, interruptible only
    by urgent events.
- **`TODO.md`:** global BA in a separate thread (step 4); tuning (step 6).
- **`ROADMAP.md`:** step 1 marked done.
