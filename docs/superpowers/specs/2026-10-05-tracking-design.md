# Tracking Against the Map (Sub-project 1) — Design Spec

Date: 2026-10-05
Status: sections 1 and the structure approved in conversation; sections 2–4
written directly into this spec at the user's request — pending spec review

## Context and Scope

Tracking was split into four sub-projects. This spec covers the first:

1. **Tracking against the map**
2. Keyframes and map growth
3. Local mapping quality (local BA, culling)
4. Relocalization and reference-keyframe tracking (place recognition)

Sub-projects 2–4 are deferred; the user has their own ideas for them.

Today `tracking::PoseEstimator` is a stub. It succeeds whenever a frame has
≥ 50 features and returns the last pose. The Tracker's map replica is a no-op.
This spec replaces both with ORB-SLAM3-style frame-to-map tracking.

**In scope**

1. `tracking::MapReplica<FE>`, built from Mapper events.
2. `features::` projection matching (`SearchByProjection`): from the last
   frame, and from the local map.
3. `optimization::OptimizePose`: motion-only g2o, with mono and stereo
   residuals.
4. A Tracker flow that uses them and deletes the `PoseEstimator` stub. It
   publishes real `PositionUpdated` and `TrackingLost`.
5. Tests, smoke runs and docs.

**Out of scope (→ sub-projects 2–4 and TODO.md)**

- Keyframe decision and `NewKeyFrameCandidate`.
- Mapper keyframe insertion and new points.
- Local BA and culling.
- Bag-of-words reference-keyframe tracking.
- Relocalization.

Without map growth, tracking on a real sequence lasts only while the initial
map's points stay in view. This is expected.

**Success criteria**

1. **Synthetic tracking accuracy.** On a synthetic sequence, tracking after
   initialization stays within the following errors of ground truth, frame by
   frame, over the frames where enough map points are visible:
   - Mono: compared after aligning the map's scale. Rotation < 0.5°;
     translation < 2 % of the travelled distance.
   - Stereo: metric. Rotation < 0.5°; translation < 2 cm.
2. **Loss.** A frame with no features, or with a pose far from the map
   (e.g. a teleport), gives `TrackingLost`. Re-initialization then works
   end to end.
3. **corridor1 (mono and stereo).** After initialization, the viewer shows the
   current camera moving and a trajectory. The log reports tracked
   frames/inliers until the map runs out. Tracking is then lost, and the app
   re-initializes.
4. **Threading.** ThreadSanitizer is clean. All existing behaviour around the
   state machine is preserved.

## Decisions (from the conversation)

- **Decomposition:** four sub-projects; this is the first.
- **Stereo residuals:** full stereo residuals in pose optimization, using the
  right camera via `T_rl` and normalized coordinates; not the rectified
  `uR = uL − bf/z` shortcut.
- **Pose optimization:** a new `optimization::OptimizePose` with unary g2o
  edges (approach A). It is not `BundleAdjust` with fixed points, and it is
  not a hand-written solver.
- **Separate units:** projection matching and the replica are separate,
  testable units.
- **No BoW fallback.** Without a vocabulary, ORB-SLAM3's
  `TrackReferenceKeyFrame` is replaced by a wide-window projection search from
  the last frame at zero motion.

## 1. Map replica and tracking flow

**`tracking::MapReplica<FE>`** (`include/voyahslam/tracking/map_replica.h`).
It is touched only on the Tracker thread. It applies Mapper events with the
map-id rule:

- map 0 is exempt;
- older maps are ignored;
- a newer map clears the replica and adopts the id.

| Event | Effect |
|---|---|
| `MapPointsAdded` | Points at `start_id + i`: position, descriptor, normal, alive. |
| `MapPointsUpdated` | Overwrite position/descriptor/normal of known ids; out-of-range ids are ignored. |
| `MapPointsRemoved` | Mark known ids dead. |
| `KeyFrameCreated` | Store the keyframe: id, `T_cw`, a copy of the measurement, and the observations (feature → point). For each observed point without a reference, record the reference observation: keyframe id, feature index, the feature's pyramid level, and the distance from that keyframe's camera centre to the point. |
| `KeyFramePoseUpdated` | Update the keyframe's `T_cw`. |
| `MapCreated` | No replica change beyond the map-id rule. |

Every event is also counted (as in `SceneState`).

**Accessors:**

- `map_id()`, `num_points()`, `IsAlive(id)`
- `Position(id)`, `Descriptor(id)`, `Normal(id)`
- `Reference(id)` → `{keyframe, feature, level, distance}` or nullopt
- `keyframe(FrameId)`
- `keyframe_ids()`

**Tracker state:**

- `last_frame_`: the measurement, `T_cw`, and the inlier matches
  (feature → `MapPointId`).
- `velocity_`: an optional `T_cur_last`.
- On `MapCreated` (Tracking entered), `last_frame_` is that keyframe's replica
  entry, with its observations as matches. `velocity_` is reset.

**Per measurement in `Tracking`** (stereo matching has already run on
dequeue):

1. **Motion model,** if `velocity_` is set.
   - Predict `T_cw = velocity · T_last`.
   - Run `SearchByProjection` from the last frame. Window: 15 px for mono,
     7 px for a binocular measurement, × the scale of the candidate's level
     (`level_scale^level`).
   - If fewer than 20 matches, clear the matches and retry with the window ×2.
   - `OptimizePose`, then drop the outlier matches. The step fails if fewer
     than 10 inliers remain.
2. **Zero-motion fallback,** if step 1 failed or there is no velocity. Same
   as step 1 with the predicted pose = `T_last` and the window ×4. Same
   gates. If it fails, the frame is lost.
3. **Local map.** Run `SearchByProjection` from the replica: every alive point
   not already matched that passes the visibility test (§2), with
   `th = 1`. Then `OptimizePose` on all matches. Success needs ≥ 30
   inliers.
4. **Success.**
   - Publish `PositionUpdated` with the optimized `T_cw`. `frame_id` stays the
     `MapCreated` keyframe id, as today; it is documented.
   - `velocity_ = T_cur · T_last⁻¹`.
   - `last_frame_` = this frame with its inlier matches.
   - Info log, at most once per second: tracked frames, inliers.
   - Debug log per frame: matches and inliers per step.
5. **Failure.** `TrackingLost` and state `Lost`, as today. The debug log says
   which step failed and with how many matches/inliers.

**Unchanged:**

- `WaitingForMap` and `Lost` keep forwarding `InitializationMeasurement`.
- No `NewKeyFrameCandidate` yet.

`PoseEstimator` (header, source, test) is deleted. `TrackerOptions` gains
`tracking` (all thresholds above, with these defaults) and `level_scale`
(1.2). `StereoMatcherOptions` also has a `level_scale`; the two are separate
on purpose, though both default to 1.2.

## 2. Projection matching (`include/voyahslam/features/projection_matcher.h`)

The matcher is a header-only template on the extractor. It works in
undistorted pixel coordinates: `x_px = K · (Xc / Xc.z)`, the same space as
`measurement.points()`. For a binocular measurement, the left image is
searched, and right matches come from the stereo matching already done.

```cpp
struct ProjectionMatcherOptions {
    double max_distance_last = 100.0;   // ORB-SLAM3 TH_HIGH for frame-to-frame
    double max_distance_map = 100.0;    // TH_HIGH for local map
    double nn_ratio_map = 0.8;
    bool check_orientation = true;      // frame-to-frame only, histogram as in the init matcher (12 bins)
    double level_scale = 1.2;
    int max_level = 7;                  // ORB 8 levels
    double max_ray_cos = std::cos(85°); // reject projections beyond the 85° limit
};
```

**`SearchFromLastFrame(current, T_cw_current, last, last_matches, replica,
radius_px, options) → std::vector<int64_t> matches`**

- `matches` is indexed by current feature; −1 means unmatched. It holds the
  `MapPointId`.
- For each alive point matched in the last frame:
  - Project it into the current frame. Skip it if `z ≤ 0`, if it is beyond
    85°, or if it falls outside the measurement's point bounding box.
  - Search candidates via `FeatureGrid` within `radius_px · level_scale^l`.
    `l` is the last frame's feature level; levels are restricted to
    `[l−1, l+1]` (ORB-SLAM3's forward/backward test is not needed without
    pure-translation prediction).
  - Take the best descriptor distance ≤ `max_distance_last`. Claims are
    one-to-one: the closer claim wins.
- Then apply the rotation-consistency histogram (same 12-bin, three-maxima
  rule as `MatchForInitialization`).

**`SearchFromLocalMap(current, T_cw, replica, out_matches, th, options) →
size_t added`**

- Fills `out_matches`, which may already contain motion-step matches.
- For each alive point not already matched, run the visibility test:
  - `z > 0`, and within 85°;
  - viewing cosine with the normal, `cos(view, normal) > 0.5`;
  - a predicted level from the reference observation:
    `l = clamp(level_ref + round(log(dist_ref / dist) / log(level_scale)), 0, max_level)`.
- Radius = `th · (cos(view, normal) > 0.998 ? 2.5 : 4.0) · level_scale^l` px.
  Candidates are restricted to levels `[l−1, l]` (ORB-SLAM3).
- Accept the best match if it is ≤ `max_distance_map`, and either the best
  and second best are on different levels, or best < `nn_ratio_map ·
  second`. Only unmatched features are candidates.
- Returns the number of matches added.

## 3. Motion-only pose optimization (`include/voyahslam/optimization/pose_optimization.h`)

The public header has no g2o.

```cpp
struct PoseObservation {
    geometry::TPoint3DF point;               // world, fixed
    geometry::TPoint2DF left;                // undistorted normalized (z = 1)
    std::optional<geometry::TPoint2DF> right;// normalized in the right camera (stereo match)
    double sigma;                            // normalized units (sigma_px · level_scale^level / f)
};
struct PoseOptimizationOptions {
    int rounds = 4;
    int iterations_per_round = 10;
    double mono_chi2 = 5.991;    // 2 dof
    double stereo_chi2 = 9.488;  // 4 dof (left + right normalized)
};
struct PoseOptimizationResult {
    bool success = false;
    geometry::RigidTransformation T_cw;
    std::vector<bool> inlier;    // per observation, from the last round's classification
    size_t num_inliers = 0;
};
[[nodiscard]] PoseOptimizationResult OptimizePose(
    const geometry::RigidTransformation& initial_T_cw,
    const std::optional<geometry::RigidTransformation>& T_rl,   // set iff any observation has `right`
    const std::vector<PoseObservation>& observations,
    const PoseOptimizationOptions& options);
```

**Edges.** One `VertexSE3Expmap` (`T_cw`) and unary edges, defined only in
`src/optimization/pose_optimization.cpp`:

- **Mono:** `e = π(T_cw·X) − left`. Information `I/σ²`.
- **Stereo:** 4 rows, `[π(T_cw·X) − left; π(T_rl·T_cw·X) − right]`.
  Information `I₄/σ²`.

The Jacobians come from `projection.h`: a pose-only variant of
`ProjectionJacobians`. The right rows chain through `T_rl`, because the left
perturbation of `T_cw` maps through `R_rl`. A new helper,
`ProjectionJacobianRightCamera`, is tested against finite differences.

**Schedule (ORB-SLAM3 `PoseOptimization`).** Each round:

1. Reset the pose to its current estimate, then run `iterations_per_round`
   iterations with a Huber kernel (δ = √χ² of the edge type).
2. Classify every edge at the result: χ² > threshold or depth ≤ 0 → outlier
   (level 1, excluded); else inlier (level 0).
3. Round 3 (0-based index 2) and later run without the kernel.

The function reports the last classification. `success = num_inliers ≥ 3`,
and the pose is finite.

**Tracker side.** Observations are built from matches:

- `left` = the current measurement's normalized point (left camera).
- `right` = the right camera's normalized point of `right_indices[i]`, when
  stereo-matched.
- `sigma = sigma_px · level_scale^level / K(0,0)`, with
  `TrackerOptions::sigma_px = 1.0`.

## 4. Testing, verification, docs

**Test support:** the synthetic sequence generator (mono) and a stereo
sequence variant: the stereo frame generator applied along the mono
trajectory. Both give ground-truth `T_cw` per frame, with world = frame 0.

| Area | Tests |
|---|---|
| `MapReplica` | Each event's effect. Reference observation recorded from the first observing keyframe (distance, level). Map-id rule: newer clears, older ignored, map 0 ignored. Out-of-range ids ignored. |
| Projection helper | `ProjectionJacobianRightCamera` matches finite differences (1e-6). |
| `OptimizePose` | Mono, noise-free: a perturbed pose recovers ground truth (1e-6). Stereo, noise-free: same, and the right rows constrain depth (metric translation recovered). 0.5 px noise plus 20 % outliers: outliers flagged ≥ 95 %, rotation < 0.2°, translation within tolerance. Too few observations → `success = false`. Invalid input → `success = false`. |
| `ProjectionMatcher` | Hand-built: projection inside/outside the window; level restriction; one-to-one claims; ratio/level rule (local map); visibility (behind camera, viewing angle, 85°); rotation histogram drops an inconsistent match. Synthetic: frame-to-frame precision ≥ 98 %; local map precision ≥ 95 %. |
| Tracker | Synthetic mono and stereo sequences, driven synchronously through the Tracker with the Mapper's events replayed from a real Mapper: tracked `T_cw` meets success criterion 1. A 0-feature frame and a teleported frame give `TrackingLost`. |
| Existing tests | Tracker tests that publish `MapCreated` without points (stub-era) are rewritten. Either they build a replica with points from a synthetic map, or they assert the new behaviour (no points → lost). The end-to-end `Slam` tests (mono and stereo) keep their shape, use the synthetic sequences, and assert `PositionUpdated` poses near ground truth. |
| ThreadSanitizer | `Tracker*:Slam*:Mapper*`. |
| Smoke runs | corridor1 mono `--max-frames=400` and stereo `--max-frames=200`. Record: init frame, number of tracked frames, typical inliers, the frame where tracking is lost, and whether re-initialization happens. Optional viewer screenshot. |

**Docs**

- `DESIGN.md`:
  - The Tracker section: replica, tracking flow, thresholds.
  - Projection matching.
  - `OptimizePose`, with mono/stereo edges.
  - The BoW-less fallback.
  - Decisions-log rows dated 2026-10-05.
  - The deleted stub.
- `TODO.md`:
  - Remove "Real PoseEstimator" and "Apply map diffs to the Tracker replica".
  - Add sub-projects 2–4 as items: they are deferred, and the user has
    designs in mind.
  - Add the BoW-less fallback limitation.
