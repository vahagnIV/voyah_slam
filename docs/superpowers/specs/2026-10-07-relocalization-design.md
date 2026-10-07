# Relocalization and Reference-Frame Tracking — Design Spec

Date: 2026-10-07
Status: approved in conversation, pending spec review (amended with the stereo-drift spike result)
Roadmap: step 3 in `docs/ROADMAP.md`

## Context and Scope

Today a tracking failure throws the map away:

1. The Tracker goes to `Lost` and publishes `TrackingLost`.
2. The Mapper clears the map.
3. Initialization restarts.

Before declaring loss, the only fallbacks are the motion model and a ×4
zero-motion window. There is no BoW-based reference-Frame tracking, no
place recognition, and no PnP solver. Since step 2, the Mapper computes BoW
data per Frame, but it never publishes it.

**Goal.** A short loss must not cost the map.
- The Tracker first tries reference-Frame tracking.
- If that fails, it relocalizes against a BoW Frame database, with PnP RANSAC
  on rays.
- Only after a timeout does it give up. A new map starts, as today, until
  multi-map (step 5).

**Constraints.**
- The Tracker never mutates map data.
- Threads talk only through events.
- Residuals are in raw pixels, with σ = sigma_px·1.2^level.
- No vocabulary means behaviour exactly as today.

**Stereo drift with `--vocabulary` (amended: spike result,
`.superpowers/spikes/2026-10-07-stereo-drift.md`).**

*What happens.*
- The damage is not gradual drift. It happens in fast-rotation stretches
  (corridor1 frames 324–378).
- There the motion model is accepted with only 10–29 inliers, so the pose is
  wrong.
- `need_close` turns that frame into a Frame whose roughly 320 stereo close
  points carry the wrong pose.
- With BoW the Mapper publishes it before the next frame is tracked. The
  Tracker locks onto the misplaced points, and the run either jumps for good
  or is lost.

*Fix (§0 below).* Gate the motion-model result at 30 inliers, so the
existing zero-motion fallback runs instead.

*Measured with `--vocabulary`, stereo 1000:*
- 0.0361, 0.0363 and 0.0369 m, one map each;
- previously 0.056 and 0.189 m, and two lost runs today.

*Measured mono 400:* 0.032 m, unchanged.

## 0. Motion-model inlier gate (prerequisite)

- `TrackingOptions` gains `min_motion_model_inliers` = 30.
- A motion-model pose with fewer final inliers counts as failed, so
  zero-motion (whose minimum stays 10), then reference-Frame tracking (§3)
  run next.
- **Smoke acceptance:**
  - stereo 1000 with `--vocabulary`: ATE ≤ 0.04 m and one map in each of 2
    runs;
  - mono 400: no regression.

**Out of scope.**
- Relocalization into other maps (step 5).
- Loop detection (step 4).
- IMU.

## 1. BoW data in the Tracker

- `events::NewFrame` (inside `MapTransaction`) gains `bow::BowVector bow`
  and `bow::FeatureVector features`. The Mapper fills them from its existing
  per-Frame computation. They stay empty when there is no vocabulary.
- The Tracker's `MapReplica` stores them with each Frame.
- The Tracker receives the same `std::shared_ptr<const bow::Vocabulary<FE>>`
  as the Mapper, through `Slam`. It uses it only for the current frame's BoW
  at loss time.

## 2. States and loss protocol (ORB-SLAM3, visual only)

```
Tracking ─(motion model and zero-motion fail)→ reference-Frame tracking ─fail→ RecentlyLost
RecentlyLost: relocalize every measurement
   success  → Tracking (same map; velocity reset; matched Frame is the reference)
   timeout  → Lost
Lost: TrackingLost(final = true) → Mapper clears map → initialization (as today)
```

- **`TrackerState` gains `RecentlyLost`.**
  - A map with ≤ `min_map_frames_to_relocalize` (10) Frames goes directly to
    Lost.
  - The timeout is `recently_lost_seconds` (5.0), measured on measurement
    timestamps.
- **While RecentlyLost,** the Tracker publishes no `FrameTracked` and no
  `InitializationMeasurement`. The Mapper's map stays intact.
- **`TrackingLost` gains `bool final`.**
  - `false` is published on entering RecentlyLost. The Mapper ignores it,
    and the viewer shows "recently lost".
  - `true` is published on entering Lost. Only this one clears the Mapper's
    map.
- **`TrackingStage` gains `ReferenceFrame` and `Relocalization`.**
- **Without a vocabulary,** RecentlyLost is skipped (direct to Lost, as
  today).
- **Options:** `TrackerOptions::relocalization` holds
  `recently_lost_seconds` = 5.0 and `min_map_frames_to_relocalize` = 10.

## 3. Reference-Frame tracking (ORB-SLAM3 `TrackReferenceKeyFrame`)

It runs after zero-motion fails and needs a vocabulary.

1. Compute the current frame's BoW data (`levels_up` = 4).
2. Run `SearchByBoW` against the reference Frame with ratio 0.7. Fewer than
   15 matches is a failure.
3. Run `OptimizePose` from the last frame's pose. Drop outliers; at least 10
   inliers must remain.
4. Run the usual local-map step; its 30-inlier gate decides.
5. On success, publish with stage `ReferenceFrame`.

## 4. Relocalization (ORB-SLAM3 `Relocalization`)

**4.1 Frame database.**
- `bow::FrameDatabase` (algs) is an inverted index from word to Frames.
- `Add(frame, bow)` and `Remove(frame)` are driven by the replica's
  transactions.
- `Query(bow)`:
  - finds the Frames sharing at least one word;
  - keeps those with `common ≥ 0.8 · max common`;
  - returns `(frame, common, L1 score)`.

**4.2 Candidates.**
- A candidate's group score is its own score plus the scores of its top 10
  covisible Frames (replica `CovisibilityIndex`) that are also candidates.
- Groups with a score ≥ 0.75 × the best are kept, using the best-scoring
  Frame of each group.

**4.3 `features::SearchByBoW`** (algs). It matches the current frame's
features against one Frame's features that observe a live point, comparing
only features under shared `FeatureVector` nodes.
- best distance ≤ 50;
- best < ratio × second best (0.75 for relocalization, 0.7 for reference
  tracking);
- rotation histogram: 12 bins, top 3;
- one-to-one.

It returns current feature → `MapPointId`. A candidate with fewer than 15
matches is dropped.

**4.4 `geometry::SolvePnPRansac`** (algs).
- **Minimal solver:** Lambda Twist P3P (Persson & Nordberg 2018) on unit
  bearing rays, which come from `Camera::Unproject`. A fourth point selects
  among the up-to-4 solutions.
- **RANSAC:** up to 300 iterations, confidence 0.99, at least 10 inliers.
- **Inliers:** raw-pixel χ² < 5.991 through `Camera::Project`, with
  σ = sigma_px·1.2^level. Points with depth ≤ 0 are outliers.
- **Output:** pose `T_cw`, inlier mask and success. Degenerate input (fewer
  than 4 points, collinear points, or no valid solution) fails cleanly,
  never with NaN.

**4.5 Verification.**
1. RANSAC runs on the candidates, with up to 5 iterations per candidate per
   round, until one yields a pose with ≥ 10 inliers.
2. `OptimizePose` runs on the inliers. Fewer than 10 survivors drops the
   candidate.
3. If fewer than 50 survive:
   - project the candidate's other points (a point source restricted to them,
     window factor 10, descriptor distance ≤ 100), then optimize again;
   - if the count is between 30 and 50, project once more with window factor
     3 and distance ≤ 64, then optimize.
4. Success means ≥ 50 inliers. The local-map step then runs, and Tracking
   resumes with stage `Relocalization`.

## 5. Testing

**`voyah_algs`:**
- **`FrameDatabase`:** add, remove, the common threshold and scores; a
  removed Frame is never returned.
- **`SearchByBoW`:** shared-node only, the ratio test, one-to-one, live
  points only.
- **`SolvePnPRansac`:**
  - Lambda Twist recovers the true pose from 3 noise-free rays to 1e-9;
  - noise-free fisheye rays up to 80° recover the pose;
  - 50 % outliers with 1 px noise recover it within 1 cm / 0.5°;
  - degenerate input fails cleanly.

**`voyah_slam`:**
- reference-Frame tracking recovers a synthetic jump that defeats the
  motion model and zero-motion;
- a blackout (empty measurements) gives RecentlyLost and
  `TrackingLost(final = false)`, the Mapper keeps its map, and resuming in
  the mapped area relocalizes into the same `map_id`;
- the 5 s timeout gives `final = true`, and the map is cleared;
- a map with ≤ 10 Frames goes directly to Lost;
- no vocabulary means behaviour unchanged;
- the database stays consistent with the replica after Frame removals.

**Integration:**
- `SlamTest.{Mono,Stereo}RelocalizesAfterBlackout`: a 1 s blackout in the
  mapped area ends in the same map, with the pose error within a
  noise-derived bound.

**Smoke** (corridor1, Release, `--vocabulary`). The apps gain
`--drop_frames a:b`, which feeds empty measurements:

| Run | Must | Report |
|---|---|---|
| Mono 400, drop 250–270 | Relocalizes into the same map | recovery frame, ATE, re-initializations |
| Stereo 1000, drop 500–520 | Relocalizes into the same map; no other loss | same |
| Mono 400, stereo 1000, no drop | No regression from the current DESIGN table | Frames, ATE, relocalization count |

## 6. Docs

- **`DESIGN.md`:**
  - the state machine and the `TrackingLost` `final` flag;
  - BoW in `MapTransaction`;
  - `FrameDatabase`, relocalization and PnP;
  - decisions-log entries.
- **`TODO.md`:** relocalization into other maps (step 5); MLPnP as an
  alternative.
- **`ROADMAP.md`:** step 3 marked done.
