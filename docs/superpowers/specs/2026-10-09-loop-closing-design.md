# Loop Closing: Detection and Correction — Design Spec

Date: 2026-10-09
Status: approved in conversation, pending spec review
Roadmap: step 4, parts a and b, in `docs/ROADMAP.md`. Global BA is step 4c;
map merging is step 5.

## Context and Scope

The `LoopCloser` thread is a skeleton: its inbox only acknowledges events.
`LoopDetected` carries only two Frame ids, and the Mapper ignores it. As a
result, drift accumulates without bound. corridor1 starts and ends in the
same mocap room, so the end-of-run ATE measures exactly what loop closing
fixes.

**Goal.** When the camera returns to a mapped place, detect it and correct
the accumulated drift: pose, and scale for mono. We follow ORB-SLAM3's
visual-only `LoopClosing` (`NewDetectCommonRegions`, `CorrectLoop`,
`OptimizeEssentialGraph`) for the algorithm, its thresholds and its order.
The plan verifies each threshold against the ORB-SLAM3 source.

**Architectural differences from ORB-SLAM3:**

| ORB-SLAM3 | Ours |
|---|---|
| LoopClosing detects and corrects, pausing LocalMapping | The LoopCloser only detects, on its own map replica. The Mapper corrects, and does no local mapping meanwhile. |
| Shared map objects corrected in place, under locks | One `MapTransaction` applied by every thread |
| The Tracker reads the corrected map | The Tracker applies the transaction and shifts its `last_frame_` |
| Global BA in a thread right after correction | Step 4c |
| Merging into Atlas maps | Step 5. Detection is reusable, `LoopDetected` carries both map ids, and the correction steps are reusable. |

**Constraints.**
- No locks on map data.
- The Mapper is the only writer.
- Residuals are in raw pixels.
- Without a vocabulary there is no loop closing, so behaviour is as today.

## 1. Map graph: spanning tree and loop edges

- **Spanning tree.** Each Frame has a parent: the existing Frame sharing the
  most points with it at creation (ORB-SLAM3 `UpdateConnections`). The first
  Frame of a map has none.
  - When a Frame is culled, its children are re-parented as in ORB-SLAM3's
    `KeyFrame::SetBadFlag`. Each child goes to the candidate parent (the
    culled Frame's parent, or an already re-parented sibling) with which it
    shares the most points.
- **Loop edges.** Each Frame keeps a set of loop partners. Frames with loop
  edges are never culled.
- **`MapTransaction` gains three parts:**
  - `parent` on each `NewFrame`;
  - `parent_changes` as (frame, new parent) pairs;
  - `added_loop_edges` as (frame, frame) pairs.

  Every replica stores them.
- **`MapTransaction` gains `map_epoch`** (`uint32_t`). It increments on
  every loop correction.
- **`FrameTracked` gains `map_epoch`:** the epoch of the Tracker's replica.

## 2. Detection and verification (LoopCloser, ORB-SLAM3 `NewDetectCommonRegions`)

**Replica.**
- The LoopCloser keeps its own `MapReplica`, with its `FrameDatabase` and
  covisibility, fed by the Mapper's transactions.
- It receives the vocabulary through `Slam`.
- Every Frame in `added_frames` is a query. Detection is skipped while the
  map has fewer than `min_map_frames` (12) Frames.

**Candidates** (`DetectNBestCandidates`).
- Database hits come from `FrameDatabase::Query`, excluding the query
  Frame's covisible Frames.
- Keep `common ≥ 0.8·max`, and compute group scores over the top 10
  covisible Frames.
- Take the best `num_candidates` (3).

**Per candidate** (`DetectCommonRegionsFromBoW`):
1. **Window:** the candidate plus its top `window` (5) covisible Frames.
2. **Matching:** `SearchByBoW` from the query against each window Frame,
   keeping map points only. The result must reach `min_bow_matches` (20).
3. **RANSAC:** `geometry::SolveSim3Ransac` on the 3-D point pairs.
   - The query side's points come from its observations; the window side's
     points are the matched map points.
   - The scale is fixed to 1 for stereo.
   - At least `min_sim3_inliers` (20) inliers.
4. **First projection:** `features::SearchBySim3Projection` of the window's
   points, then `optimization::OptimizeSim3`. At least `min_optimized` (15)
   inliers.
5. **Second projection:** with the optimized Sim3, at least `min_projection`
   (100) matches makes it a candidate loop.

**Consistency** (`mnLoopNumCoincidences`).
- A candidate is confirmed `required_coincidences` (3) times before it
  becomes a loop.
- Confirmations come from the query's covisible Frames (ORB-SLAM3
  `DetectAndReffineSim3FromLastKF`, at least 50 projection matches), or from
  later queries within `max_unconfirmed_queries`.
- A failed confirmation resets the count.

**`LoopDetected`** carries:
- `query_map_id`, `match_map_id` (equal for now), `query_frame` and
  `match_frame`;
- the Sim3 `S_query_to_match`, as scale s, rotation R and translation t,
  with s = 1 for stereo;
- the matched pairs `(query_point, match_point)`.

**New in algs:**
- **`geometry::SolveSim3Ransac`:**
  - Horn / Umeyama, with or without scale;
  - a seeded RANSAC over 3-point samples;
  - inliers by raw-pixel χ² 9.21, as ORB-SLAM3's `Sim3Solver` uses in both
    images;
  - clean failure on degenerate input.
- **`optimization::OptimizeSim3`:**
  - g2o, ORB-SLAM3 `OptimizeSim3`: projection residuals in both Frames in raw
    pixels, robust, 5 + 5 iterations;
  - outliers removed between the stages;
  - the scale is fixed for stereo.
- **`features::SearchBySim3Projection`:** ORB-SLAM3
  `SearchByProjection(KeyFrame*, Sim3, …)`, using ORB-exact radii like
  `SearchFromFrame`.

## 3. Correction (Mapper, ORB-SLAM3 `CorrectLoop`)

**Rejection.** On `LoopDetected`, reject it if the map differs, the query or
match Frame is culled, or a correction is already running.

**Steps:**
1. **Window:** the query Frame plus its covisible Frames.
2. **Window poses:**
   - the corrected Sim3 of the query is `S_query_to_match` composed with the
     match Frame's pose;
   - each window Frame keeps its relative Sim3 to the query.
3. **Points:**
   - each point observed by the window moves by its reference Frame's
     correction, `X' = S_new⁻¹·S_old·X`;
   - for mono, the scale folds into each Frame's translation.
4. **Loop fusion:**
   - the `LoopDetected` pairs replace the query side's points with the match
     side's;
   - `SearchAndFuse` projects the loop window's points into the corrected
     window Frames (Sim3, radius 4) and merges duplicates, through the
     existing fusion and "points replaced" machinery.
5. **Connections:** covisibility is updated, which creates the cross-loop
   links.
6. **Essential graph** (`optimization::OptimizeEssentialGraph`, g2o):
   - **vertices:** Sim3 per Frame, with the scale fixed for stereo; the match
     Frame is fixed;
   - **edges:** the spanning tree, all loop edges, and covisibility edges
     with ≥ `min_essential_weight` (100) shared points;
   - **initial values:** the corrected window poses, and the old poses
     elsewhere;
   - **solver:** 20 LM iterations.

   Frames take the optimized poses, and points follow their reference Frame.
7. **Loop edge:** add query ↔ match.
8. **Publish one `MapTransaction`:**
   - all updated poses and points;
   - fused, replaced and removed points;
   - observations;
   - the loop edge;
   - `map_epoch + 1`.

**Stale epoch.** The Mapper never creates a Frame from a `FrameTracked` with
an older `map_epoch`. Its visible/found counters are still updated.

**Tracker.**
- After applying a transaction with a new epoch, `last_frame_`'s pose
  follows its reference Frame's world change:
  `T_last ← T_last · T_ref_old⁻¹ · T_ref_new`, with the scale for mono.
- The velocity is unchanged.
- `FrameTracked` carries the replica's epoch.

**Reuse for step 5.** Steps 3–6 are Mapper functions that take a window and
a Sim3, so map merging can call them.

## 4. Options

- **`LoopCloserOptions`:**
  - `min_map_frames` 12;
  - `num_candidates` 3, `window` 5;
  - `min_bow_matches` 20, `min_sim3_inliers` 20, `min_optimized` 15;
  - `min_projection` 100, `confirmation_projection` 50;
  - `required_coincidences` 3, `max_unconfirmed_queries` 3;
  - RANSAC 300 iterations, confidence 0.99, χ² 9.21.
- **`MapperOptions::loop`:**
  - `min_essential_weight` 100;
  - `essential_iterations` 20;
  - `fuse_radius` 4.
- **The apps:** `--no_loop_closing`.

## 5. Testing

**`voyah_algs`:**
- **`SolveSim3Ransac`:** a noise-free known Sim3 recovered to 1e-9, with
  scale or with it fixed; 50 % outliers, checked statistically over fixed
  seed ranges; degenerate input fails cleanly.
- **`OptimizeSim3`:** a finite-difference Jacobian check; a perturbed Sim3 is
  recovered; stereo keeps scale fixed.
- **`SearchBySim3Projection`:** the radius in both directions.
- **`OptimizeEssentialGraph`:** a synthetic ring with drift and one loop edge
  is corrected within a bound; the fixed vertex stays put; stereo keeps
  scale.

**`voyah_slam`:**
- **Spanning tree:** a parent on creation; re-parenting on cull; parents and
  loop edges reach the replicas; loop-edge Frames are never culled.
- **LoopCloser:**
  - a synthetic return-to-start loop is detected only after 3 confirmations;
  - a covisible revisit is not a loop;
  - an unmapped look-alike is rejected;
  - a stale map is ignored.
- **Mapper correction:** the window, points and graph are corrected; the loop
  points are fused; one transaction is published; the replica stays
  consistent; a stale-epoch `FrameTracked` creates no Frame.
- **Tracker:** `last_frame_` follows its reference after a correction, and
  the next frame is tracked.

**Integration.** `SlamTest.{Mono,Stereo}ClosesLoop`: a synthetic loop with
drift (mono drift includes scale). The end-of-run error with loop closing is
≤ 1/3 of the error without, measured over 3 runs.

**ThreadSanitizer** covers the Tracker, Mapper and LoopCloser.

**Smoke** (corridor1, Release, `--vocabulary`, full sequence, 2 runs each,
with and without `--no_loop_closing`):

| Run | Must | Report |
|---|---|---|
| Mono full | Loop detected on return to the room and corrected; one map; ATE (Sim3) below the no-loop run | loop frame, correction ms, ATE both ways, Frames, points |
| Stereo full | Same; no tracking loss across the correction | same, ATE (SE3) |

## 6. Docs

- **`DESIGN.md`:**
  - the LoopCloser replica and detection;
  - the spanning tree and loop edges;
  - correction, the epoch and the Tracker shift;
  - the essential graph;
  - decisions-log entries.
- **`TODO.md`:** global BA (4c); map merging (step 5); scale limits without
  IMU.
- **`ROADMAP.md`:** step 4a+4b marked done.
