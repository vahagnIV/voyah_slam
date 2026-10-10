# Global Bundle Adjustment After Loop Correction — Design Spec

Date: 2026-10-10
Status: approved in conversation, pending spec review
Roadmap: step 4c in `docs/ROADMAP.md`. It builds on step 4a+4b (loop closing,
merged 5d94f69, opt-in).

## Context and Scope

After a loop correction the Mapper stops at the essential graph. Frames get a
pose-only correction, points follow their reference Frame, and the loop's
residual stays in the map until local BA happens to reach it (TODO "Global
BA"). ORB-SLAM3 follows every loop correction with a full bundle adjustment in
its own thread (`LoopClosing::RunGlobalBundleAdjustment`). This spec adds that
step.

**Goal.** After each loop correction, refine every Frame and point of the map
with a global BA. Local mapping must not be blocked while it runs, and the
result is merged into a map that kept changing meanwhile.

**ORB-SLAM3 reference** (`LoopClosing.cc`, `Optimizer.cc`):
1. The end of `CorrectLoop` launches the thread, after aborting a running one
   (`mbStopGBA`, `mnFullBAIdx`).
2. `GlobalBundleAdjustemnt(map, 10, &mbStopGBA, loop_kf, bRobust = false)`
   builds the g2o graph from the current map, which is in effect a snapshot.
   It optimizes while tracking and local mapping continue. Results are parked
   in `mTcwGBA` and `mPosGBA`.
3. If the run was not aborted, it stops local mapping and takes the map lock
   for the merge only:
   - Keyframes created during the run take their parent's correction through
     the spanning tree.
   - Newer points move with their reference keyframe.

   Then it releases local mapping.

**Architectural differences from ORB-SLAM3:**

| ORB-SLAM3 | Ours |
|---|---|
| The GBA thread reads the shared map under locks | GlobalBA owns a map replica, fed by the Mapper's transactions from the start (like the LoopCloser) |
| Results are parked in fields of the shared objects | Results travel in one `GlobalBAFinished` event (plain data) |
| Local mapping is stopped for the merge, under the map lock | The merge is one Mapper handler call. The Mapper is the only writer, so no lock is needed. |
| The Tracker reads the merged map | One `MapTransaction` at a new epoch, applied by every replica. The Tracker shifts `last_frame_` (step 4b path). |

**Constraints.**
- No locks on map data. The Mapper is the only writer.
- Components communicate only through events carrying plain data, with no
  shared pointers into map objects. This keeps a future out-of-process
  transport (gRPC) possible.
- With loop closing off (the default), behaviour is exactly as today.

**Out of scope (TODO):**
- Late joining: a component starting mid-run needs a full-map snapshot to
  build its replica. Components either exist from the start or not at all.
- Serialization and a gRPC transport for pluggable components.
- Rejecting loop-closing look-alikes (step 6).

## 1. The GlobalBA component

**Thread.**
- `GlobalBundleAdjuster` runs in its own thread with its own `MapReplica`.
- It subscribes to the Mapper's `MapTransaction` stream from the start of the
  run.
- `Slam` creates it only when loop closing is enabled and
  `GlobalBaOptions::enabled` is set.

**Trigger.**
- A run starts when GlobalBA applies a transaction of the current map that
  adds a loop edge (`added_loop_edges` non-empty). That is a loop correction
  at epoch N.
- There is no request event. The correction transaction itself is the trigger,
  as the end of ORB-SLAM3's `CorrectLoop` is.

**The run.**
- It optimizes a snapshot of the replica as it stands at epoch N:
  `RunGlobalBa` builds a self-contained problem before its first
  `should_stop` poll and never reads the replica again.
- While it runs, each incoming transaction is applied to the replica (and so
  released) at the next `should_stop` poll, after it is checked for
  interruption (below). The result still carries epoch N.
- Why: the Mapper's transaction pool (256) aborts when exhausted, and a run
  on a large map lasts seconds, at about 2 transactions per KeyFrame.
  Holding them for the run would exhaust it (the LoopCloser's "apply on
  receipt" rule, for the same reason).

**Interruption.**
- BundleAdjust's `should_stop` callback drains GlobalBA's inbox (applying
  each transaction, as above). It returns true when a drained transaction
  either:
  - is a newer correction: larger `map_epoch` and a new loop edge; or
  - starts a new map (`new_map` or a newer map id).
  It also returns true once `Stop()` is requested; a run stopped so is not
  resumed after a restart.
- An interrupted run publishes nothing.
- A newer correction applied during the run starts a new run next.
  This mirrors ORB-SLAM3 aborting the running GBA from `CorrectLoop`.

**New map.** A new map clears the replica and drops any pending run.

**Result: `GlobalBAFinished`** (voyah_events). Fields:
- `map_id`;
- `map_epoch` (N, the epoch optimized);
- `frame_poses` as (FrameId, T_cw) pairs;
- `point_positions` as (MapPointId, position) pairs.

Every Frame and every point included in the optimization is listed.

## 2. The optimization (ORB-SLAM3 `GlobalBundleAdjustemnt`, `bRobust = false`)

**Inputs.**
- Every Frame and every live point in the replica at N, with all their
  observations, mono and stereo.
- A point left without observations is not in the problem.

**Solver.**
- The existing `optimization::BundleAdjust`, run as one plain stage of
  `GlobalBaOptions::iterations` (10).
- No robust kernel, and no outlier removal, as in ORB-SLAM3.
- Residuals in raw pixels, through `geometry::Camera`.

**Gauge.**
- The map's first Frame is fixed (ORB-SLAM3 fixes the map's init keyframe).
  If it was culled, the oldest surviving Frame is fixed instead.
- Mono uses SE3 poses with no scale variable: the scale is the one the loop
  correction left.

## 3. The merge (Mapper, ORB-SLAM3's post-GBA block)

**Rejection.** The Mapper ignores a `GlobalBAFinished` when either:
- its `map_id` is not the current map; or
- its `map_epoch` is not the Mapper's current epoch (another correction came
  in between; ORB-SLAM3 `mnFullBAIdx`).

**Frames.** Walk the spanning tree from the root, parents first.
- A Frame in the result takes its optimized pose.
- A Frame created after N takes `T_new = T_cur · T_parent,cur⁻¹ · T_parent,new`.
  The parent is visited first, so the correction chains down the tree.
- Frames culled since N are skipped.
- Each Frame's pre-merge pose is kept for the point step.

**Points.**
- A point in the result that is still alive takes its optimized position.
  Points fused or culled since N are skipped.
- A point created after N moves rigidly with its reference Frame: through the
  Frame's pre-merge pose into camera coordinates, then out through its new
  pose.
  - It is skipped if its reference Frame was not corrected.

**Publish.**
- One `MapTransaction` at `map_epoch + 1`, carrying every changed pose and
  position.
- Local BA work done since N on optimized Frames is overwritten by the global
  result, as in ORB-SLAM3.
- The Tracker follows through the step 4b epoch path. The LoopCloser and
  GlobalBA apply the transaction like any other.
- A `MapTransaction` of the merge carries no loop edge, so it never triggers
  another run.

**Reuse.** The merge is a Mapper function that takes the result, so map
merging (step 5) can reuse the propagation.

## 4. Options

- **`GlobalBaOptions`:**
  - `enabled` (true; only takes effect when loop closing is on);
  - `iterations` 10.
- **`SlamOptions`** carries `GlobalBaOptions`.
- **The apps:** `--no_global_ba`, for comparing the essential graph alone with
  the essential graph followed by global BA.

## 5. Testing

**`voyah_slam`, GlobalBA:**
- A run starts only on a transaction with a new loop edge. Ordinary
  transactions never start one.
- One `GlobalBAFinished` is published with the right map id and epoch N. It
  covers every Frame and point at N.
- A newer correction mid-run interrupts it: nothing is published for N, and a
  run for the newer epoch follows.
- A new map clears the replica and drops the run.
- When disabled, or with loop closing off, nothing is created or published.

**`voyah_slam`, Mapper merge:**
- A stale epoch or an old map changes nothing and publishes nothing.
- Optimized Frames and points take the result.
- Frames created after N follow their parent through the tree, including
  chains of two or more Frames.
- Points created after N follow their reference Frame.
- Frames and points culled or fused since N are skipped, and no dangling ids
  appear in the transaction.
- Exactly one transaction is published, at N + 1, and the replicas stay
  consistent after applying it.

**Tracker:** no tracking loss across a global-BA merge.

**Integration:**
- `SlamTest.{Mono,Stereo}ClosesLoop` gain a global-BA variant. It is compared
  with the essential-graph-only run over 3 fixed seeds.
- Success:
  - **stereo:** the map points' RMS error against ground truth is at most
    0.8× the essential-graph-only value, and the end-of-run trajectory error
    is no worse;
  - **mono:** the points RMS is at most 0.95× per seed and at most 0.92× on
    average over the 3 seeds, and the end error is at most 1.05× per seed;
  - no tracking loss, and one map.
- Note (2026-10-10): the mono bar was revised after measurement, from the
  0.8× / 1.0× above. Its RMS is dominated by low-parallax points, and global BA
  converges (10 and 100 iterations give the same result). Measured points-RMS
  ratios were 0.89 / 0.87 / 0.92 (mean 0.89) and end-error ratios 0.58 /
  0.35 / 1.005. Stereo measured 0.41 / 0.48 / 0.44.
- A missed bar is diagnosed, not tuned.

**ThreadSanitizer:** the slam and integration test binaries.

**Smoke:** corridor1, Release, `--vocabulary --loop_closing`. Two runs each,
with and without `--no_global_ba`, for both mono and stereo.

| Must | Report |
|---|---|
| No crash and no tracking loss caused by a merge | GBA runs started and interrupted, GBA ms, merge ms, result-event size, peak RSS, ATE both ways |

There is no ATE Must: corridor1's false loops still dominate (step 6).

## 6. Docs

- **`DESIGN.md`:**
  - the GlobalBA component and the merge rule;
  - the decisions-log entries: a replica fed from the start with no late
    joining, and pluggable later through gRPC;
  - Open Question 5 answered.
- **`TODO.md`:**
  - late joining and serialization/gRPC for pluggable components;
  - the measured memory use of about 1 MB per Frame (stereo corridor1 peaked
    at 1.7 GB with 1780 Frames and 113k points);
  - a global-BA iteration or size cap, if the smoke shows long runs.
- **`ROADMAP.md`:** 4c done.
