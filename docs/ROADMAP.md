# Roadmap

Agreed 2026-10-07. Each step gets its own spec → plan → subagent-driven
execution; nothing is merged without an explicit "merge and push".

Done: tracking (sub-project 1), camera model (raw-pixel residuals),
keyframes and map growth (sub-project 2, merged b93fcf4), restructure
(step 0, merged 7426697), map quality (step 1, merged 9da7081), own DBoW2
(step 2, merged 6fb863a), relocalization (step 3, merged 6afacc7), loop
detection and correction (step 4a+4b: implemented (opt-in, off by default),
merged 2026-10-10; corridor1 Musts missed → step 6), global BA (step 4c,
merged 9245f92), map merging (step 5,
merged 9326160).

## 0. Restructure (done 2026-10-07)

**Result:** four libraries under `libs/` with separate include roots, one
test binary per library plus `voyah_integration_tests`, `check` / `check_all`
targets and `VOYAHSLAM_OPTIMIZE_TESTS`. Measured in the Debug tree (option
ON, `-j8`): `check` 449 tests in 1.5 s (was 45.3 s without the option),
`check_all` 459 tests in 2.3 s (was 66.7 s; the serial Debug baseline was
381 s). Smoke runs unchanged. Spec:
`docs/superpowers/specs/2026-10-07-restructure-design.md`; see DESIGN §7
"Library layout" and the 2026-10-07 decisions-log entry.

The plan as agreed:

Mechanical, no behaviour change. Goal: architecture boundaries and a fast
test loop.

| Library | Contents | Depends on |
|---|---|---|
| `voyah_common` | defs, ids, geometry types, `Camera`/distortion models, memory pools, logger, feature/measurement data types | Eigen |
| `voyah_events` | event types, pub/sub (`Observable`, `Listener`, `WorkerThread`) | common |
| `voyah_algs` | geometry algorithms, estimators, extractors, matchers, triangulation, optimization (g2o PRIVATE), BoW | common |
| `voyah_slam` | Tracker, Mapper, LoopCloser, replicas, `Slam` | all of the above |

- Measurements live in common because events carry `TMeasurement<FE>`; the
  viewer (`voyahviz`) then depends only on common + events.
- Tracking stays in `voyah_slam` with the Mapper and LoopCloser (shared
  replica / map code).
- One test binary per library.
- ctest labels: `unit` (default, seconds) and `integration` (synthetic
  end-to-end, Tracker/Slam sequences; run before commit/merge).
- Tests built optimized (`-O2 -g`): Eigen/g2o in Debug are 10–50× slower.
  Today the full suite is 458 tests / 76 s at `-j8`, dominated by ~5 tests of
  33–63 s each.

## 1. Map quality (sub-project 3) (done 2026-10-07)

**Result:** every Mapper change is one `MapTransaction` (two per created
Frame); fusion (`SearchInNeighbors`), recent-point culling, observation
removal, Frame culling and a local BA in the Mapper thread that an urgent
`FrameTracked` interrupts. Apps write their trajectory (`--trajectory_out`)
and `tools/ate.py` reports ATE against the mocap. Smoke on corridor1
(Release, two runs each):

| Run | Before | After |
|---|---|---|
| Mono 400 | 2 maps, first lost after 129 tracked frames, 3 Frames created | **1 map** from initialization (frame 118) to the end, 281–282 tracked, 64–68 Frames created / 41–45 culled, ATE Sim3 0.032–0.033 m |
| Stereo 1000 | 1–3 maps (lost at ~360 in two of four runs) | 1 map, no loss, 999 tracked, 280–297 Frames created / 73–89 culled, ATE SE3 0.036–0.037 m |

Local BA about 105 ms (mono) / 135 ms (stereo) per Frame, interrupted once
per stereo run; a Mapper pass about 250 ms. Synthetic leave-the-view error
bounds tightened from 30 % of the path / 0.3 m to 0.249 m (4.2 %) / 0.037 m.
Spec: `docs/superpowers/specs/2026-10-07-map-quality-design.md`; see DESIGN
"Map Quality" and the 2026-10-07 decisions-log entries.

The step as agreed:

Fusion (`SearchInNeighbors`), recent-point culling, `ObservationsRemoved`,
Frame culling, interruptible local BA. Fixes the mono Frame-creation stall
(triangulated points have 2 observations < `min_obs` 3, so `ref_points`
stays low).

## 2. Own DBoW2 without OpenCV (done 2026-10-07)

**Result:** `bow::Vocabulary<FE>` in `voyah_algs` (no OpenCV): DBoW2 TF-IDF /
L1, the text loader, our checksummed binary format and
`tools/convert_vocabulary`. On ORB-SLAM3's `ORBvoc.txt` it reproduces the
original DBoW2 exactly on 3 corridor1 frames (words, feature filing,
bit-identical weights). Load: text 0.53 s (DBoW2 2.7 s), binary 0.13 s. ORB
offsets are rounded as in ORB-SLAM3. The apps take `--vocabulary`; the Mapper
keeps a BowVector / FeatureVector per Frame (about 7 ms) and triangulates
with `SearchByBoW`. Smoke on corridor1 (Release, two runs each):

| Run | Without vocabulary | With vocabulary |
|---|---|---|
| Mono 400 | triangulation 178–182 ms per Frame, Mapper pass 292–294 ms; 1 map, 277–278 tracked, ATE Sim3 0.034 m | triangulation **6.5–6.8 ms (26–28×)**, Mapper pass 140–143 ms; 1 map, 278 tracked, ATE Sim3 0.031–0.033 m |
| Stereo 1000 | triangulation 146–149 ms, Mapper pass 286–287 ms; 1 map, no loss, ATE SE3 0.037–0.038 m | triangulation **3.7–3.8 ms (39×)**, Mapper pass 137–138 ms; 1 map, no loss, **ATE SE3 0.056 / 0.189 m** |

The 3× target is met. Stereo accuracy regressed (timing-dependent drift when
the Mapper is fast, not fewer triangulated points; TODO), and an extra stereo run lost tracking at the known
fast-rotation frame (325). Spec: `docs/superpowers/specs/2026-10-07-bow-design.md`;
see DESIGN "Bag of Words" and the 2026-10-07 decisions-log entries. Out of
scope and in TODO: vocabulary training, the `ORBvoc.txt` licence, the Frame
database (step 3).

The step as agreed:

Vocabulary tree, binary k-majority, `BowVector` / `FeatureVector`, vocabulary
loader; copy the few OpenCV pieces needed (keep notices: DBoW2 BSD, OpenCV
Apache-2.0). `FeatureVector` also enables `SearchByBoW`, replacing the
brute-force triangulation matching (~25 ms per mono neighbour, ~0.75 s per
Frame at 30 neighbours).

## 3. Relocalization (sub-project 4) (done 2026-10-08)

**Result:** a short tracking loss is recovered into the **same map**. With a
vocabulary the Tracker falls back to reference-Frame tracking
(`SearchByBoW`), then enters `RecentlyLost` (`TrackingLost(final = false)`,
the Mapper keeps its map) and relocalizes every measurement for up to 5 s:
`bow::FrameDatabase` query, covisibility-grouped candidates, `SearchByBoW`,
Lambda Twist P3P RANSAC on rays, `OptimizePose` over all matches, ORB-exact
projection passes, 50 inliers. Only the timeout (`final = true`) clears the
map. A motion-model inlier gate (30) fixed the stereo-with-vocabulary
regression of step 2 (0.056 / 0.189 m → 0.036 m). The apps take
`--drop_frames a:b`. Smoke on corridor1 (Release, `--vocabulary`, two runs
each):

| Run | Result |
|---|---|
| Mono 400, drop 250–270 | relocalized into map 1 at frame 271 in both runs (the first frame after the drop); 1 map; ATE Sim3 0.0325 / 0.0336 m |
| Stereo 1000, drop 500–520 | relocalized into map 1 at frames 521 / 524; no other loss; ATE SE3 0.0351 / 0.0349 m |
| Mono 400, no drop | 1 map, 277–278 tracked, ATE Sim3 0.0324 / 0.0323 m, no relocalization |
| Stereo 1000, no drop | 1 map, 999 tracked, no loss, ATE SE3 0.0365 / 0.0363 m (step 2: 0.056 / 0.189 m) |

A relocalization attempt costs about 10 ms.

Spec: `docs/superpowers/specs/2026-10-07-relocalization-design.md`; see
DESIGN "Relocalization and Reference-Frame Tracking" and the 2026-10-08
decisions-log entries. Out of scope and in TODO: relocalization into other
maps (step 5), MLPnP, ORB-SLAM3's post-relocalization gate.

The step as agreed:

Plus reference-Frame tracking. Fixes the stereo loss in fast rotation
(120–170°/s; the zero-motion window cannot cover 6–8° per frame).

## 4. Loop closing

### 4a+4b. Detection and correction: implemented (opt-in, off by default), merged 2026-10-10; corridor1 Musts missed → step 6

**Result:** the LoopCloser detects on its own replica, ORB-SLAM3
`NewDetectCommonRegions` with its source thresholds (BoW candidates, Sim3
RANSAC, `OptimizeSim3`, projections, three confirmations) and publishes
`LoopDetected` with the Sim3 and the matched points; the Mapper corrects
(`CorrectLoop`: window, points, `SearchAndFuse`, essential graph, loop edge)
as one `MapTransaction` at a new map epoch; the Tracker's last frame follows
its reference Frame. The map has a spanning tree and loop edges. The apps
take `--loop_closing`: loop closing is opt-in and off by default (user
decision 2026-10-10), because on corridor1 it makes the ATE worse (below).
The LoopCloser adds a normal-consistency gate.
Synthetic loops (integration): the end error with
loop closing is 0.04–0.15 of the error without (mono over three seeds,
stereo), with no tracking loss across the correction. Smoke on corridor1
(Release, `--vocabulary`, full sequence, two runs each):

With the normal-consistency gate (a user decision beyond ORB-SLAM3), fix
round 1:

| Run | Result |
|---|---|
| Mono, loop closing | one map in both runs; opposite-facing false loops rejected (0.84–1.00); same-heading false loops 7–22 m apart corrected (run 1: then true returns rejected too); ATE Sim3 0.679 / 0.669 m |
| Mono, no loop closing | one map, ATE Sim3 0.341 / 0.500 m |
| Stereo, loop closing | one map, no extra loss; opposite-facing loops rejected (0.93–1.00); a same-heading false loop 22–30 m apart corrected per run, then true returns rejected; ATE SE3 1.295 / 18.16 m |
| Stereo, no loop closing | one map, ATE SE3 0.098 / 0.138 m |

Before the gate (e529d07), the opposite-facing loop 2262→1587 cost map 1 in
both stereo runs (4 maps) and mono run 2 (2 maps). The smoke Musts are
**missed**; nothing was tuned. Same-heading look-alike rejection is the first
item of step 6.

Spec: `docs/superpowers/specs/2026-10-09-loop-closing-design.md`; see
DESIGN "Loop Closing" and the 2026-10-09 / 2026-10-10 decisions-log entries. Out of scope
and in TODO: map merging (step 5), scale without IMU; global BA is 4c below.

### 4c. Global BA (merged 9245f92, 2026-10-10)

**Result:** ORB-SLAM3's global BA (10 iterations, no robust kernel) runs
after each loop correction in its own component, `GlobalBundleAdjuster`, on
its own replica fed from the start (user decision: no late joining; pluggable
later through gRPC). A newer correction or a new map interrupts a run; the
Mapper merges the plain-data `GlobalBAFinished` as one transaction at a new
epoch, propagating through the spanning tree to Frames and points created
during the run. It exists only with loop closing (opt-in) and is on by
default with it; `--no_global_ba` turns it off.

Synthetic loops (integration, seeds 1–3): the map points' RMS error against
truth is 0.41–0.48× the essential graph's in stereo and 0.87–0.92× in mono
(mono bar revised after measurement), with no tracking loss.

Smoke on corridor1 (full, Release, `--vocabulary --loop_closing`, two runs
each with and without `--no_global_ba`): no crash and no tracking loss
caused by a merge (Musts met). Runs take 3–28 s in mono and up to 85 s in
stereo, merges 4–32 ms, results 0.6–3.3 MB; peak RSS 4.23 GB in stereo
against 2.3 GB without. Three of the four runs with global BA lost map 1
after a false-loop correction, against none without (two runs per mode:
not separable from loop variance); the ATE is not comparable this round and
false loops still dominate (step 6). Open items in TODO "Global BA": an
iteration or size cap, memory, late joining, serialization.

Spec: `docs/superpowers/specs/2026-10-10-global-ba-design.md`; see DESIGN
"Global BA".

## 5. Map merging (multi-map Atlas) (merged 9326160, 2026-10-11)

**Result:** ORB-SLAM3's Atlas (scope A). A lost map with at least 10 Frames
is kept inactive; the LoopCloser detects revisits of inactive maps (merge
candidates beside loop candidates, the same gates except the normal gate,
mono free-scale); the Mapper merges the active map into the inactive one
(`MergeLocal`, visual: windows, absorb by the id rule, window correction,
re-rooted tree, seam fusion, weld, essential graph of the rest, merge edge)
as one transaction; the target keeps its id and world. Every replica (Tracker,
LoopCloser, GlobalBA, viewer) mirrors the whole Atlas under the replica rule
(replacing "newest id wins"), with each Frame's measurement shared read-only;
the Tracker follows a merge without a reset. Opt-in with loop closing: without it (or
without a vocabulary) lost maps are discarded, as before the Atlas.

Synthetic loops with a blackout (integration, seeds 1–3, mono and stereo):
one map at the end holding both parts, no tracking loss across the merge,
whole-run error 0.63–1.31× (mono) and 0.90–1.40× (stereo) the run without
the blackout (bar 1.5×; thin margins, TODO). Shared measurements cut the
stereo corridor1 peak RSS from 1.7 GB to about 1.2 GB. Corridor1 merge smoke:
DESIGN "Loop Closing" (map merging smoke). Open items in TODO "Map merging
follow-ups": relocalization into inactive maps, the viewer's inactive maps,
separate merge edges.

Spec: `docs/superpowers/specs/2026-10-10-map-merging-design.md`.

## API hardening for library consumers (ownership audit, before step 6)

Added 2026-10-12 (user decision): the Java/JNI binding (`../voyah_jni`) needs
every lifetime rule enforced by the types, not by comments, because Java
objects die in any order. Audit:
`../voyah_jni/docs/voyah_slam_ownership_audit.md`; items in TODO "Ownership
audit". Done: #1 shared distortion models (0.2.0), #4 pools grow instead of
aborting (0.3.0), #3 with #2 (0.4.0, 2026-10-13: events are
`shared_ptr<const T>` with no pools, `AddObserver` returns a `Subscription`,
and every destruction order of producer, subscription, observer and listener is
safe, so `Slam`'s users no longer need a declaration or teardown order beyond
holding their `Subscription`s before the `Slam`), #5 (0.5.0, 2026-10-14:
the thread-affine accessors of the Mapper, Tracker, LoopCloser and GlobalBA are
private behind test access structs, and `mapping::MapFollower` is the supported
map read path), #6 (0.6.0, 2026-10-14: the Created to Stopped lifecycle is an atomic
state machine with `UsageError` on invalid transitions, `Start()` / `Stop()` are
valid from any thread, and `Stop()` then `Start()` is decided as not supported:
Stopped is terminal, a new session is a new `Slam`), #7 (0.7.0, 2026-10-14: `OrbFeatureExtractor`
has no mutable state, the pyramid lives in a caller-owned `Workspace`, so one extractor is safe to
share between threads), #8 (0.8.0, 2026-10-14: every logger writes through one shared
`dist_sink_mt`, so `Logger::Initialize` re-targets components created before it, the level is atomic and
logger creation is serialized), #9 (0.9.0, 2026-10-15: `images::Image` is fixed in place, with an
empty default, view-correct `Convert` / `operator*=` / `operator*`, an overflow-checked size constructor
and `size_t` view offsets). The API hardening list is complete. Each item is merged separately with a minor
version bump and new Conan packages.
The Phase 1 verification gaps and the review findings CR-01, CR-02, WR-01, WR-06 and IN-07 are fixed in
0.9.2 (2026-10-21, patch release on branch `fix/phase1-gaps`; the other review findings are open items in TODO).

## 6. TUM-VI tuning

At the very end. First item: loop-closing look-alike rejection. Same-heading
corridor look-alikes pass every gate, and correcting them makes corridor1
worse, so loop closing stays off by default until they are rejected. The
candidates are a drift-bound gate, verify-before-publish, or both (TODO
"TUM-VI tuning").
