# Roadmap

Agreed 2026-10-07. Each step gets its own spec → plan → subagent-driven
execution; nothing is merged without an explicit "merge and push".

Done: tracking (sub-project 1), camera model (raw-pixel residuals),
keyframes and map growth (sub-project 2, merged b93fcf4), restructure
(step 0, merged 7426697), map quality (step 1, merged 9da7081), own DBoW2
(step 2, merged 6fb863a).

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

## 3. Relocalization (sub-project 4)

Plus reference-Frame tracking. Fixes the stereo loss in fast rotation
(120–170°/s; the zero-motion window cannot cover 6–8° per frame).

## 4. Loop closing

Detection in the LoopCloser thread (BoW + Sim3/SE3 verification); correction
in the Mapper (fusion, essential graph, global BA).
Global BA runs in its own thread (local BA stays in the Mapper; TODO).

## 5. Map merging (multi-map Atlas)

Needs its own design discussion: it conflicts with today's rule that a newer
map clears and adopts (old maps must be kept).

## 6. TUM-VI tuning

At the very end.
