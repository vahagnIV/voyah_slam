# Roadmap

Agreed 2026-10-07. Each step gets its own spec → plan → subagent-driven
execution; nothing is merged without an explicit "merge and push".

Done: tracking (sub-project 1), camera model (raw-pixel residuals),
keyframes and map growth (sub-project 2, merged b93fcf4), restructure
(step 0, merged 7426697).

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

## 1. Map quality (sub-project 3)

Fusion (`SearchInNeighbors`), recent-point culling, `ObservationsRemoved`,
Frame culling, interruptible local BA. Fixes the mono Frame-creation stall
(triangulated points have 2 observations < `min_obs` 3, so `ref_points`
stays low).

## 2. Own DBoW2 without OpenCV

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

## 5. Map merging (multi-map Atlas)

Needs its own design discussion: it conflicts with today's rule that a newer
map clears and adopts (old maps must be kept).

## 6. TUM-VI tuning

At the very end.
