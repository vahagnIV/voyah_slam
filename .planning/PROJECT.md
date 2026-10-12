# voyahslam

## What This Is

voyahslam is a lock-free, event-driven, ORB-SLAM3-inspired visual SLAM library (monocular and stereo) in C++20 on Eigen, packaged with Conan 2 (currently version 0.3.0). The Tracker, Mapper, LoopCloser, GlobalBA and viewer threads never share map data: each owns a replica of the map and they talk only through pooled publish/subscribe events, with the Mapper as the single writer. It targets Linux x86_64 and Android arm64-v8a (consumed from a Kotlin + JNI app through the Conan package); the apps in `examples/` are optional reference consumers.

This is a brownfield project. The library is implemented and tested (746 tests in `check`, 773 in `check_all`); `.planning/` was initialized from the repo's own docs (41 documents ingested, see `.planning/intel/SYNTHESIS.md`). Only the pending work becomes roadmap phases.

## Core Value

ORB-SLAM3-class accuracy on real sequences (TUM-VI corridor, mono and stereo) delivered through a lock-free, replica-per-thread, event-only architecture. If a change trades the architecture or the accuracy away, it is the wrong change.

## Success Metric

Developer-facing, applied as a regression guard to every phase and as the exit bar of the last one:

- ATE on TUM-VI corridor sequences (mono + stereo) is no worse than the baselines recorded in the docs.
- The full test suite is green (`check_all`, about 773 tests: labels `unit` + `integration`, plus `vocabulary` when `$VOYAHSLAM_ORBVOC` is set).

Recorded baselines (corridor1, Release, from `docs/ROADMAP.md`). The two sets use different run lengths and are not directly comparable:

| Run | ATE baseline | Where recorded |
|---|---|---|
| Mono, first 400 frames, no loop closing | Sim3 0.032-0.033 m (0.0323-0.0336 m with vocabulary + relocalization) | step 1, step 3 |
| Stereo, first 1000 frames, no loop closing | SE3 0.036-0.037 m (0.0349-0.0369 m with vocabulary + relocalization) | step 1, step 3 |
| Mono, full sequence, no loop closing | Sim3 0.341 / 0.500 m (two runs) | step 4a+4b |
| Stereo, full sequence, no loop closing | SE3 0.098 / 0.138 m (two runs) | step 4a+4b |
| Mono / stereo, full sequence, WITH loop closing (today, opt-in) | Sim3 0.679 / 0.669 m; SE3 1.295 / 18.16 m (worse: the case Phase 1 must fix) | step 4a+4b |

No new numeric bars are invented in this planning set. Where a source states no acceptance bar, the requirement says "to be specced" and the bar is set with the user when that phase is specced.

## Requirements

### Validated

Shipped, with their acceptance results recorded in `docs/DESIGN.md` section 8 and `docs/ROADMAP.md`. Full text in `.planning/intel/requirements.md` (Part A). These are existing behaviour and are not re-opened; changing one needs explicit discussion.

- ✓ `REQ-event-memory-infrastructure` - lock-free pooled, ref-counted, acknowledged events (pool grows on overflow since 0.3.0)
- ✓ `REQ-data-layer` - strong ids, SoA `MapPointStorage`, `Frame`, `CovisibilityIndex`, `StereoRig`, measurements
- ✓ `REQ-component-threads-and-state-machines` - `Slam<FE>` wiring, `WorkerThread`/`WakeSignal`, Tracker and Mapper state machines
- ✓ `REQ-two-view-geometry` - H/E estimators, generic RANSAC, `TwoViewReconstructor`
- ✓ `REQ-mono-initialization` / `REQ-stereo-initialization` - real map initialization (two-keyframe BA; stereo identity keyframe)
- ✓ `REQ-camera-model-raw-pixel` - `geometry::Camera`, raw-pixel residuals, Brown-Conrady and Kannala-Brandt
- ✓ `REQ-tracking-against-map` - motion model, zero-motion fallback, reference-Frame tracking, `OptimizePose`
- ✓ `REQ-keyframes-and-map-growth` / `REQ-map-quality` - Mapper-decided Frames, fusion, culling, local BA, `MapTransaction`
- ✓ `REQ-bow-vocabulary` - own OpenCV-free DBoW2 port, `SearchByBoW`
- ✓ `REQ-relocalization` - `RecentlyLost`, `FrameDatabase`, P3P RANSAC, motion-model gate
- ✓ `REQ-library-restructure-and-fast-tests` - four libraries, `check` / `check_all`
- ✓ `REQ-loop-closing` (acceptance gap: corridor1 Musts missed, shipped opt-in by user decision 2026-10-10; gap carried by LOOP-01..04)
- ✓ `REQ-global-ba` - own component with its own replica, merged as one transaction
- ✓ `REQ-map-merging-atlas` - multi-map Atlas, `MergeLocal`, replica rule (thin integration margins carried by MERGE-01)
- ✓ `REQ-map-viewer` - `voyahviz::MapViewer` over event-only `SceneState`
- ✓ `REQ-opencv-free-libraries` - FAST ported bit-exact, SIFT removed, no OpenCV in `voyah_*`
- ✓ `REQ-packaging-and-android` - Conan 2 package 0.3.0, Android arm64 profile (device runs pending, PLAT-01)

### Active

The 20 pending requirements, all mapped to phases in `.planning/ROADMAP.md`. Full text with sources and acceptance in `.planning/REQUIREMENTS.md`.

- [ ] Reject loop-closing look-alikes and false merges so loop closing and merging can be enabled on corridor1 (LOOP-01..04, MERGE-01) - Phase 1
- [ ] Bound global BA time and memory; relocalize into inactive maps (GBA-01, ATLAS-01) - Phase 2
- [ ] ORB extractor rework, geometry robustness, descriptor-traits layering, own vocabulary (EXTR-01, GEOM-01, LAYR-01, VOCAB-01) - Phase 3
- [ ] Lossy observer delivery, storage and pool sizing, Tracker measurement policy, platform validation (EVT-01, MEM-01, TRKR-01, PLAT-01) - Phase 4
- [ ] Remote visualization and serialization, component late joining, mixed mono/stereo maps (REMOTE-01, LATE-01, MIXED-01) - Phase 5
- [ ] TUM-VI tuning and dataset validation (TUNE-01, DATA-01) - Phase 6

### Out of Scope

- IMU / visual-inertial fusion and metric scale without an IMU - DESIGN Open Question 10 and the loop-closing spec list it as out of scope; mono scale drift is accepted.
- SIFT and other non-ORB extractors - removed by user decision (2026-10-11); templates are instantiated for ORB only.
- OpenCV inside the four `voyah_*` libraries - principle 8; only `voyahviz`, tests and benchmarks link it.
- Locks on the map or event path - principle 1; any new component talks through events of plain data.
- Configuration-file parsing in the library - configuration lives in the user application (`examples/full_slam/` is the reference).
- Committing the 145 MB `ORBvoc.txt` - never committed; supplied through `$VOYAHSLAM_ORBVOC`.

## Context

- **Source of truth.** `docs/DESIGN.md` (architecture, principles, decisions log, open questions; wins over any older spec), `docs/ROADMAP.md` (order of upcoming work) and `docs/TODO.md` (pending and deferred items, about 193 open) remain the project's own documents. `.planning/ROADMAP.md` is derived from `docs/ROADMAP.md` (step 6, split across Phase 1 and Phase 6) plus the strategic open items of `docs/TODO.md` (Phases 2-5). **It must be kept in sync with `docs/ROADMAP.md`**: when a step is added, reordered, finished or dropped there, update `.planning/ROADMAP.md` (and `.planning/REQUIREMENTS.md` traceability) in the same change, and the other way round. Deferred work still goes to `docs/TODO.md`; design decisions still go to the DESIGN decisions log in the same change.
- **Status.** Done and merged (steps 0-5): restructure, map quality, own DBoW2, relocalization, loop detection and correction (opt-in), global BA, map merging. Remaining: ROADMAP step 6, "TUM-VI tuning", whose first item is loop-closing look-alike rejection. Loop closing, global BA and merging are opt-in (`--loop_closing`) and stay off by default until look-alikes are rejected.
- **Reference data.** TUM-VI `dataset-corridor1_1024_16` (20 Hz stereo, 1024 x 1024, equidistant fisheye, baseline about 0.101 m) with a mocap room at the start and end, so end-of-run ATE measures loop-closing quality. Tuning is specified on `dataset-corridor1_512_16`. EuRoC (Brown-Conrady) has never been run on real images.
- **Look-alike diagnosis.** Same-heading corridor look-alikes 7-30 m apart pass every ORB-SLAM3 visual gate and the added normal-consistency gate; a diagnosis document is cited in TODO (`.superpowers/sdd/2026-10-09-loop-closing/false-loop-diagnosis.md`) but is git-excluded and not present in the working tree.
- **Open design questions** (DESIGN section 9): OQ 1 id width, 2 storage growth, 9 culling propagation, 10 IMU, 14 pool sizing, 17 mixed mono/binocular, 19 Tracker measurement policy, 21 ORB extraction speed, 22 fisheye rays beyond 90 degrees, 25 event pool lifetime. Those still pending are mapped to Phases 3-5.
- **Parallel workstream outside the docs.** The user's project memory records an in-flight `voyah_jni` ownership audit (`/home/vahagn/git/voyah_jni/docs/voyah_slam_ownership_audit.md`): items #1 and #4 are done (0.2.0, 0.3.0); #3 (pooled events hold a raw back-pointer to their pool) with #2 (RAII subscriptions from `AddObserver`), then #5 (map follower, core of a planned `voyahslam::Session` facade) and #6-#9 are next. These items are not in `docs/TODO.md` or `docs/ROADMAP.md`, so they are not requirements here. They overlap Phase 4 (event pool lifetime, observer disconnect, lossy delivery). Add them to `docs/TODO.md` first if they should become planned work.
- **Stale items to prune** (see `.planning/INGEST-CONFLICTS.md`, INFO): TODO items about `MapCreated` / the old `map_id` consumer rule, the done "immutable shared measurement" item and the decided Faiss-vs-DBoW2 spike; DESIGN.md's "Last updated: 2026-10-08" header and the "merge pending" lines in section 10.
- **Codebase map.** `.planning/codebase/` (ARCHITECTURE, CONCERNS, CONVENTIONS, INTEGRATIONS, STACK, STRUCTURE, TESTING; mapped at b0e957c).

## Constraints

- **Tech stack**: C++20, Eigen 3.4, spdlog, moodycamel `concurrentqueue`, g2o (vendored pristine as the local Conan package `g2o/20241228.1@voyah/stable`, linked PRIVATE), GTest; Conan 2 + CMake >= 3.23; GCC 13 verified (Clang 14+, Apple Clang 15+ listed). The package requires `compiler.cppstd >= 20`.
- **Platform**: Linux x86_64 (primary) and Android arm64-v8a (API 24, clang 21, NDK r30). Android uses a single JNI `.so` and `c++_static`; if a second C++ `.so` is ever added, everything must switch to `c++_shared`. `shared=True` is rejected on Windows and with `c++_static`.
- **Concurrency**: no mutex on the map or event path; Mapper is the only map writer; Tracker, LoopCloser, GlobalBA and the viewer never mutate map objects; components exchange plain data only (no pointers into map objects) so an out-of-process transport stays possible.
- **Identity**: `FrameId` unique per run and never reused; `MapPointId` append-only per map, restarting per map, so `(map_id, id)` identifies a point; `MapId` starts at 1 and only increases.
- **Layering**: `voyah_common`; `voyah_events` and `voyah_algs` each on common only; `voyah_slam` on all three; `voyahslam` umbrella; `voyahviz` on common + events. A lower layer including an upper header must fail to compile. Public headers never include OpenCV, Pangolin or g2o.
- **Conventions**: Google style (`PascalCase` types and functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference outputs, `kPascalCase` constants, `T` prefix for template parameters); include guards `VOYAHSLAM_<PATH>_H`; one event type per header; `INSTANTIATE_CLASS` for ORB only; `.clang-format` 4 spaces; new sources registered in `libs/<lib>/CMakeLists.txt`.
- **Tests**: `check` (label `unit`, about 10 s) is the fast loop; `check_all` (adds `integration`, `vocabulary`; about 30 s) runs at the end of a task and before a merge; a test above 1 s in the optimized build belongs in `libs/slam/tests/integration/`. Re-run ThreadSanitizer after every loop-closing or global-BA change. Tests, apps and tools build only with `VOYAHSLAM_BUILD_TESTS` / `VOYAHSLAM_BUILD_EXAMPLES` (OFF by default).
- **Process**: brainstorm, spec, plan, implement task by task on a feature branch with tests, review, merge. Nothing is merged or pushed without an explicit "merge and push" from the user. Every design decision is recorded in `docs/DESIGN.md` (decisions log plus the relevant section) in the same change.
- **Performance budgets on record**: triangulation >= 3x faster with a vocabulary; FAST at least as fast as `cv::FAST` single-threaded; relocalization about 10 ms per attempt; local BA about 105 ms mono / 135 ms stereo per Frame; stereo corridor1 peak RSS about 1.2 GB (global BA adds a replica; 4.23 GB measured before measurement sharing).

## Key Decisions

<decisions>
status: LOCKED. There are no ADRs; the effective locked set is `docs/DESIGN.md` section 2 (principles) and section 8 (decisions log), plus the rules in `CLAUDE.md`. Full text of all 214 entries with sources: `.planning/intel/decisions.md`. Plans and phases must not contradict an entry below without the user explicitly amending it first, and the amendment is recorded in the DESIGN decisions log.

**Principles (locked)**
- PRINCIPLE-1 No locks on the map or event path; cross-thread communication only via events on `moodycamel::ConcurrentQueue`; event pools are lock-free.
- PRINCIPLE-2 One replica of the map per thread (Tracker, Mapper, LoopCloser, GlobalBA, viewer).
- PRINCIPLE-3 Mapper is the single writer; every change is published as a diff every replica applies verbatim.
- PRINCIPLE-4 Other threads are read-only on the map; they publish requests or facts.
- PRINCIPLE-5 IDs are never reused (see Constraints for the exact scopes).
- PRINCIPLE-6 No virtual dispatch on hot paths; events dispatch on a type tag; mono/stereo chosen at compile time.
- PRINCIPLE-7 Structure-of-Arrays storage for map points and observations.
- PRINCIPLE-8 OpenCV is an implementation detail (none at all in the four `voyah_*` libraries since 2026-10-11).

**Architecture decisions (locked)**
- DEC-2026-10-07-2 `MapTransaction` is the only Mapper event, applied atomically in one fixed order by every consumer; it must never be dropped (pool default 256, grows on overflow).
- DEC-2026-10-10-6 The replica rule: every replica mirrors the Mapper's Atlas; the active map comes from `active_map` only, never from the newest id; unknown or removed maps are ignored.
- DEC-2026-10-06-2 The Mapper, not the Tracker, decides Frame creation (`ShouldCreateFrame`).
- DEC-2026-10-02-2 Loop-closure and merge corrections are made by the Mapper; LoopCloser and GlobalBA only detect or compute.
- DESIGN section 6 pointer rule: an event may point only at data that never changes; `TSharedMeasurement<FE>` (2026-10-10) and `TDistortionModelPtr` (2026-10-12) are `shared_ptr<const ...>`; everything mutable travels by value.
- DEC-2026-10-05-9 Residuals and gates are in raw distorted pixels (sigma = `sigma_px * level_scale^level`); rays stay rays for two-view RANSAC and triangulation.
- DEC-2026-10-07-1 Four libraries under `libs/<lib>/{include,src,tests}` with separate include roots and unchanged `#include <voyahslam/...>` paths.
- DEC-2026-10-02-8 / DEC-2026-10-12-3 Pooled objects are default-constructed once and kept alive; `Create()` takes no arguments and now grows by doubling instead of aborting; `TryCreate()` never grows (LoopCloser and GlobalBA drop on purpose).
- DEC-2026-10-02-13 Configuration lives in the user application; the library has no config-file parsing.
- DEC-2026-10-05-2 g2o is vendored pristine, packaged as a local Conan recipe, linked PRIVATE.

**User decisions (locked)** - each is an explicit `[USER DECISION]` in the DESIGN log
- DEC-2026-10-10-1 Loop closing is opt-in and off by default (`LoopCloserOptions::enabled` false; apps `--loop_closing`). Do not flip the default without the user's explicit approval, even after Phase 1.
- DEC-2026-10-09-15 Normal-consistency gate in the LoopCloser (a deliberate deviation from ORB-SLAM3); verify-before-publish was rejected for that first round.
- DEC-2026-10-10-8 GlobalBA owns a replica fed from the start; no late joining: a component either exists from the start or not at all. (LATE-01 would amend this; see below.)
- DEC-2026-10-10-9 Components stay pluggable through gRPC later: GlobalBA and LoopCloser talk to the Mapper only through events of plain data.
- DEC-2026-10-10-23 Map merging scope A with ORB-SLAM3 parity (keep lost maps of at least 10 Frames inactive; relocalization stays active-map-only until ATLAS-01).
- DEC-2026-10-11-10 / -11 Own FAST, bit-exact with OpenCV 5, at least as fast as `cv::FAST` single-threaded.
- DEC-2026-10-11-12 SIFT extractor removed.
- DEC-2026-10-11-14 Android profile uses `c++_static`.
- DEC-2026-10-12-1 Distortion models are shared and immutable (`TDistortionModelPtr`); null means pinhole.
- DEC-2026-10-03-6 No OpenCV types in public headers (user requirement).

**Process rules (locked)** - RULE-WORKFLOW (spec, plan, task-by-task; nothing merged without "merge and push"), RULE-DOCS-LIVING (DESIGN.md kept current in the same change; DESIGN wins over specs), RULE-NAMING, RULE-LAYERING.

**Superseded - do not treat as current**: pool `Create()` aborting on exhaustion (now grows); "newest map id wins" and the `map_id` ignore-older/adopt-newer consumer rule (now the replica rule); Tracker decides keyframes; `ObservationStorage<Stereo>` in the Mapper (now `CovisibilityIndex` only); Mapper-only BoW (BoW travels in transactions, Tracker keeps a `FrameDatabase`); normalized-plane BA residuals; loop closing on by default and `--no_loop_closing`; raw distortion pointer rule; Conan 0.1.0 and 0.2.0; SIFT and OpenCV-linked `voyah_algs`.

**Amendments that later phases would need** - LATE-01 (Phase 5) contradicts DEC-2026-10-10-8 as written; MIXED-01 (Phase 5) builds on the locked long-term goal DEC-2026-10-02-12 (binding decided later, OQ 17); LOOP-03 (Phase 1) reopens an option DEC-2026-10-09-15 rejected "for the first round". Each needs an explicit user decision in that phase's discuss step.
</decisions>

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Look-alike rejection mechanism for loops: drift-bound gate (LOOP-02), verify-before-publish (LOOP-03), or both; the 2026-10-11 user decision favours a drift-consistency check (LOOP-04) but does not say whether it replaces LOOP-02 or whether LOOP-03 is added | `docs/TODO.md` "TUM-VI tuning" says "compare on corridor1 and decide with the user"; merges cannot use a drift check (two maps share no world) and need verify-before-publish | — Pending: OPEN, decide in `/gsd-discuss-phase 1` |
| Brownfield intake: shipped capabilities are Validated, only pending items are phased | Guidance for this ingest; keeps the roadmap about what is left | ✓ Good |
| Step 6 is split: look-alike rejection is Phase 1, the remaining tuning is the last phase | ROADMAP says tuning is "at the very end" and look-alike rejection is its first item; tuned thresholds depend on the extractor, geometry and Tracker work in Phases 3-4 | — Pending user review of the ordering |
| Phases 2-5 are groupings of open `docs/TODO.md` items, ordered by dependency and theme | `docs/ROADMAP.md` has no steps after 6; order is a planning proposal, not a user decision | — Pending user review |
| `.planning/` mirrors, and never replaces, `docs/ROADMAP.md` and `docs/TODO.md` | The project's workflow and CLAUDE.md name those files as the source of truth | ✓ Good |
| Defaults assumed because no `.planning/config.json` existed: standard granularity, sequential phase ids | Roadmapper defaults | — Pending config |

---
*Last updated: 2026-10-12 after initialization from ingest (brownfield, 41 documents, no ADRs/PRDs)*
