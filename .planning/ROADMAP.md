# Roadmap: voyahslam

## Overview

voyahslam is already implemented: tracking, mapping, relocalization, loop closing, global BA and map merging are merged (ROADMAP steps 0-5). What is left is `docs/ROADMAP.md` step 6, whose first item is rejecting same-heading look-alike loops so loop closing, global BA and merging can be switched on for TUM-VI corridor1 without making ATE worse, plus the strategic open items of `docs/TODO.md`. The journey: first finish the in-flight API hardening for library consumers, so every ownership and lifetime rule is enforced by the types (Phase 1, the `voyah_jni` ownership audit), then make loop closing trustworthy (Phase 2), harden what sits on top of it (Phase 3), improve the feature front-end and geometry (Phase 4), make the library fit an Android/JNI consumer under load (Phase 5), open the architecture to remote components and mixed camera maps (Phase 6), and finish with TUM-VI tuning and a second real dataset (Phase 7), keeping ATE on the corridor sequences no worse than the recorded baselines and `check_all` green throughout.

**Sync rule:** this file is derived from `docs/ROADMAP.md` ("API hardening" is Phase 1; step 6 is split across Phase 2 and Phase 7) and `docs/TODO.md` (Phases 3-6). Those two files remain the project's source of truth. When `docs/ROADMAP.md` changes, update this file and the traceability in `.planning/REQUIREMENTS.md` in the same change.

**Regression guard for every phase:** ATE on TUM-VI corridor (mono Sim3, stereo SE3) no worse than the baselines in `.planning/PROJECT.md` "Success Metric", and `check_all` (about 773 tests) green. Nothing is merged or pushed without the user's explicit "merge and push".

<details>
<summary>Shipped before this roadmap (docs/ROADMAP.md steps 0-5, not phased here)</summary>

- Tracking against the map, camera model (raw-pixel residuals), keyframes and map growth (merged b93fcf4)
- Step 0 restructure into four libraries with fast tests (7426697, 2026-10-07)
- Step 1 map quality: `MapTransaction`, fusion, culling, interruptible local BA (9da7081)
- Step 2 own OpenCV-free DBoW2 (6fb863a)
- Step 3 relocalization and reference-Frame tracking (6afacc7, 2026-10-08)
- Step 4a+4b loop detection and correction, opt-in and off by default (merged 2026-10-10; corridor1 Musts missed, carried into Phase 2)
- Step 4c global BA (9245f92, 2026-10-10)
- Step 5 map merging / multi-map Atlas (9326160, 2026-10-11)
- Later hardening: OpenCV-free libraries and own FAST, Conan package, Android profile, shared distortion models (0.2.0), growing object pools (0.3.0)

</details>

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

Decimal phases appear between their surrounding integers in numeric order.

- [ ] **Phase 1: API Ownership and Lifetime Contracts** - Finish the voyah_jni ownership audit: self-owning `shared_ptr<const T>` events (pools removed) and RAII observer subscriptions first, then thread affinity, lifecycle, extractor, logger and image fixes (ROADMAP "API hardening", inserted 2026-10-12)
- [ ] **Phase 2: Loop-Closing False-Positive Rejection** - Reject same-heading look-alike loops and false merges so loop closing and merging stop worsening corridor1 (ROADMAP step 6, first item)
- [ ] **Phase 3: Multi-Map and Global BA Hardening** - Bound global BA time and memory and let a lost Tracker relocalize into inactive maps
- [ ] **Phase 4: Feature Front-End and Geometry Robustness** - Per-level FAST extractor, 5-point solver and full-FOV rays, descriptor-traits layering, own vocabulary
- [ ] **Phase 5: Consumer Runtime and Platform Readiness** - Lossy delivery for display observers, storage and pool sizing, Tracker measurement policy, arm64/Android validation
- [ ] **Phase 6: Remote Components and Mixed-Camera Maps** - Serialization and remote viewer, component late joining, mono and stereo keyframes in one map
- [ ] **Phase 7: TUM-VI Tuning and Dataset Validation** - Tune the flagged thresholds on corridor1 512 and validate on EuRoC (ROADMAP step 6, "at the very end")

## Phase Details

### Phase 1: API Ownership and Lifetime Contracts

**Goal**: A library consumer whose objects die in any order (the Java/JNI binding in `../voyah_jni`, or any C++ app) cannot crash the library through declaration order, destruction order or calls from the wrong thread: every type owns or shares what it points to after a call returns, and every required ordering is removed or enforced by an assert or an error.
**Depends on**: Nothing (in flight: audit items #1 and #4 are merged as 0.2.0 and 0.3.0)
**Requirements**: OWN-01, OWN-02, OWN-03, OWN-04, OWN-05, OWN-06, OWN-07
**Success Criteria** (what must be TRUE):
  1. An event, `Listener` or `AsyncObserver` that outlives the component that published it stays valid and is freed exactly once (a test destroys the producer before the consumer releases); events are `std::shared_ptr<const T>` and `ObjectPool` / `Recyclable` are gone (decision 2026-10-13); and the `global_ba_` ordering invariant in `slam.h` and the apps' `StopAll` guard are gone.
  2. An observer destroyed before its producer is never notified again (a test), and connecting or disconnecting while a component runs fails loudly; the examples no longer depend on declaring the viewer and `TrajectoryWriter` before `Slam`.
  3. The "Mapper thread only" / "never while running" accessors cannot be reached from an app thread while running (test access or a debug thread assert), and apps have a supported way to read the map from their own thread.
  4. Invalid lifecycle transitions (`AddObserver` while running, `Start` twice, `Track` before `Start`) assert or return an error, and whether `Stop()` then `Start()` is supported is decided and documented.
  5. Two threads may call `DetectAndExtract` concurrently, or the API makes that impossible to do by mistake; `Logger::Initialize` also re-targets loggers created before it; a default-constructed `Image` is empty and view operations honour `offset_` / `stride_`.
  6. Each item has a DESIGN decisions-log entry, `check_all` is green, ThreadSanitizer is clean on the event and Slam tests, and the Conan packages (linux-gcc13 static and shared, android-arm64) are recreated with a version the voyah_jni session is told about.

**Plans**: 7/20 plans executed (six releases, sequential; each release ends with the user's "merge and push")

Plans:
**Wave 1**
- [x] 01-01-PLAN.md — R1 tracer: voyah_events delivery layer (EventPtr, InboxCore, Subscription, UsageError), destruction-order matrix, TSan/ASan

**Wave 2** *(blocked on Wave 1 completion)*
- [x] 01-02-PLAN.md — R1: voyah_slam, voyahviz and the apps on make_shared events and Subscriptions; pools, drop-limits, global_ba_ invariant and StopAll removed

**Wave 3** *(blocked on Wave 2 completion)*
- [x] 01-03-PLAN.md — R1: Tracker and Mapper test migration

**Wave 4** *(blocked on Wave 3 completion)*
- [x] 01-04-PLAN.md — R1: LoopCloser, GlobalBA, replica, matcher and voyahviz test migration; delivery-while-held tests

**Wave 5** *(blocked on Wave 4 completion)*
- [x] 01-05-PLAN.md — R1: slam_test migration, Slam-level lifetime/wiring tests, pools deleted, -Werror=unused-result, check_all

**Wave 6** *(blocked on Wave 5 completion)*
- [x] 01-06-PLAN.md — R1: DESIGN §6.1 rewrite, exception policy, TODO/ROADMAP/README, version 0.4.0

**Wave 7** *(blocked on Wave 6 completion)*
- [x] 01-07-PLAN.md — R1 release gates, "merge and push" checkpoint, merge, voyah_jni message (0.4.0)

**Wave 8** *(blocked on Wave 7 completion)*
- [ ] 01-08-PLAN.md — R2 tracer: MapFollower and MapSnapshot, examples' PrintAtlas on the follower

**Wave 9** *(blocked on Wave 8 completion)*
- [ ] 01-09-PLAN.md — R2: Tracker/LoopCloser/GlobalBA accessors (incl. LoopCloser stats) private behind TestAccess, GlobalBA counters atomic

**Wave 10** *(blocked on Wave 9 completion)*
- [ ] 01-10-PLAN.md — R2: Mapper accessors private behind MapperTestAccess; docs; version 0.5.0

**Wave 11** *(blocked on Wave 10 completion)*
- [ ] 01-11-PLAN.md — R2 release gates, checkpoint, merge (0.5.0)

**Wave 12** *(blocked on Wave 11 completion)*
- [ ] 01-12-PLAN.md — R3 tracer: CAS lifecycle in EventNode and the Tracker; Mapper/LoopCloser/GlobalBA/Slam

**Wave 13** *(blocked on Wave 12 completion)*
- [ ] 01-13-PLAN.md — R3: lifecycle docs (Stopped is terminal), version 0.6.0, TSan stress

**Wave 14** *(blocked on Wave 13 completion)*
- [ ] 01-14-PLAN.md — R3 release gates, checkpoint, merge (0.6.0)

**Wave 15** *(blocked on Wave 14 completion)*
- [ ] 01-15-PLAN.md — R4: extractor Workspace, concurrency tests, examples, docs, version 0.7.0

**Wave 16** *(blocked on Wave 15 completion)*
- [ ] 01-16-PLAN.md — R4 release gates (incl. ORB golden digests), checkpoint, merge (0.7.0)

**Wave 17** *(blocked on Wave 16 completion)*
- [ ] 01-17-PLAN.md — R5: logger shared dist sink, atomic level, docs, version 0.8.0

**Wave 18** *(blocked on Wave 17 completion)*
- [ ] 01-18-PLAN.md — R5 release gates, checkpoint, merge (0.8.0)

**Wave 19** *(blocked on Wave 18 completion)*
- [ ] 01-19-PLAN.md — R6: Image view fixes and overflow guard, docs, version 0.9.0

**Wave 20** *(blocked on Wave 19 completion)*
- [ ] 01-20-PLAN.md — R6 release gates, checkpoint, merge (0.9.0)

**Cross-cutting constraints:**
- Nothing is merged into master or pushed until the user types 'merge and push' (D-21, project rule)
- check_all is green, both example apps build, and a 40-frame corridor1 run of each app exits 0 (D-21)
- AddressSanitizer is clean on voyah_events_tests

**Notes**: Source: `../voyah_jni/docs/voyah_slam_ownership_audit.md` (TODO "Ownership audit"). Order agreed earlier: #3 with #2 first (designed together), then #5, #6, #7, #8, #9. Each item is its own branch, merged on the user's explicit "merge and push" with a minor version bump. Takes OQ 25 (event pool lifetime) from MEM-01; 2026-10-13 the pools are removed instead (events are coarse, allocation cost negligible).

### Phase 2: Loop-Closing False-Positive Rejection

**Goal**: Loop closing and map merging can be enabled on TUM-VI corridor1 without making the result worse, because same-heading look-alike loops and false merges are rejected while true returns are kept.
**Depends on**: Nothing in code (sequencing: after Phase 1)
**Requirements**: LOOP-01, LOOP-02, LOOP-03, LOOP-04, MERGE-01
**Success Criteria** (what must be TRUE):
  1. On corridor1 (Release, `--vocabulary --loop_closing`, full sequence, mono and stereo, two runs each as in the recorded smoke) the documented same-heading look-alikes (2215-2236 vs 692-807, 3210 to 792, 4666 to 4029, 5017 to 2950) and the opposite-facing 2262 to 1587 loop are not corrected, and the true return to the start room at the end of the run is still corrected.
  2. ATE with loop closing on is no worse than with it off, mono Sim3 0.341 / 0.500 m and stereo SE3 0.098 / 0.138 m (recorded no-loop-closing runs); no map is lost to a false correction.
  3. The merge smoke (`--drop_frames` on corridor1) shows no false merge, and the thin merge integration margins are wider than today (map 1 holds 10-11 Frames against the keep threshold 10; stereo whole-run ratio 1.33-1.40 against the 1.5 bar).
  4. The normal-consistency gate is retested after the fix and no longer rejects true returns (in the 2026-10-09 smoke it rejected 178 stereo and 70 mono return candidates after a false correction), and the synthetic `SlamTest.{Mono,Stereo}ClosesLoop` / `*MergesMaps` integration tests still pass.

**Plans**: TBD
**Open decision (for `/gsd-discuss-phase 2`, do not pre-empt)**: which mechanism is built for loops. Options recorded in `docs/TODO.md` "TUM-VI tuning": (1) a drift-bound gate (LOOP-02), (2) verify-before-publish: reprojection error after the essential graph or the next local BA's initial cost (LOOP-03; rejected for the first round on 2026-10-09, listed again), (3) both. The user decision of 2026-10-11 asks for a drift-consistency check (LOOP-04: loop Sim3 against the map's own estimate, bounded per metre travelled, plus a mono scale bound); it is close to option 1 but is not stated to replace it. A drift check cannot apply to merges (two maps share no world), so MERGE-01 needs verify-before-publish or another hypothesis-independent check whatever is chosen for loops. Compare on corridor1 and decide with the user.
**Notes**: Loop closing stays opt-in; flipping its default needs the user's explicit approval (DEC-2026-10-10-1). Re-run ThreadSanitizer after any loop-closing change. The false-loop diagnosis file cited in TODO is not in the working tree.

### Phase 3: Multi-Map and Global BA Hardening

**Goal**: With false positives rejected, the global BA and Atlas stack is bounded in time and memory, and a Tracker that is lost can relocalize into an older map instead of only the active one.
**Depends on**: Phase 2 (map loss after loop corrections can only be re-measured once look-alikes are rejected)
**Requirements**: GBA-01, ATLAS-01
**Success Criteria** (what must be TRUE):
  1. A global BA run is bounded by a cap or time budget (value set in the phase spec): stereo corridor1 runs no longer take up to 85 s, and `Slam::Stop()` no longer waits 6-8 s for a run to finish.
  2. Stereo corridor1 peak RSS with loop closing and global BA is re-measured against the recorded figures (4.23 GB before measurement sharing, 2.28 GB without global BA) and the extra memory is reduced or bounded by a stated budget.
  3. ~~A dropped `GlobalBAFinished` is retried~~ — moot since 2026-10-13: Phase 1 (D-25) publishes it unbounded, so it is never dropped (a Phase 1 test shows a result delivered while an earlier one is held).
  4. After a blackout, a Tracker lost in the active map relocalizes into an inactive Atlas map in a synthetic scenario, and the corridor1 `--drop_frames` smoke is unchanged otherwise.

**Plans**: TBD

### Phase 4: Feature Front-End and Geometry Robustness

**Goal**: Feature extraction and two-view geometry have no known blind spots or per-frame allocations, descriptor-specific code no longer leaks into the lowest layer, and the project controls its own vocabulary.
**Depends on**: Phase 2 (sequencing only; no code dependency)
**Requirements**: EXTR-01, GEOM-01, LAYR-01, VOCAB-01
**Success Criteria** (what must be TRUE):
  1. A corner on a cell border is detected (no 6-px undetectable strip), extraction allocates nothing per frame in steady state, and the new ORB golden digests are recorded with the output change stated; extraction time is re-profiled and reported against the OQ 21 profile (cell-based FAST was 57 % of extraction time).
  2. Narrow-FOV two-view initialization recovers the pose through a 5-point solver, and refinement no longer locks onto an initial inlier set (shown by synthetic scenes that are rejected or weak today).
  3. A fisheye feature beyond 85 degrees off axis is kept and triangulates correctly, and a `rational_polynomial` camera projects and unprojects round trip.
  4. Code that links only `voyah_common` and calls `DetectAndExtract` fails at compile time, `voyah_common` no longer declares `OrbFeatureExtractor`, and `check_all` is green.
  5. A vocabulary trained by the project's own tool loads and serves relocalization and loop detection on corridor1 with ATE no worse than with `ORBvoc.txt` (bar to be specced), and the `ORBvoc.txt` licence position is recorded.

**Plans**: TBD
**Notes**: Re-record the ORB golden digests deliberately (`VOYAHSLAM_RECORD_ORB_GOLDEN=1`); the extractor and geometry changes shift what Phase 7 tunes, which is why tuning comes last. Sub-items of GEOM-01 may be scoped down at spec time.

### Phase 5: Consumer Runtime and Platform Readiness

**Goal**: An Android/JNI (or any embedded) consumer can run the library under load and on arm64 without unbounded memory growth, with the platform paths actually validated.
**Depends on**: Phase 1 (event ownership and subscriptions; lossy delivery builds on them)
**Requirements**: EVT-01, MEM-01, TRKR-01, PLAT-01
**Success Criteria** (what must be TRUE):
  1. A stalled display-only observer using lossy delivery no longer grows its inbox (and the events it holds) without bound, and `MapTransaction` replicas and the Mapper's `FrameTracked` input never use lossy delivery (shown by a test).
  2. The `MapPointStorage` growth/compaction question (OQ 2) is decided in the DESIGN decisions log and implemented (event pool sizing, OQ 14, and lifetime, OQ 25, disappear with the pools in Phase 1).
  3. The Tracker's behaviour when it falls behind (track in order or skip to the newest) is chosen and tested, and `Tracker::Stop()` drains or discards queued measurements as documented.
  4. The FAST NEON path is bit-exact against the arm64 reference and timed with `fast_benchmark` on real arm64 hardware, `test_package` runs on a device, and an Android logcat sink exists.
  5. The package has a real licence in place of the placeholder, shared builds have symbol visibility, ASan runs no longer abort in g2o, and `check_all` also builds the apps.

**Plans**: TBD
**Notes**: Builds on Phase 1 (event ownership, subscriptions, lifecycle). Event pools are removed in Phase 1 (OWN-01, 2026-10-13); the Android logcat sink here pairs with OWN-06 (logger initialization order).

### Phase 6: Remote Components and Mixed-Camera Maps

**Goal**: SLAM, the viewer and pluggable components can run in separate processes or machines, and a single map can hold keyframes from both monocular and binocular measurements.
**Depends on**: Phase 1 and Phase 5 (event ownership, subscriptions and delivery modes underpin a transport)
**Requirements**: REMOTE-01, LATE-01, MIXED-01
**Success Criteria** (what must be TRUE):
  1. A viewer in a separate process receives serialized events over a chosen transport and shows the same scene as the in-process viewer, built only from event contents (references are serialized as values).
  2. The viewer shows inactive Atlas maps, receives the camera image on a separate flow, and renders a large map without per-frame immediate-mode cost.
  3. A component started mid-run (GlobalBA, LoopCloser or a remote one) builds its replica from a full-map snapshot and then follows transactions, ending equal to a replica that started with the run (a test compares them).
  4. A mono-then-stereo and a stereo-then-mono sequence run in one map without tracking loss (behaviour and bars to be designed and specced).

**Plans**: TBD
**UI hint**: yes
**Notes**: LATE-01 amends the locked user decision DEC-2026-10-10-8 and needs the user's explicit approval; MIXED-01 settles OQ 17. The UI is the Pangolin desktop viewer, not a web frontend. This is the farthest-out phase and the first candidate to defer to v2.

### Phase 7: TUM-VI Tuning and Dataset Validation

**Goal**: Every threshold flagged for TUM-VI tuning is tuned on corridor1 512 with a recorded before/after, and the library is validated on a second real dataset family, ending with ATE no worse than the recorded baselines.
**Depends on**: Phase 2, Phase 4, Phase 5 (extractor, geometry and Tracker policy change what is tuned); Phase 6 is independent but runs earlier so tuning is last, as `docs/ROADMAP.md` says ("at the very end")
**Requirements**: TUNE-01, DATA-01
**Success Criteria** (what must be TRUE):
  1. Each flagged item has a before/after result on `dataset-corridor1_512_16` for mono and stereo: local-map inlier gate (30 against 20), initialization acceptance and parallax, rescale before init BA, stereo depth cap (about 17 m), mono translation accuracy at default init, and `ShouldCreateFrame` parameters. A change is adopted only where ATE is no worse than the recorded baselines.
  2. Final ATE on the corridor sequences (mono Sim3, stereo SE3), with and without loop closing, is no worse than the recorded baselines, and `check_all` is green (the project success metric).
  3. EuRoC MH_01 runs end to end in mono and stereo (Brown-Conrady) with tracking, maps and ATE (`tools/ate.py`) reported, and TUM-VI 512x512 is benchmarked.
  4. CLAHE is available as app-side preprocessing, and the tuned defaults are recorded in the DESIGN decisions log with the baseline tables in `docs/ROADMAP.md` updated.

**Plans**: TBD
**Notes**: The recorded baselines were taken on corridor1 (reference `dataset-corridor1_1024_16`); record a 512 baseline first if the 512 numbers differ. Acceptance bars for individual thresholds are to be specced when the phase starts.

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5 → 6 → 7 (decimal insertions in between if needed)

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. API Ownership and Lifetime Contracts | 7/20 | In Progress (audit #1, #4 merged before GSD) | - |
| 2. Loop-Closing False-Positive Rejection | 0/TBD | Not started | - |
| 3. Multi-Map and Global BA Hardening | 0/TBD | Not started | - |
| 4. Feature Front-End and Geometry Robustness | 0/TBD | Not started | - |
| 5. Consumer Runtime and Platform Readiness | 0/TBD | Not started | - |
| 6. Remote Components and Mixed-Camera Maps | 0/TBD | Not started | - |
| 7. TUM-VI Tuning and Dataset Validation | 0/TBD | Not started | - |
