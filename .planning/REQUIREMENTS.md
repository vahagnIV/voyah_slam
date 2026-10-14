# Requirements: voyahslam

**Defined:** 2026-10-12
**Core Value:** ORB-SLAM3-class accuracy on real sequences (TUM-VI corridor, mono and stereo) delivered through a lock-free, replica-per-thread, event-only architecture.

Brownfield intake. The 19 shipped capabilities are Validated (see `.planning/PROJECT.md` and `.planning/intel/requirements.md` Part A) and are not re-listed as work. The 27 requirements below are the pending items from `docs/ROADMAP.md` ("API hardening", the voyah_jni ownership audit, added 2026-10-12, and step 6) and the strategic open items of `docs/TODO.md`. Source names in brackets are the entries in `.planning/intel/requirements.md` Part B.

Acceptance rule: where a source states no bar, the entry says "to be specced". No numeric bar is invented. Every phase also carries the project success metric as a regression guard: ATE on TUM-VI corridor (mono + stereo) no worse than the recorded baselines (PROJECT.md "Success Metric") and `check_all` green.

## v1 Requirements

### API ownership and lifetime contracts (Phase 1)

Source: `../voyah_jni/docs/voyah_slam_ownership_audit.md` (TODO "Ownership audit"); audit items #1 and #4 are already merged (0.2.0, 0.3.0). Target principle: a type owns, or shares ownership of, everything it points to after a call returns; required orderings are removed or enforced, not just documented.

- [x] **OWN-01** (audit #3, OQ 25): Events no longer point back to a pool that may be destroyed: since 2026-10-13 (user decision) events are heap-allocated and published as `std::shared_ptr<const T>`, `ObjectPool` and `Recyclable` are deleted, and the `global_ba_` ordering invariant (`slam.h`, DESIGN) and the apps' `StopAll` guard are removed. Acceptance: an event released after its producer is destroyed is still valid and freed exactly once (test); otherwise to be specced. *[ownership audit #3]*
- [x] **OWN-02** (audit #2): Observer registration does not keep a raw pointer that can dangle: an RAII subscription returned by `AddObserver` (or observers held by `shared_ptr`), covering `ConnectAsync`, `ConnectSync`'s `ctx` and `SetWakeSignal`; connecting or disconnecting while the component runs fails loudly. Acceptance: an observer destroyed before its producer is not notified again (test); the examples no longer need a declaration order. *[ownership audit #2]*
- [x] **OWN-03** (audit #5): Thread-affinity accessors (`Mapper` "Mapper thread only"; `Tracker::atlas()`, `map_replica()`, `last_matches()`) are enforced (test access or a debug thread assert), and apps get a supported way to read the map from their own thread (for example a map follower consuming `MapTransaction` into its own replica, the core of a planned `voyahslam::Session` facade). Acceptance: to be specced. *[ownership audit #5]*
- [x] **OWN-04** (audit #6): Lifecycle state machine (Created, Running, Stopped) with asserts or errors on invalid transitions (`AddObserver` while running, `Start` twice, `Start` after `Stop`, `Track()` after `Stop()`; `Track()` before `Start()` queues); restart after `Stop()` decided and documented. Acceptance: to be specced. *[ownership audit #6]*
- [x] **OWN-05** (audit #7): `OrbFeatureExtractor::DetectAndExtract` is either safe to call concurrently (per-call or caller-provided workspace) or non-const. Acceptance: to be specced. *[ownership audit #7]*
- [x] **OWN-06** (audit #8): `Logger::Initialize` also replaces the sinks of loggers created before it, and `sinks_` is protected against concurrent reads. Acceptance: a component constructed before `Initialize` logs to the new sinks (test). *[ownership audit #8]*
- [ ] **OWN-07** (audit #9): `images::Image` correctness: default construction initializes every member; `Convert()`, `operator*=` and `operator*` honour `offset_` and `stride_` of views; `Set()` is non-const and the buffer aliasing of views is documented. Acceptance: tests on a sub-image view. *[ownership audit #9]*

### Loop closing (Phase 2)

- [ ] **LOOP-01**: With `--vocabulary --loop_closing` on corridor1 (Release, full sequence, mono and stereo), same-heading look-alike loops are not corrected while true returns still are, and ATE with loop closing is no worse than without it. Look-alike examples: frames 2215-2236 vs 692-807 (14.9 m apart); 3210 to 792 (22 m); 4666 to 4029 (22 m); 5017 to 2950 (30 m); also the opposite-facing 2262 to 1587. Recorded no-loop-closing baselines: mono Sim3 0.341 / 0.500 m, stereo SE3 0.098 / 0.138 m. The normal-consistency gate is retested after the fix. *Outcome shared by LOOP-02..04; bar inferred from the loop-closing spec smoke Must, TODO states the goal not numbers. [REQ-loop-closing acceptance gap; REQ-loop-lookalike-rejection-*]*
- [ ] **LOOP-02**: Candidate mechanism 1 (OPEN, user must choose): a drift-bound gate rejects a loop whose query and match region lie farther apart in the map than drift could explain since the match, for example a bound per metre travelled; safe in stereo while drift << 15 m, mono needs a scale-aware bound. Acceptance: LOOP-01. *[REQ-loop-lookalike-rejection-drift-bound]*
- [ ] **LOOP-03**: Candidate mechanism 2 (OPEN, user must choose): verify before publishing; check the reprojection error of the fused points after the essential graph, or the next local BA's initial cost, and drop a correction that breaks the map. Rejected for the first round on 2026-10-09, listed again as a candidate; candidate 3 is LOOP-02/04 plus this. Acceptance: LOOP-01. *[REQ-loop-lookalike-rejection-verify-before-publish]*
- [ ] **LOOP-04**: Drift-consistency check for loops (user decision 2026-10-11, favoured; relation to LOOP-02 not stated): compare the loop's Sim3 with the map's own estimate of the query-match pose and reject when the difference exceeds what drift could accumulate along the path between them; bound rotation and translation per metre travelled; mono also needs a scale bound. Does not apply to merges (two maps share no world). Acceptance: the diagnosed example 2262 to 1587 (map puts the Frames 9.2 m and 176 degrees apart, the Sim3 claims 0.2 m and 6 degrees, stereo drift about 0.1 m) is rejected. *[REQ-loop-drift-consistency-check]*

### Map merging (Phase 2)

- [ ] **MERGE-01**: Map merges get a hypothesis-independent false-match check (likely verify-before-publish, since the normal gate is not applied across maps and the drift check does not apply), while merging stays opt-in. Acceptance: no false merge on the corridor1 `--drop_frames` smoke (every observed merge so far was true); the thin integration margins are widened (map 1 holds 10-11 Frames against `atlas.min_frames_to_keep` 10; stereo seed 2 whole-run ratio 1.33-1.40 against the 1.5 bar). *[REQ-merge-false-match-protection]*

### Global BA and Atlas (Phase 3)

- [ ] **GBA-01**: Global BA is bounded and cheap enough to leave on: an iteration or size cap or time budget (stereo corridor1 runs took up to 85 s, interruptions land only at the next LM iteration, up to about 6 s, and `Slam::Stop()` waits 6-8 s); memory re-measured and reduced (about 1 MB per Frame; peak RSS 4.23 GB stereo with global BA against 2.28 GB without, measured before measurement sharing); ~~a dropped `GlobalBAFinished` is retried~~ (moot: Phase 1 D-25 removes the drop, 2026-10-13); map loss after loop corrections is re-measured once look-alikes are rejected. Budgets: to be specced. *[REQ-global-ba-hardening]*
- [ ] **ATLAS-01**: A Tracker lost in the active map can relocalize into an inactive Atlas map (today relocalization is active-map-only). Also in scope as TODO sub-items: MLPnP as an alternative, a candidate cap, a scale-distance gate in `SearchFromFrame`. Acceptance: to be specced. *[REQ-relocalization-into-inactive-maps]*

### Feature front-end and geometry (Phase 4)

- [ ] **EXTR-01**: ORB extractor rework (OQ 21): run FAST once per pyramid level and assign corners to cells (>= 20 if any, else >= 7), removing the undetectable 6-px strips at cell borders; preallocate blur and pyramid buffers so extraction allocates nothing per frame; check the 19-px border handling; extract left and right in parallel; profile 8-bit against float working images; re-record the ORB golden digests (`VOYAHSLAM_RECORD_ORB_GOLDEN=1`). Acceptance: not stated numerically; re-profile after the change. *[REQ-orb-extractor-rework]*
- [ ] **GEOM-01**: Geometry robustness: 5-point essential solver (priority raised for narrow FOV); fix the refinement inlier-set lock-in; unit bearing vectors instead of z = 1 points so features beyond 85 degrees survive on fisheye (OQ 22); `rational_polynomial` distortion; per-model camera classes only if needed. Acceptance: to be specced. *[REQ-geometry-robustness]*
- [ ] **LAYR-01**: Event, map and viewer code is templated on descriptor traits instead of the extractor class, so `voyah_common` no longer declares `OrbFeatureExtractor`. Acceptance: code linking only common that calls `DetectAndExtract` fails at compile time rather than at link time. *[REQ-descriptor-traits-refactor]*
- [ ] **VOCAB-01**: Own vocabulary training (k-majority), a recorded position on the `ORBvoc.txt` licence, and the phantom-node difference against ORB-SLAM3 settled. Deferred by user decision earlier; acceptance: to be specced. *[REQ-vocabulary-training-and-licence]*

### Consumer runtime and platform (Phase 5)

- [ ] **EVT-01**: Per-observer delivery mode (for example `AddObserver(o, Delivery::Lossy)`) with a bounded queue that drops and releases the oldest events, for display-only consumers. Never for `MapTransaction` replicas or the Mapper's `FrameTracked` input. Acceptance: a stalled display-only observer no longer grows the Tracker or Mapper pools without bound. *[REQ-lossy-observer-delivery]*
- [ ] **MEM-01**: Decide and implement `MapPointStorage` growth and compaction (append-only ids leave dead slots, the Tracker scans all ids per frame; OQ 2), Event pool sizing (OQ 14) and lifetime (OQ 25) are gone with the pools (OWN-01, Phase 1). Acceptance: to be specced. *[REQ-storage-growth-and-pool-sizing]*
- [ ] **TRKR-01**: Decide the Tracker measurement policy (track every measurement in order, or skip to the newest when falling behind, possibly configurable; OQ 19) and whether `Tracker::Stop()` drains or discards queued measurements. Acceptance: to be specced. *[REQ-tracker-measurement-policy]*
- [ ] **PLAT-01**: Platform validation: FAST NEON path bit-exact against the arm64 reference and timed with `fast_benchmark`, and `test_package` run on an arm64 device; Android logcat sink; symbol visibility for shared builds; a real package licence (placeholder `proprietary` today); ASan runs no longer abort in g2o (Eigen malloc ODR mix); g2o under clang/libc++; `check_all` also builds the apps. *[REQ-platform-validation]*

### Remote components and mixed maps (Phase 6)

- [ ] **REMOTE-01**: Event serialization and transport (HTTP, ROS or gRPC; choice open) so SLAM and the viewer, and later pluggable components, run on different machines; a separate image flow for the camera view; inactive Atlas maps in the viewer; VBO rendering for large maps; macOS setup. Constraint: consumers use only event contents (plain data, no pointers into map objects). Acceptance: to be specced. *[REQ-remote-visualization-and-serialization]*
- [ ] **LATE-01**: A component starting mid-run (GlobalBA, LoopCloser, remote) builds its replica from a full-map snapshot. Amends the locked user decision DEC-2026-10-10-8 ("exists from the start or not at all"); needs the user's explicit approval. Acceptance: to be specced. *[REQ-component-late-joining]*
- [ ] **MIXED-01**: Monocular and binocular keyframes coexist in one map (a stereo rig whose second camera drops out; merging mono and stereo sessions); today `Frame<FE, TMeasurement>` fixes one type per map (OQ 17). Long-term goal beyond ORB-SLAM3. Acceptance: behaviour of mono-then-stereo and stereo-then-mono sequences to be designed and specced. *[REQ-mixed-mono-stereo-maps]*

### Tuning and dataset validation (Phase 7)

- [ ] **TUNE-01**: Tune every map-quality, initialization and stereo threshold on TUM-VI 512 (`dataset-corridor1_512_16`): local-map inlier gate (30 against 20), initialization acceptance and parallax, rescale before init BA, stereo depth cap (about 17 m), mono translation accuracy at default init, `ShouldCreateFrame` parameters. Acceptance: not stated; to be specced. A change is adopted only where ATE is no worse than the recorded baselines. *[REQ-tumvi-tuning]*
- [ ] **DATA-01**: Real-data run on EuRoC MH_01 (mono and stereo, Brown-Conrady; never run on EuRoC images) with tracking, maps and ATE (`tools/ate.py`) reported; own CLAHE as app-side preprocessing; benchmark on TUM-VI 512x512. Acceptance: tracking, maps and ATE reported. *[REQ-dataset-validation]*

## v2 Requirements

None. All 27 pending requirements are mapped to phases. The remaining open items of `docs/TODO.md` (deferred review findings, test gaps, flaky-test notes, tuning notes; about 193 open in total) stay in that file, which remains the tracker; they are pulled into a phase at its discuss or plan step when they touch its scope. Phase 6 is the farthest-out work and the first candidate to move here if the user wants a shorter roadmap.

## Out of Scope

| Feature | Reason |
|---------|--------|
| IMU / visual-inertial fusion; metric scale without an IMU | DESIGN OQ 10 and the loop-closing spec list it as out of scope |
| SIFT and other non-ORB extractors | Removed by user decision 2026-10-11 |
| OpenCV in the `voyah_*` libraries | Principle 8; only `voyahviz`, tests and benchmarks link it |
| Locks on the map or event path | Principle 1 |
| Configuration-file parsing in the library | Configuration lives in the user application |
| Committing the 145 MB `ORBvoc.txt` | Supplied through `$VOYAHSLAM_ORBVOC` |
| Switching loop closing on by default | User decision 2026-10-10; revisit only with the user's explicit approval after Phase 2 |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| OWN-01 | Phase 1 | Complete |
| OWN-02 | Phase 1 | Complete |
| OWN-03 | Phase 1 | Complete |
| OWN-04 | Phase 1 | Complete |
| OWN-05 | Phase 1 | Complete |
| OWN-06 | Phase 1 | Complete |
| OWN-07 | Phase 1 | Pending |
| LOOP-01 | Phase 2 | Pending |
| LOOP-02 | Phase 2 | Pending (candidate, open decision) |
| LOOP-03 | Phase 2 | Pending (candidate, open decision) |
| LOOP-04 | Phase 2 | Pending (user decision 2026-10-11, scope open) |
| MERGE-01 | Phase 2 | Pending |
| GBA-01 | Phase 3 | Pending |
| ATLAS-01 | Phase 3 | Pending |
| EXTR-01 | Phase 4 | Pending |
| GEOM-01 | Phase 4 | Pending |
| LAYR-01 | Phase 4 | Pending |
| VOCAB-01 | Phase 4 | Pending |
| EVT-01 | Phase 5 | Pending |
| MEM-01 | Phase 5 | Pending |
| TRKR-01 | Phase 5 | Pending |
| PLAT-01 | Phase 5 | Pending |
| REMOTE-01 | Phase 6 | Pending |
| LATE-01 | Phase 6 | Pending |
| MIXED-01 | Phase 6 | Pending |
| TUNE-01 | Phase 7 | Pending |
| DATA-01 | Phase 7 | Pending |

**Coverage:**
- v1 requirements: 27 total (7 ownership-audit items OWN-01..07, added 2026-10-12; 19 pending entries of `.planning/intel/requirements.md` Part B, plus LOOP-01, the outcome statement shared by the three competing look-alike entries)
- Mapped to phases: 27
- Unmapped: 0

**Open decision (not a coverage gap):** LOOP-02, LOOP-03 and LOOP-04 are alternatives or complements. The user picks which are built in `/gsd-discuss-phase 2`; the ones not chosen move to Out of Scope with the reason, and LOOP-01 stays the acceptance for whatever is built.

---
*Requirements defined: 2026-10-12*
*Last updated: 2026-10-12: ownership audit inserted as Phase 1 (OWN-01..07), later phases renumbered*
