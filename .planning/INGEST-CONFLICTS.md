## Conflict Detection Report

Ingest: 41 classified documents (22 SPEC, 19 DOC; 0 ADR, 0 PRD, 0 UNKNOWN). Mode: new. Precedence applied: docs/DESIGN.md (override 0) > SPEC > PRD > DOC, later-dated spec/plan supersedes earlier on the same topic, docs/superpowers/plans/* treated as history.

### BLOCKERS (0)

(none) No ADR carries `locked: true`, so no LOCKED-vs-LOCKED contradiction is possible. The DESIGN.md decisions log (the effective locked set) was checked internally: every later row that reverses an earlier row says so explicitly (see INFO entries 1 and 2), so there is no unresolved contradiction inside it. No UNKNOWN or low-confidence classification exists. No existing .planning context (merge mode) was ingested, so no existing-vs-ingest locked conflict applies.

### WARNINGS (3)

[WARNING] Competing candidate fixes for loop-closing look-alike rejection (REQ pending, roadmap step 6)
  Found: docs/TODO.md "TUM-VI tuning" lists three candidate fixes to "compare on corridor1 and decide with the user": (1) a drift-bound gate, (2) verify-before-publish (reprojection error after the essential graph, or the next local BA's initial cost), (3) both. The next item in the same section records a user decision of 2026-10-11 for a "drift-consistency check for loops" (compare the loop Sim3 with the map's own estimate, bounded per metre travelled), which is close to candidate 1 but is not stated to replace the comparison. docs/DESIGN.md decisions log 2026-10-09 records verify-before-publish as "rejected option" for the first round. Merges are stated to need verify-before-publish because two maps share no world.
  Impact: The roadmapper cannot tell whether step 6 means drift-bound, verify-before-publish, or both for loops, so the phase scope and its acceptance test cannot be fixed without losing intent.
  → Variants preserved as REQ-loop-lookalike-rejection-drift-bound and REQ-loop-lookalike-rejection-verify-before-publish, plus the decided REQ-loop-drift-consistency-check and REQ-merge-false-match-protection in .planning/intel/requirements.md. Choose one (or both) before routing step 6.

[WARNING] Cross-reference graph contains cycles (treated as benign mutual citations, not blockers)
  Found: Resolving `cross_refs` between documents gives one strongly connected component of 22 documents and 27 direct two-way pairs. The hubs are docs/DESIGN.md, docs/ROADMAP.md and docs/TODO.md, each of which cites specs that cite them back (for example docs/DESIGN.md and docs/superpowers/specs/2026-10-05-tracking-design.md; docs/ROADMAP.md and docs/superpowers/specs/2026-10-09-loop-closing-design.md; docs/DESIGN.md and docs/TODO.md). The other members are 12 other specs, docs/superpowers/plans/2026-10-07-map-quality.md and .superpowers/spikes/2026-10-07-stereo-drift.md.
  Impact: Under a strict reading of the synthesizer rule (cycle = unresolved blocker, skip the cyclic set) 22 of 41 documents, including DESIGN.md, would be excluded and nothing useful could be synthesized. The references here are "see also" back-links in living documents, not dependency or supersession chains, and this synthesis does not order anything by cross-reference (order comes from precedence, dates and the ROADMAP), so no synthesis loop can occur.
  → Treated as non-blocking and synthesized in full. Deviation from the strict rule is recorded here for explicit acknowledgement; if you want the strict behaviour, re-run with a manifest that excludes the back-links.

[WARNING] Pending requirements without stated acceptance criteria
  Found: Several pending items have no measurable bar in any source: TUM-VI tuning (ROADMAP step 6 has no spec yet), ORB extractor rework, relocalization into inactive maps, remote visualization/serialization, late joining, global-BA caps, mixed mono/stereo maps, storage growth, Tracker measurement policy, geometry robustness, platform validation. Only look-alike rejection has a diagnosed example and corridor1 baselines, and even there the pass bar is inferred from the loop-closing spec's smoke Must (ATE no worse than without loop closing).
  Impact: Roadmapper will have to invent acceptance bars or leave them open; invented bars would not be traceable to a source.
  → Each such entry in .planning/intel/requirements.md says "not stated". Set the bars with the user when each step is specced (the project workflow is spec then plan), or accept "to be specced" phases.

### INFO (14)

[INFO] Auto-resolved: DESIGN.md (precedence 0) over the 2026-09-30 core data-structures spec
  Note: docs/superpowers/specs/2026-09-30-core-slam-data-structures-design.md says ThreadSafePolicy is a `std::mutex`, `Recycle()` calls the destructor, `Create(args...)`, relaxed atomics, MapPointStorage free-list id reuse, `R_rl`/`t_rl`, `BinocularCameraFrame`, and an event list of `NewKeyFrameCandidate`, `MapPointsAdded/Updated/Removed`, `KeyFrameCreated`, `KeyFramePoseUpdated`, `PositionUpdated`. docs/DESIGN.md wins: lock-free Treiber stack (2026-10-02), pooled objects kept alive and `Create()` without arguments (2026-10-02), acq_rel (2026-09-30), append-only point ids (2026-10-05), `T_rl` (2026-10-04), `BinocularMeasurement` (2026-10-02), `FrameTracked` and `MapTransaction` (2026-10-06, 2026-10-07). Synthesized intel follows DESIGN.md.

[INFO] Auto-resolved: DESIGN.md decisions-log reversals recorded inside DESIGN.md itself
  Note: Later rows supersede earlier rows and say so: pool `Create()` aborts on exhaustion (2026-10-02) -> grows (2026-10-12); "newest map id wins" (2026-10-03) -> replica rule (2026-10-10); Tracker decides keyframes (2026-10-02) -> Mapper decides (2026-10-06); `ObservationStorage<Stereo>` in the Mapper (2026-10-04) -> `CovisibilityIndex` only (2026-10-07); Mapper-only BoW (2026-10-07) -> BoW in transactions and Tracker FrameDatabase (2026-10-08); Faiss/DBoW2 deferral (2026-10-06) -> own DBoW2 (2026-10-07); normalized-plane BA residual (2026-10-05) -> raw pixels (2026-10-05, later); loop closing default on (2026-10-09) -> opt-in (2026-10-10); raw distortion pointer rule -> `shared_ptr` (2026-10-12); Conan package 0.1.0 -> 0.2.0 -> 0.3.0. decisions.md marks each superseded row.

[INFO] Auto-resolved: later-dated spec over earlier on tracker states and map identity
  Note: docs/superpowers/specs/2026-10-03-tracking-states-design.md defines Tracker states `WaitingForMap/Tracking/Lost`, the `MapCreated` marker event, the `map_id` "ignore older, reset-and-adopt newer" consumer rule and the stub `PoseEstimator`/`MapInitializer`. Superseded by DESIGN.md: `RecentlyLost` (2026-10-08), `MapTransaction.new_map`, the replica rule with `active_map`/`removed_maps`/`absorbed_map` (2026-10-10), real estimators. docs/superpowers/specs/2026-10-04-pangolin-viewer-design.md "newest map wins" is likewise replaced by the replica rule (2026-10-10).

[INFO] Auto-resolved: camera-model spec (2026-10-05) over init-BA spec (2026-10-05) on residual space
  Note: docs/superpowers/specs/2026-10-05-init-bundle-adjustment-design.md specifies residuals on the undistorted normalized plane; docs/superpowers/specs/2026-10-05-camera-model-design.md and the DESIGN log ("Superseded later on 2026-10-05") move BundleAdjust, OptimizePose, projection matcher, stereo gate and init matcher to raw pixels. Same-day dates; the camera-model spec is later by DESIGN's own ordering. Rays stay rays for two-view RANSAC and triangulation.

[INFO] Auto-resolved: DESIGN.md over the stereo-initialization and tracking specs
  Note: docs/superpowers/specs/2026-10-04-stereo-initialization-design.md ("Mapper keeps both a mono and a stereo observation storage") is reversed by DESIGN 2026-10-07. docs/superpowers/specs/2026-10-05-tracking-design.md ("No BoW fallback", zero-motion x4 replaces `TrackReferenceKeyFrame`, `PositionUpdated`/`TrackingLost` events) holds only without a vocabulary since 2026-10-08; with one, reference-Frame tracking and relocalization apply.

[INFO] Auto-resolved: keyframes spec stage names and event names
  Note: docs/superpowers/specs/2026-10-06-keyframes-design.md names `KeyFrameCreated` and `TrackingStage::{MotionModel, ZeroMotion, LocalMap}`. DESIGN 2026-10-07 replaces the event with `MapTransaction`; DESIGN 2026-10-08 ruling C replaces the stage set with `MotionModel/ZeroMotion/ReferenceFrame/Relocalization` (`LocalMap` removed). DESIGN wins.

[INFO] Auto-resolved: loop-closing spec over by DESIGN decisions on defaults and flags
  Note: docs/superpowers/specs/2026-10-09-loop-closing-design.md uses `--no_loop_closing`, an enabled-by-default LoopCloser, and a smoke Must "ATE below the no-loop run". DESIGN 2026-10-10 (user decision) makes loop closing opt-in with `--loop_closing`. The spec's thresholds and `S_query_to_match` name were amended in DESIGN 2026-10-09 (RANSAC inliers 15, `OptimizeSim3` 20, projection 80, coarse 50, `S_query_match`). The corridor1 Musts remain missed and are carried as pending requirements; the user accepted shipping opt-in (ROADMAP step 4).

[INFO] Auto-resolved: global-BA spec bar revised after measurement
  Note: docs/superpowers/specs/2026-10-10-global-ba-design.md section 5 states a mono points-RMS bar of 0.8x and annotates its revision to 0.95x per seed / 0.92x average (end error 1.05x). DESIGN 2026-10-10 agrees. Section 1's first backlog-based `should_stop` is superseded the same day by apply-on-poll (DESIGN 2026-10-10 Task 2 ruling).

[INFO] Auto-resolved: SIFT and OpenCV statements in older specs
  Note: Specs 2026-10-04 mono-initialization, 2026-10-04 two-view-geometry, 2026-10-07 bow (`Vocabulary<FE>` for ORB and SIFT) and the DESIGN rows on SIFT levels/`max_distance` mention SIFT and an OpenCV-linked algs library. DESIGN 2026-10-11 removes SIFT (user decision) and makes the `voyah_*` libraries OpenCV-free. docs/superpowers/specs/2026-10-03-map-viewer-design.md's pointer rule ("data that lives for the whole run (calibration)") is superseded by the shared-ownership pointer rule (DESIGN section 6, 2026-10-12).

[INFO] Auto-resolved: TODO.md items stale against DESIGN.md
  Note: (a) docs/TODO.md "Immutable shared measurement in events ... copy the measurement (about 80 KB per frame)" is done: DESIGN 2026-10-10 map merging Task 1 shares `TSharedMeasurement<FE>`. (b) "Faiss vs DBoW2/FBoW spike" is decided (own DBoW2, 2026-10-07); the item itself says revisit only if needed. (c) "Before relocalization / multi-map: the Tracker adopts a newer map's id but drops its `MapCreated`" and "`SceneState` and `LoopCloser` don't apply the `map_id` consumer rule" refer to `MapCreated` and a rule replaced on 2026-10-10 by the replica rule; also "A newer `MapCreated` while `Tracking`" in the Threads section. (d) TODO's own note records stale `KeyFrameCreated`/`MapCreated` text in older DESIGN sections. These TODO items were not carried into requirements.md; consider pruning them.

[INFO] Staleness inside DESIGN.md and CLAUDE.md
  Note: docs/DESIGN.md header says "Last updated: 2026-10-08" although the log runs to 2026-10-12. DESIGN section 10 "Done" still says "merge pending" for branches that docs/ROADMAP.md records as merged (ROADMAP wins on status). DESIGN section 7 quotes 521/533 tests and section 10 quotes 735/764 tests; CLAUDE.md states 746 (`check`) and 773 (`check_all`) and is the most recent, so those counts are used. DESIGN's "Required follow-up" and "Next" lists overlap with TODO.md and ROADMAP.md; ROADMAP governs order.

[INFO] Plans classified SPEC treated as history
  Note: docs/superpowers/plans/2026-10-05-tracking.md, 2026-10-10-global-ba.md and 2026-10-10-map-merging.md were classified SPEC (medium confidence) because they embed interfaces, but per the ingest override all docs/superpowers/plans/* are history. Their sibling specs (2026-10-05-tracking-design.md, 2026-10-10-global-ba-design.md, 2026-10-10-map-merging-design.md) are the authority and no constraint was taken from the plans. docs/superpowers/specs/2026-10-07-restructure-timings.md (SPEC) is a measurement baseline; its counts (458/459 tests) are historical and appear only as context.

[INFO] Accepted acceptance gaps (user-decided, not conflicts)
  Note: Missed smoke Musts on record: camera model (<= 5 re-initializations, got 18; cured later by map growth), keyframes (first mono map >= 150 frames, stereo no loss; cured by map quality), BoW stereo ATE regression (cured by the motion-model gate), loop closing on corridor1 (ATE worse with loop closing; shipped opt-in by user decision 2026-10-10, carried into step 6). Global BA and map merging have no ATE Must by design because corridor1 false loops dominate. Merge integration margins are thin (stereo ratio 1.33-1.40 vs bar 1.5; map 1 holds 10-11 Frames vs the keep threshold 10) and tracked in TODO.

[INFO] Memory index and CLAUDE.md are consistent with the synthesized set
  Note: The roadmap order in the user's memory (voyah_jni ownership audit, then the next tracking sub-project work) was not an ingested document and was not used; it does not contradict ROADMAP.md, which ends at step 6. Codebase map in .planning/codebase/ was not ingested.
