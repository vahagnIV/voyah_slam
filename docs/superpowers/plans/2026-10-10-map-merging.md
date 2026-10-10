# Map Merging (Multi-Map Atlas) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep lost maps as inactive maps, detect when the active map revisits one, and merge the active map into it (ORB-SLAM3 Atlas + `MergeLocal`).

**Architecture:**
- **Shared measurements.** A Frame's measurement becomes a shared, immutable object.
- **Atlas everywhere.** Each replica becomes an Atlas: `tracking::AtlasReplica<FE>` holds one `MapReplica<FE>` per map plus the active map id. The single-map algorithms keep taking a `MapReplica`.
- **Mapper.** Its per-map state moves into a `MapState` held by id, with one map active.
- **Merge detection** reuses `DetectCommonRegion` against per-map `FrameDatabase`s.
- **The merge** is a new `mapper_merge.cpp`. It reuses the 4b correction steps and publishes one `MapTransaction` with `absorbed_map` set. Every replica applies it with the same id rule.

**Tech Stack:** C++20, Eigen, g2o (PRIVATE in algs), GoogleTest, CMake/Conan 2.

**Spec:** `docs/superpowers/specs/2026-10-10-map-merging-design.md`

## Global Constraints

- **Locks and writers.** No locks on map data. The Mapper is the only map writer. Threads communicate only via events.
- **Ids.**
  - Frame ids are never reused in a run.
  - Map ids are never reused.
  - Point ids are append-only per map; `(map_id, id)` identifies a point.
- **Opt-in.** Merging is part of loop closing: opt-in under `--loop_closing`, off by default. With default flags, behaviour is as on master, except for the shared measurements and the Atlas bookkeeping.
- **Gates.** Merges never get looser gates than loops: the same `LoopCloserOptions` and the normal-consistency gate.
- **ORB-SLAM3 values.**
  - `min_frames_to_keep` 10 (`Tracking` reset vs `CreateMapInAtlas`).
  - `merge_window` 25 (`numTemporalKFs`).
  - Fuse radius 4.
  - Mono merges use a free-scale Sim3; stereo uses scale 1.
- **The replica rule** replaces "newest id wins".
  - Apply a transaction to the map it names.
  - Ignore unknown or removed maps.
  - The active map comes only from `MapTransaction::active_map`.
- **Conventions.** Google style naming, the `out_` prefix, and current DESIGN.md / TODO.md / ROADMAP.md.
- **Tests.** Fixed seeds or deterministic scenes, with margin. Never hand-pick a seed.
- **Commands** are in CLAUDE.md. At the end of every task run `check_all` plus a manual standalone compile of the changed headers.
- **Known pre-existing flakes:** `SlamTest.StereoRelocalizesAfterBlackout`, `SlamTest.MonoRelocalizesAfterBlackout`, `SlamTest.DuplicatesFusedInLongSequence`.
- **ThreadSanitizer** runs in a separate build tree under `setarch -R`.

## Review Focus

1. **A `FrameTracked` of the absorbed map M, in flight when the merge lands.** Expected: no Frame is created in a removed map, and nothing crashes. Test: Task 5, `MapperMergeTest.IgnoresFrameTrackedOfAbsorbedMap`.
2. **A global-BA result for M arriving after M was absorbed.** Expected: it is rejected, because the map was removed. Test: Task 5, `MapperMergeTest.RejectsGlobalBaOfAbsorbedMap`.
3. **The same place detected again after the merge,** now within one map. Expected: it is treated as an already-closed loop, so there is no second correction. Test: Task 5, `MapperMergeTest.IgnoresRepeatOfAppliedMerge`.
4. **The Tracker is RecentlyLost when a merge lands.** Expected: no pose shift; it relocalizes in N with translated point ids. Test: Task 6, `TrackerMergeTest.MergeWhileRecentlyLost`.
5. **A final loss of a map with exactly 9 / 10 Frames.** Expected: 9 is discarded (`removed_maps`), 10 is kept inactive. Test: Task 3, `MapperAtlasTest.KeepsMapsFromTenFrames`.

---

### Task 1: Shared, immutable measurements

**Files:**
- Modify:
  - `libs/events/include/voyahslam/events/frame_tracked.h` (`measurement`)
  - `libs/events/include/voyahslam/events/map_transaction.h` (`NewFrame::measurement`)
  - `libs/common/include/voyahslam/map/frame.h` (`measurement_`)
  - `libs/slam/include/voyahslam/tracking/map_replica.h` (`ReplicaKeyFrame::measurement`)
  - every producer and consumer: Tracker, Mapper, map initializer, LoopCloser, GlobalBA, viewer/apps, and tests
- Test: `libs/slam/tests/integration/slam_test.cpp` (`SlamTest.MeasurementsAreShared`)

**Interfaces:**
- Produces: `template <typename FE> using TSharedMeasurement = std::shared_ptr<const features::TMeasurement<FE>>;` in `voyahslam/features/measurement.h`. Every place above holds a `TSharedMeasurement<FE>`.
  - Accessors keep returning `const TMeasurement&`, so `Frame::measurement()` and `ReplicaKeyFrame` users read through it.
  - Only the Tracker creates one, with `std::make_shared` once per tracked frame.
- **Immutability:** `const` by type. If any code mutates a measurement after creation, report it and rule the smallest change.

- [ ] **Step 1: Failing test.** `SlamTest.MeasurementsAreShared` runs a short stereo scene, then for a given Frame checks that:
  - the Mapper's Frame, the Tracker's replica and the LoopCloser's replica hold the same measurement address (`&frame.measurement()` equal);
  - the `shared_ptr` use count is ≥ 3.
- [ ] **Step 2: Run** `check`. Expected: compile failure.
- [ ] **Step 3: Implement.** This is a mechanical type change; behaviour must stay identical. No test except the new one may change its assertions.
- [ ] **Step 4: Run** `check_all`, Release and the apps. Measure peak RSS of a full stereo corridor1 run with default flags plus `--vocabulary` (`/usr/bin/time -v`; dataset `/home/vahagn/Downloads/dataset-corridor1_1024_16`, vocabulary `/home/vahagn/data/vocabulary/ORBvoc.bin`). Master's figure is about 1.7 GB peak at 1780 Frames. Report the measured number; there is no bar here.
- [ ] **Step 5: Commit** `git commit -m "Shared immutable measurements (shared_ptr<const>)"`

### Task 2: Transaction fields and `AtlasReplica`

**Files:**
- Create: `libs/slam/include/voyahslam/tracking/atlas_replica.h`; test: `libs/slam/tests/tracking/atlas_replica_test.cpp`
- Modify:
  - `map_transaction.h` (fields)
  - `map_replica.h` (remove the map-id admission; one map only)
  - Tracker, LoopCloser, GlobalBundleAdjuster and the viewer scene (find it by grepping `AdmitMap` / `new_map`): switch to `AtlasReplica`
  - Mapper: fill `active_map` on every transaction, and `removed_maps` on a final loss

**Interfaces:**
- Produces:
  ```cpp
  // MapTransaction additions
  std::optional<map::MapId> active_map;      // active map after applying; set on every transaction
  std::vector<map::MapId> removed_maps;      // discarded / absorbed maps
  std::optional<map::MapId> absorbed_map;    // Task 5: merge of absorbed_map into Event::map_id
  // tracking/atlas_replica.h
  template <typename FE> class AtlasReplica {
  public:
      void Apply(const events::MapTransaction<FE>& transaction);    // the replica rule
      [[nodiscard]] std::optional<map::MapId> active_map() const;
      [[nodiscard]] const MapReplica<FE>* active() const;           // null if none
      [[nodiscard]] const MapReplica<FE>* map(map::MapId id) const; // null if unknown/removed
      [[nodiscard]] std::vector<map::MapId> map_ids() const;        // live maps, ascending
  };
  ```
- **Replica rule:**
  - A transaction with `new_map` creates its map.
  - Other transactions apply to an existing live map; transactions naming unknown or removed maps are ignored.
  - Then `removed_maps` are dropped, and `active_map` is taken.
  - Removed ids are remembered and never re-admitted.
  - The `absorbed_map` handling is Task 5's; here, assert it is unset.
- **Behaviour in this task:** unchanged. On a final loss the Mapper still discards its map, now expressed as `removed_maps = {old}`. The Tracker, LoopCloser and GlobalBA work on `active()` exactly as before.

- [ ] **Step 1: Failing tests** in `AtlasReplicaTest`:
  - `NewMapCreatesAndActivates`
  - `IgnoresUnknownAndRemovedMaps`
  - `RemovedMapNeverReadmitted`
  - `ActiveMapOnlyFromTransaction`
  - `KeepsInactiveMaps`: two maps, the first inactive, both live.
- [ ] **Step 2: Run** `check`. Expected: compile failure.
- [ ] **Step 3: Implement**, and switch every replica holder.
- [ ] **Step 4: Run** `check_all` and TSan on `voyah_slam_tests`. Expected: green, with every existing test unchanged.
- [ ] **Step 5: Commit** `git commit -m "MapTransaction active_map/removed_maps; AtlasReplica replaces newest-id-wins"`

### Task 3: The Mapper's Atlas

**Files:**
- Modify: `libs/slam/include/voyahslam/mapping/mapper.h`, `libs/slam/src/mapping/mapper*.cpp`
- Test: `libs/slam/tests/mapping/mapper_test.cpp` (suite `MapperAtlasTest`)

**Interfaces:**
- Produces:
  - `MapperOptions::atlas` with `min_frames_to_keep = 10` and `merge_window = 25`.
  - A private `struct MapState`, which groups every per-map member: Frames, points, covisibility, references, spanning tree, loop edges, `map_epoch_`, next point id and recent points.
  - `std::map<map::MapId, MapState> maps_` plus `std::optional<map::MapId> active_`. Existing code works on `Active()`.
  - Test accessors: `map_ids()`, `active_map()`, and `num_keyframes(map::MapId)`.
- **Final loss of the active map:**
  - fewer than 10 Frames: discard it and publish `removed_maps = {id}, active_map = nullopt`;
  - otherwise: keep it inactive and publish `active_map = nullopt`.
  - The next initialization creates the next id.
- **Inactive maps:** a `FrameTracked`, `LoopDetected` (as a loop) or `GlobalBAFinished` naming an inactive or removed map is rejected or ignored, as today's old-map rules are.

- [ ] **Step 1: Failing tests** in `MapperAtlasTest`:
  - `KeepsMapsFromTenFrames` (Review Focus 5)
  - `DiscardsSmallMap`
  - `NewMapGetsNextId`
  - `InactiveMapIsFrozen`: no transaction ever names it after deactivation.
  - `ReplicasMirrorAtlas`: an `AtlasReplica` fed the Mapper's transactions holds the same live maps and active map.
- [ ] **Step 2: Run** `check`. Expected: failures.
- [ ] **Step 3: Implement.** This is a mechanical move of per-map members into `MapState`; then the loss handling.
- [ ] **Step 4: Run** `check_all`. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper Atlas: keep lost maps (ORB-SLAM3 min 10 Frames), one active map"`

### Task 4: Merge detection in the LoopCloser

**Files:**
- Modify:
  - `libs/slam/include/voyahslam/loop_closing/loop_closer.h`
  - `libs/slam/src/loop_closing/loop_closer.cpp`
  - `common_region.{h,cpp}`
- Test: `libs/slam/tests/loop_closing/loop_closer_test.cpp` (suite `LoopCloserMergeTest`)

**Interfaces:**
- Consumes: Task 2 `AtlasReplica`.
- Produces:
  - **Databases:** one `bow::FrameDatabase` per live map, dropped on `removed_maps`.
  - **Candidates:** `DetectNBestCandidates` returns `{loop, merge}` lists. Loop candidates come from the active map with covisibles excluded; merge candidates come from inactive maps. Each list keeps the best `num_candidates`.
  - **Pending state:** a separate `std::optional<PendingRegion> pending_merge_` with its own coincidence and not-found counters.
  - **Publishing:** a confirmed merge publishes `LoopDetected` with `query_map_id = active` and `match_map_id = N`. A merge confirmed at the same query as a loop wins, and that loop is dropped.
  - **Scale:** mono merges use a free-scale Sim3; stereo uses scale 1.

- [ ] **Step 1: Failing tests** in `LoopCloserMergeTest`. Build the replica from synthetic transactions: map 1 made inactive, map 2 active, and map 2 revisiting map 1's area.
  - `FindsMergeOnlyInInactiveMap`
  - `MergeConfirmedThreeTimes`: published only after the 3rd confirmation, with S close to the truth.
  - `LoopAndMergePendingIndependent`
  - `MergeWinsOverLoop`
  - `NeverMatchesRemovedMap`
  - `MonoMergeFreeScale`: the recovered scale is within 2 % of the true scale ratio between the two maps (≠ 1).
- [ ] **Step 2: Run** `check`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "LoopCloser: merge candidates in inactive maps (ORB-SLAM3 NewDetectCommonRegions)"`

### Task 5: The merge (Mapper) and its replica application

**Files:**
- Create: `libs/slam/src/mapping/mapper_merge.cpp`
- Modify:
  - `mapper.h`
  - `mapper.cpp` (dispatch a `LoopDetected` with different map ids to `OnMergeDetected`)
  - `atlas_replica.h` (apply `absorbed_map`)
- Test: `libs/slam/tests/mapping/mapper_test.cpp` (suite `MapperMergeTest`)

**Interfaces:**
- Consumes:
  - Task 3 `MapState`;
  - Task 4's merge `LoopDetected`;
  - the 4b functions `CorrectWindow`, `FuseLoopPoints` and `OptimizeGraph` (`mapper_loop.cpp`);
  - `optimization::BundleAdjust`.
- **Merge transaction format** (`Event::map_id` = N):
  - `absorbed_map = M`, `removed_maps = {M}`, `active_map = N`, and N's `map_epoch + 1`;
  - `updated_poses`: every Frame of M, with its N-frame pose;
  - `parent_changes`: the re-rooted tree;
  - point positions for every point of M under its new id;
  - `replaced_points` for the fusions, under new ids;
  - `added_loop_edges = {(query, match)}`.
- **Id rule, applied identically by the Mapper and by `AtlasReplica::Apply`:** M's alive points, sorted by old id ascending, get `N.next_id + k`. The replica moves M's Frames, observations and points into N under the rule, then applies the rest of the transaction, then drops M.
- **Steps:** spec §3 steps 1–8, using `merge_window` 25 and fuse radius 4. The weld uses `BundleAdjust` with the current window free and the merge window fixed.
- **Rejection:** spec §3.
- **Mapper bookkeeping:** the Mapper's own per-map state for M (recent points, culling queues, the `FrameTracked` pipeline) moves into N under the same id rule.

- [ ] **Step 1: Failing tests** in `MapperMergeTest`, with a two-map synthetic scene built through `FrameTracked`s and a final loss, then a hand-made merge `LoopDetected`:
  - `BuildsWindows`
  - `ReRootsTreeIntoSingleTree`: one root, acyclic.
  - `RenumbersPointsContiguously`
  - `FusesSeamDuplicates`
  - `WeldMovesOnlyCurrentWindow`
  - `MovesRestOfAbsorbedMap`
  - `PublishesOneMergeTransaction`: `AtlasReplica` passes its consistency check and equals the Mapper.
  - `RejectsActiveOrRemovedTarget`, `RejectsCulledFrames` and `RejectsWhileCorrecting`
  - `IgnoresFrameTrackedOfAbsorbedMap` (Review Focus 1)
  - `RejectsGlobalBaOfAbsorbedMap` (Review Focus 2)
  - `IgnoresRepeatOfAppliedMerge` (Review Focus 3)
- [ ] **Step 2: Run** `check`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Mapper: merge maps (ORB-SLAM3 MergeLocal), one transaction, replica id rule"`

### Task 6: Tracker follows a merge

**Files:**
- Modify: `libs/slam/include/voyahslam/tracking/tracker.h`, `libs/slam/src/tracking/tracker.cpp`
- Test: `libs/slam/tests/tracking/tracker_test.cpp` (suite `TrackerMergeTest`)

**Interfaces:**
- Consumes: Task 5's transaction format and id rule. Expose the rule as a helper, for example `AtlasReplica::TranslatePoint(MapId absorbed, MapPointId old) -> std::optional<MapPointId>`, applied after the replacements.
- Behaviour, per spec §4:
  - **While Tracking:** switch to N, shift `last_frame_` by its reference Frame's change (the 4b path), and translate every point reference in `last_frame_` and the local map. Untranslatable references become unmatched.
  - **While RecentlyLost or Lost:** no shift, but references are still translated.

- [ ] **Step 1: Failing tests:**
  - `TrackerMergeTest.TracksAcrossMerge`: the next frame is tracked in N, with no loss.
  - `TranslatesPointReferences`
  - `MergeWhileRecentlyLost` (Review Focus 4)
  - `FrameTrackedCarriesTargetMap`
- [ ] **Step 2: Run** `check`. Expected: failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `check_all`. Expected: green.
- [ ] **Step 5: Commit** `git commit -m "Tracker: follow map merges (switch, shift, translate point ids)"`

### Task 7: Integration, smoke runs and docs

**Files:**
- Modify:
  - `libs/slam/tests/integration/slam_test.cpp`
  - the apps, if they need to print the map count or merges
  - `docs/DESIGN.md`, `docs/TODO.md`, `docs/ROADMAP.md`, `README.md`

- [ ] **Step 1: Integration tests.** `SlamTest.{Mono,Stereo}MergesMaps` run the synthetic loop with a blackout long enough to lose the map before the return, over seeds 1..3. Assert:
  - the run ends with one live map holding Frames of both parts;
  - no tracking loss across the merge;
  - the whole-run error under one alignment is ≤ 1.5× the same scene without the blackout.

  Put the numbers in the commit message. A missed bar gets DONE_WITH_CONCERNS with a diagnosis; nothing is tuned.
- [ ] **Step 2: TSan** on `voyah_slam_tests` and `voyah_integration_tests`. Expected: no warnings.
- [ ] **Step 3: Smoke runs** (spec §6). Release, run one at a time, with the dataset and vocabulary paths from Task 1.
  - **Merge runs:** `--vocabulary --loop_closing --drop_frames 1500:1700`, mono and stereo, 2 runs each.
    - Must: no crash, and no tracking loss caused by a merge.
    - Report: merges found and applied, false merges (checked against ground truth), maps over time, and the final map's ATE.
  - **Memory run:** a default-flags stereo run.
    - Must: peak RSS ≤ master's 1.7 GB.
    - Report: the number.
- [ ] **Step 4: Docs** (spec §7).
  - **DESIGN.md:**
    - the Atlas and the replica rule, replacing "newest id wins" wherever it is described;
    - shared measurements;
    - merge detection and the merge;
    - the smoke table;
    - decisions-log entries dated 2026-10-10.
  - **TODO.md:** relocalization into inactive maps, the viewer's inactive maps, and separate merge edges.
  - **ROADMAP.md:** step 5 done.
- [ ] **Step 5: Run** `check_all`. Expected: green.
- [ ] **Step 6: Commit** `git commit -m "Map merging: integration, smoke runs, docs"`
