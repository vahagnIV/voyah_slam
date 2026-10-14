---
phase: 01-api-ownership-and-lifetime-contracts
plan: 08
subsystem: mapping
tags: [map-follower, map-snapshot, thread-affinity, atlas-replica, tsan, examples]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "R1 (0.4.0): shared_ptr<const T> events, RAII Subscription, inbox core, AtlasReplica fed by MapTransactions"
provides:
  - "mapping::MapFollower<FE>: an inbox plus its own AtlasReplica, updated by Update() on the caller's thread"
  - "mapping::MapSnapshot: immutable plain-data copy of the Atlas (shared_ptr<const>), readable on any thread"
  - "full_slam::PrintAtlas(const MapSnapshot&, num_merges, out) and both example apps reading the map through a follower"
  - "branch refactor/thread-affinity created from master (R2 work, D-21)"
affects: [01-09, 01-10, 01-11, voyah_jni]

actuals:
  tokens: 7900       # chars/4 over git diff plan_head_before..plan_head_after (31467 chars)
  tasks: 2
  commits: 2
plan_head_before: 3b8249790f3807ba5985c9c613137d506d0f48b0
plan_head_after: fd43a13d76276ae2ac175c9007fbafee54e5c87a
commits: 2

tech-stack:
  added: []
  patterns:
    - "Supported cross-thread map read: private replica per reader, Update() on the reader's own thread, immutable shared snapshot out"
    - "Reader inbox subscribed before the producer starts (D-05); follower declared before the Subscription holder"

key-files:
  created:
    - libs/slam/include/voyahslam/mapping/map_snapshot.h
    - libs/slam/include/voyahslam/mapping/map_follower.h
    - libs/slam/tests/mapping/map_follower_test.cpp
  modified:
    - libs/slam/tests/CMakeLists.txt
    - libs/slam/tests/integration/slam_test.cpp
    - examples/full_slam/full_slam_common.h
    - examples/full_slam/monocular_full_slam.cpp
    - examples/full_slam/binocular_full_slam.cpp

key-decisions:
  - "MapFollower asserts (debug) that Update(), atlas() and Snapshot() run on the thread of the first Update(); Snapshot() before any Update() is unrestricted"
  - "Update() counts only transactions the AtlasReplica admitted (Apply returned true); ignored ones (unknown or removed map) are not counted"
  - "SlamTest.MapFollowerMirrorsMapper reuses RunSequence: RunSetup gained an optional follower pointer, RunCounts captures the Mapper's map_ids / active_map / num_keyframes(id) after Stop"

requirements-completed: [OWN-03]

coverage:
  - id: D1
    description: "MapFollower applies pending MapTransactions only on Update(), releases other event types, counts what it applied"
    requirement: OWN-03
    verification:
      - kind: unit
        ref: "libs/slam/tests/mapping/map_follower_test.cpp#MapFollowerTest.UpdateWithNothingPendingReturnsZero, AppliesTransactionsOnlyOnUpdate, IgnoresOtherEventTypes"
        status: pass
    human_judgment: false
  - id: D2
    description: "MapSnapshot is immutable plain data that outlives the follower, is correct, drops removed maps and can be read on another thread (also TSan-clean)"
    requirement: OWN-03
    verification:
      - kind: unit
        ref: "libs/slam/tests/mapping/map_follower_test.cpp#MapFollowerTest.SnapshotOfEmptyFollowerHasNoMaps, SnapshotHoldsTheMapsContent, SnapshotIsImmutableAndOutlivesFollower, SnapshotReadOnAnotherThread, RemovedMapLeavesTheSnapshot"
        status: pass
      - kind: unit
        ref: "setarch -R cmake-build-tsan/libs/slam/tests/voyah_slam_tests --gtest_filter='MapFollowerTest.*' (0 ThreadSanitizer warnings)"
        status: pass
    human_judgment: false
  - id: D3
    description: "After a real stereo Slam run, Stop() and Update(), the follower's live maps, active map and per-map Frame counts equal the Mapper's"
    requirement: OWN-03
    verification:
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.MapFollowerMirrorsMapper (Debug and TSan, 0 warnings)"
        status: pass
    human_judgment: false
  - id: D4
    description: "Both example apps print the Atlas from a follower snapshot and no example calls a Mapper-thread accessor"
    requirement: OWN-03
    verification:
      - kind: other
        ref: "monocular_full_slam / binocular_full_slam on corridor1 (--rate=0 --max-frames=40) exit 0; git grep -e 'mapper.map_ids()' -e 'mapper.active_map()' -- examples is empty"
        status: pass
    human_judgment: false

duration: ~40 min active (TSan rebuild dominated; the wall clock jumped during it)
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 8: MapFollower and MapSnapshot Summary

**`mapping::MapFollower` (own inbox and AtlasReplica, `Update()` on the caller's thread) with immutable `MapSnapshot` is the supported map read path, mirrors a real stereo Mapper run, is TSan-clean, and both example apps now print the Atlas from a snapshot.**

## Performance

- **Duration:** about 40 min of work; the TSan rebuild dominated
- **Completed:** 2026-10-14
- **Tasks:** 2
- **Files modified:** 8 (3 created, 5 modified)

## Accomplishments
- `MapSnapshot` plain data (per live map: id, epoch, alive point ids and world positions, Frame ids and T_cw; active map; transactions applied; `Find()`), published as `std::shared_ptr<const MapSnapshot>`.
- `MapFollower<FE>` header-only: `inbox()`, `Update()`, `atlas()`, `Snapshot()`, `num_transactions_applied()`, with the one-thread rule asserted in debug builds and the cost and wiring rules (subscribe before Start, D-05) in the class comment.
- 8 `MapFollowerTest` cases (the 7 planned plus `SnapshotHoldsTheMapsContent`) and `SlamTest.MapFollowerMirrorsMapper` on a real stereo Slam run.
- `PrintAtlas` takes a snapshot and the any-thread `Mapper::num_merges()`; both apps wire a follower before `Start()` and update it after `Stop()` (D-26).
- Branch `refactor/thread-affinity` created from master (R1 0.4.0), as D-21 requires.

## Task Commits

1. **Task 1: Tracer, Mapper to MapFollower to Snapshot on a real Slam run** - `5079f94` (feat)
2. **Task 2: Examples print the Atlas from a MapFollower snapshot; TSan on the follower** - `fd43a13` (feat)

**Plan metadata:** the docs commit that adds this SUMMARY.

## Verification Results

| Check | Result |
|-------|--------|
| Debug build of `voyah_slam_tests`, `voyah_integration_tests` | pass |
| `MapFollowerTest.*` (Debug) | 8 of 8 pass |
| `SlamTest.MapFollowerMirrorsMapper` (Debug) | pass, 1.0 s |
| Tracer gate (re-run of Task 1 `<verify>` before Task 2) | passed, expansion continued |
| `monocular_full_slam`, `binocular_full_slam` build | pass |
| `git grep -e 'mapper.map_ids()' -e 'mapper.active_map()' -- examples` | empty |
| Apps on corridor1 `--rate=0 --max-frames=40` | both rc 0, no "terminate". Bino: `atlas: map 1 (1 Frames, active); merges: 0`. Mono: `atlas:; merges: 0` (still WaitingForMap at 40 frames, as in 01-07) |
| TSan `MapFollowerTest.*` (`setarch -R`) | 8 pass, 0 ThreadSanitizer warnings |
| TSan `SlamTest.MapFollower*` | 1 pass (11.3 s), 0 ThreadSanitizer warnings |

`check` / `check_all` were not part of this plan's verification and were not run; the plan's own gates passed. Logs are in the session scratchpad (`tsan_mf.log`, `tsan_sl.log`, `binocular_08.log`, `monocular_08.log`).

## Decisions Made
- Debug thread-affinity assert on the follower: the owner is the thread of the first `Update()`; calls before that are unrestricted, so a snapshot of a never-updated follower is allowed anywhere.
- `Update()` returns and counts only admitted transactions, so `num_transactions_applied()` matches what the replica really holds.
- The Slam integration test extends `RunSequence` (optional `RunSetup::follower`, Mapper atlas fields in `RunCounts`) instead of duplicating the run harness.

## Deviations from Plan

None - plan executed exactly as written. Two small additions inside the plan's intent: an extra test (`SnapshotHoldsTheMapsContent`, 8 cases instead of 7) and the follower is also passed through `ProtocolRecorder::Attach` so its Subscription lives with the recorder's.

**Total deviations:** 0.

## Issues Encountered
- The first Debug build rebuilt `voyah_slam` from scratch (stale build tree); no effect on results.
- The TSan rebuild showed a wall-clock jump (about 1069 min real vs 5 min user); results unaffected.

## Notes for plan 01-10
`SlamTest.MapFollowerMirrorsMapper` reads `slam.mapper().map_ids()`, `active_map()` and `num_keyframes(id)` inside `RunSequence` after `Stop()`. These accessors become private in 01-10, so that block needs the test-access route there (as that plan already anticipates). The assertion multiset of this test is what 01-10 must keep.

## Known Stubs
None.

## Threat Flags
None. T-01-15 (cross-thread map reads) is mitigated as planned: follower-owned replica, immutable snapshot copy, TSan clean. T-01-16 accepted; the inbox is named "MapFollower" for the backlog warning and the class comment documents it.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
Plan 01-09 and 01-10 can make the Mapper-thread accessors private: the examples no longer use them, and the supported read path exists and is tested. The next plan on this branch is 01-09.

## Self-Check: PASSED

Created files exist (`map_snapshot.h`, `map_follower.h`, `map_follower_test.cpp`); commits `5079f94` and `fd43a13` are ancestors of HEAD on `refactor/thread-affinity`.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-14*
