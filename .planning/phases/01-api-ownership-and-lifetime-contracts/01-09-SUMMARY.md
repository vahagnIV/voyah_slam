---
phase: 01-api-ownership-and-lifetime-contracts
plan: 09
subsystem: tracking
tags: [thread-affinity, test-access, concepts, atomics, loop-closer, global-ba]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "01-08: mapping::MapFollower / MapSnapshot as the supported cross-thread map read path"
provides:
  - "Tracker::atlas(), map_replica(), last_matches() private behind tracking::TrackerTestAccess"
  - "LoopCloser::atlas(), map_replica() and the five worker-written stats accessors private behind loop_closing::LoopCloserTestAccess"
  - "GlobalBundleAdjuster::atlas(), map_replica() private behind global_ba::GlobalBundleAdjusterTestAccess; run counters std::atomic<size_t>"
  - "libs/slam/tests/test_access.h (one definition per binary) and ThreadAffinityTest compile-time concept checks"
affects: [01-10, 01-11, voyah_jni]

actuals:
  tokens: 20000      # chars/4 over git diff plan_head_before..plan_head_after (79972 chars)
  tasks: 2
  commits: 2
plan_head_before: e0f440e0c4c8350a60bcf70cd89f17b5649c40a1
plan_head_after: bbc6c978f9fe4aa2c1a20c08314bab58c67c4aa6
commits: 2

tech-stack:
  added: []
  patterns:
    - "Private accessor + friend struct <X>TestAccess defined in tests/test_access.h; static template functions take the component by const reference"
    - "C++20 concept + static_assert(!Concept<T>) proves an accessor is not callable from outside the class"
    - "Worker-written plain counters become std::atomic<size_t> with acquire loads / acq_rel fetch_add"

key-files:
  created:
    - libs/slam/tests/test_access.h
    - libs/slam/tests/thread_affinity_test.cpp
  modified:
    - libs/slam/include/voyahslam/tracking/tracker.h
    - libs/slam/include/voyahslam/loop_closing/loop_closer.h
    - libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h
    - libs/slam/src/global_ba/global_bundle_adjuster.cpp
    - libs/slam/tests/CMakeLists.txt
    - libs/slam/tests/tracking/tracker_test.cpp
    - libs/slam/tests/tracking/tracker_test_fixture.h
    - libs/slam/tests/tracking/relocalization_test.cpp
    - libs/slam/tests/loop_closing/loop_closer_test.cpp
    - libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp
    - libs/slam/tests/integration/slam_test.cpp

key-decisions:
  - "TrackerTestAccess / LoopCloserTestAccess / GlobalBundleAdjusterTestAccess live in tests/test_access.h; MapperTestAccess stays in mapper_test.cpp until plan 01-10 moves it"
  - "Static accessors that return a class template or type named like the static (MapReplica, QueryStats) spell the return type qualified (tracking::MapReplica<FE>, loop_closing::QueryStats); the QueryStats accessor is named Queries"
  - "No LoopCloser counter becomes atomic: the five stats accessors go private (D-26), the Stop() summary log already reports the query figures"

requirements-completed: [OWN-03]

coverage:
  - id: D1
    description: "Tracker, LoopCloser and GlobalBA thread-affine accessors are not callable from outside the class; atomic counters and state() stay public"
    requirement: OWN-03
    verification:
      - kind: unit
        ref: "libs/slam/tests/thread_affinity_test.cpp#ThreadAffinityTest.RestrictedAccessorsAreNotPublic (static_assert(!Concept) checks compile only while the accessors are private)"
        status: pass
    human_judgment: false
  - id: D2
    description: "GlobalBA run counters are std::atomic<size_t>, safe to read from any thread"
    requirement: OWN-03
    verification:
      - kind: unit
        ref: "libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp (all GlobalBundleAdjusterTest cases, check_all)"
        status: pass
    human_judgment: false
  - id: D3
    description: "Migrated tests keep every EXPECT_/ASSERT_ macro: per-file assertion multiset equals the branch base; whole suite green"
    requirement: OWN-03
    verification:
      - kind: other
        ref: "assertion-preservation command on tracker_test.cpp, tracker_test_fixture.h, relocalization_test.cpp, loop_closer_test.cpp, global_bundle_adjuster_test.cpp, slam_test.cpp (excl. MapFollowerMirrorsMapper): no 'assertions changed'"
        status: pass
      - kind: integration
        ref: "cmake --build cmake-build-debug/build/Debug --target check_all: 802 tests, 0 failed (1 skipped, VocabularyEquivalenceTest, pre-existing)"
        status: pass
    human_judgment: false

duration: ~15 min (builds dominated)
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 9: Thread-affine accessors private (Tracker, LoopCloser, GlobalBA) Summary

**Tracker, LoopCloser and GlobalBA expose no worker-written state to app threads: replicas and the LoopCloser stats are private behind friend `*TestAccess` structs in `tests/test_access.h`, GlobalBA's run counters are atomic, and `ThreadAffinityTest` fails the build if an accessor becomes public again.**

## Performance

- **Duration:** about 15 min; two Debug rebuilds and `check_all` dominated
- **Completed:** 2026-10-14
- **Tasks:** 2
- **Files modified:** 13 (2 created, 11 modified)

## Accomplishments
- `Tracker::atlas()`, `map_replica()`, `last_matches()` moved below `private:` with `friend struct TrackerTestAccess`; class comment points apps at `mapping::MapFollower`.
- `LoopCloser::atlas()`, `map_replica()`, `num_coincidences()`, `num_merge_coincidences()`, `last_stats()`, `last_merge_stats()`, `query_stats()` private behind `LoopCloserTestAccess` (statics `Atlas`, `MapReplica`, `NumCoincidences`, `NumMergeCoincidences`, `LastStats`, `LastMergeStats`, `Queries`).
- `GlobalBundleAdjuster::atlas()` / `map_replica()` private behind `GlobalBundleAdjusterTestAccess`; `num_runs_started_` / `num_runs_interrupted_` are `std::atomic<size_t>` (acq_rel `fetch_add`, acquire loads, the `Stop()` log reads through the accessors). Both class comments state the rule: every public accessor is atomic or a lifecycle/driver call.
- `ThreadAffinityTest.RestrictedAccessorsAreNotPublic` with concept checks for all 12 restricted accessors plus positive checks for `Tracker::state()`, `num_runs_started()`, `num_runs_interrupted()`.
- About 150 test call sites rewritten mechanically (macro and expected value untouched).

## Task Commits

1. **Task 1: Tracker accessors private behind TrackerTestAccess; test_access.h and ThreadAffinityTest** - `abfcccf` (feat)
2. **Task 2: LoopCloser replica and stats accessors private, GlobalBA accessors private, counters atomic** - `bbc6c97` (feat)

**Plan metadata:** the docs commit that adds this SUMMARY.

## Verification Results

| Check | Result |
|-------|--------|
| Task 1 build + `Tracker*:Relocalization*:ThreadAffinityTest.*` | 47 of 47 pass, ThreadAffinityTest line present |
| Assertion-preservation command (both tasks) | no "assertions changed" output |
| `check_all` | 802 tests, 0 failed, 1 skipped (pre-existing `VocabularyEquivalenceTest`) |
| Negative grep for stats calls outside LoopCloser / access struct / concept checks | empty |
| Full Debug build (apps, examples) | rc 0 |
| `grep -c` of query_stats / last_merge_stats / num_merge_coincidences in thread_affinity_test.cpp | 6 (>= 3) |
| voyah_jni scan for the restricted accessors | no hits |

## Decisions Made
- Access structs share one header `tests/test_access.h`, included by both test binaries (the include directory was already set for both); `MapperTestAccess` is left for 01-10.
- The static returning `MapReplica<FE>` spells its return type qualified to avoid GCC's "changes meaning" error; likewise `Queries` for `QueryStats`, as the plan anticipated.
- No TSan run: the plan does not call for one (no new cross-thread path; the only concurrency change is making two counters atomic).

## Deviations from Plan

None - plan executed exactly as written.

**Total deviations:** 0.

## Issues Encountered
None.

## Notes for plan 01-10
- `docs/DESIGN.md` still lists `replica()`, `atlas()` and the LoopCloser stats accessors as public in the component table (lines 132-133) and has no entry for this plan's decision; 01-10 is the plan that records the D-26 reading and the table, so it was not touched here.
- `SlamTest.MapFollowerMirrorsMapper` reads `slam.mapper().map_ids()`, `active_map()` and `num_keyframes(id)` (see 01-08 notes); those need the `MapperTestAccess` route when the Mapper accessors go private.

## Known Stubs
None.

## Threat Flags
None. T-01-17 is mitigated as planned: accessors private plus friend access structs, atomic GlobalBA counters, concept checks that fail the build, and a negative grep for stats calls outside the access struct.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
Plan 01-10 can make the Mapper's accessors private with `MapperTestAccess` moved into `test_access.h` and record D-26 in DESIGN.

## Self-Check: PASSED

Created files exist (`test_access.h`, `thread_affinity_test.cpp`); commits `abfcccf` and `bbc6c97` are ancestors of HEAD on `refactor/thread-affinity`.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-14*
