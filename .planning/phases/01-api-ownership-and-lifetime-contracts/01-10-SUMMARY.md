---
phase: 01-api-ownership-and-lifetime-contracts
plan: 10
subsystem: mapping
tags: [thread-affinity, test-access, concepts, docs, version-0.5.0, map-follower]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "01-08: MapFollower / MapSnapshot; 01-09: Tracker, LoopCloser, GlobalBA accessors private, tests/test_access.h"
provides:
  - "Mapper's twelve Mapper-thread accessors private behind mapping::MapperTestAccess (now in tests/test_access.h)"
  - "ThreadAffinityTest concept checks for all twelve plus a positive check for num_keyframes()"
  - "DESIGN Map Follower subsection, updated component table, decisions-log rows 'Ownership audit #5 implemented' and 'Package version 0.5.0'"
  - "Package version 0.5.0 (CMakeLists.txt, conanfile.py, README.md, DESIGN layout/SOVERSION lines)"
affects: [01-11, voyah_jni]

actuals:
  tokens: 21100      # chars/4 over git diff plan_head_before..plan_head_after (84506 chars)
  tasks: 2
  commits: 2
plan_head_before: d887ceae16c909de3547c83cb57ca57c5214338f
plan_head_after: 6c8e4ddadee092e7fb7d1d0069edd9783f568b5e
commits: 2

tech-stack:
  added: []
  patterns:
    - "One friend struct per component in tests/test_access.h; one static per restricted accessor named in PascalCase after it; overloads on strong-id argument types (NumVisible(mapper, MapPointId) vs NumVisible(mapper, MapId))"
    - "Call sites converted by a compiler-driven rewrite: build with -k, take the file:line:col of each 'private within this context' error (the column is the call's open paren), rewrite recv.Name(args) to MapperTestAccess::Name(recv, args)"

key-files:
  created: []
  modified:
    - libs/slam/include/voyahslam/mapping/mapper.h
    - libs/slam/tests/test_access.h
    - libs/slam/tests/thread_affinity_test.cpp
    - libs/slam/tests/mapping/mapper_test.cpp
    - libs/slam/tests/integration/slam_test.cpp
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CLAUDE.md
    - CMakeLists.txt
    - conanfile.py

key-decisions:
  - "MapperTestAccess moved to tests/test_access.h unchanged for its existing helpers; the new statics take the Mapper by const reference and mirror the accessor names"
  - "Only mapper_test.cpp and slam_test.cpp used the restricted Mapper accessors; global_bundle_adjuster_test, loop_closer_test and tracker_test needed no change (their .covisibility() / .map_ids() hits belong to replicas)"
  - "TODO keeps one open follow-up item for the deferred Session facade and the viewer on a follower, since the old #5 text carried them"

requirements-completed: [OWN-03]

coverage:
  - id: D1
    description: "The twelve Mapper-thread accessors are not callable from outside the Mapper and its test access; the no-argument num_keyframes() stays callable"
    requirement: OWN-03
    verification:
      - kind: unit
        ref: "libs/slam/tests/thread_affinity_test.cpp#ThreadAffinityTest.RestrictedAccessorsAreNotPublic (static_assert(!Concept<TMapper>) x12 and static_assert(HasNumKeyframes<TMapper>))"
        status: pass
    human_judgment: false
  - id: D2
    description: "Migrated Mapper tests keep every EXPECT_/ASSERT_ macro: per-file multiset equals the branch base; whole suite green"
    requirement: OWN-03
    verification:
      - kind: other
        ref: "assertion-preservation command on mapper_test.cpp, global_bundle_adjuster_test.cpp, loop_closer_test.cpp, tracker_test.cpp, slam_test.cpp (excl. MapFollowerMirrorsMapper): no 'assertions changed'"
        status: pass
      - kind: integration
        ref: "cmake --build cmake-build-debug/build/Debug --target check_all: 802 tests, 0 failed on the second run (1 skipped, VocabularyEquivalenceTest, pre-existing)"
        status: pass
    human_judgment: false
  - id: D3
    description: "R2 is documented (Map Follower subsection, component table, #5 and 0.5.0 decisions rows naming LoopCloserTestAccess and D-26) and versioned 0.5.0"
    requirement: OWN-03
    verification:
      - kind: other
        ref: "grep VERSION 0.5.0 CMakeLists.txt; grep version = \"0.5.0\" conanfile.py; grep 'Package version 0.5.0' docs/DESIGN.md; git grep 0.4.0 -- CMakeLists.txt conanfile.py README.md is empty; check target 771 of 771 pass"
        status: pass
    human_judgment: false

duration: ~14 min (builds dominated)
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 10: Mapper accessors private, R2 documented, version 0.5.0 Summary

**The Mapper's twelve Mapper-thread accessors are private behind a shared `MapperTestAccess` in `tests/test_access.h`, `ThreadAffinityTest` fails the build if one is made public again, DESIGN records the whole R2 decision (MapFollower, the TestAccess restriction, atomics, the D-26 reading), and the package is 0.5.0.**

## Performance

- **Duration:** about 14 min; one full Debug rebuild per iteration and two `check_all` runs dominated
- **Completed:** 2026-10-14
- **Tasks:** 2
- **Files modified:** 12 (0 created, 12 modified)

## Accomplishments
- `Mapper::covisibility()`, `Reference()`, `NumVisible()`, `NumFound()`, `last_local_ba()`, `BowVectorOf()`, `Parent()`, `LoopEdges()`, `KeyFrame()`, `map_ids()`, `active_map()` and `num_keyframes(MapId)` moved below `private:` next to `friend struct MapperTestAccess`; comments say "Mapper-thread state; tests read it through MapperTestAccess; apps use mapping::MapFollower". `num_keyframes()` with no argument, `num_map_points()` and every other atomic accessor stay public.
- `MapperTestAccess` moved from `mapper_test.cpp` to `tests/test_access.h` with its helpers unchanged, plus statics `Covisibility`, `Reference`, `NumVisible(mapper, MapPointId)`, `NumFound(mapper, MapPointId)`, `LastLocalBa`, `BowVectorOf`, `Parent`, `LoopEdges`, `KeyFrame`, `MapIds`, `ActiveMap`, `NumKeyframes(mapper, MapId)`.
- 104 Mapper call sites rewritten (102 in `mapper_test.cpp`, the rest in `SlamTest.MapFollowerMirrorsMapper`'s block of `slam_test.cpp`); no assertion touched.
- `ThreadAffinityTest` gained twelve `static_assert(!Concept<TMapper>)` checks and `static_assert(HasNumKeyframes<TMapper>)`.
- DESIGN: new "Map Follower (supported map read path)" subsection; component-table rows for Tracker, Mapper, LoopCloser (five stats accessors test-only through `LoopCloserTestAccess`) and GlobalBA (atomic run counters); the Mapper bullet and the covisibility sentence match; decisions-log rows "Ownership audit #5 implemented" (D-13, D-14, D-26 reading) and "Package version 0.5.0"; layout and SOVERSION 0.5; status lines.
- TODO ticks #5 with a pointer, adds one open item for the deferred `voyahslam::Session` facade and the viewer on a follower; ROADMAP records #5 done, next #6; README, `CMakeLists.txt`, `conanfile.py` at 0.5.0; CLAUDE.md counts re-measured with `ctest -N` (771 unit, 802 total).

## Task Commits

1. **Task 1: Mapper accessors private behind the shared MapperTestAccess; migrate call sites** - `b69d573` (feat)
2. **Task 2: Document R2 and bump to 0.5.0** - `6c8e4dd` (docs)

**Plan metadata:** the docs commit that adds this SUMMARY.

## Verification Results

| Check | Result |
|-------|--------|
| Debug build of all targets (tests, apps, examples) after the accessors went private | rc 0 |
| Assertion-preservation command (mapper_test, global_bundle_adjuster_test, loop_closer_test, tracker_test, slam_test excl. MapFollowerMirrorsMapper) | no "assertions changed" output |
| `check_all` | 802 tests; first run 1 failure (`SlamTest.DuplicatesFusedInLongSequence`), second run 0 failed, 1 skipped (pre-existing `VocabularyEquivalenceTest`) |
| `SlamTest.DuplicatesFusedInLongSequence` alone, three runs | all pass (1.0-1.3 s) |
| `check` after the docs and version change | 771 of 771 pass |
| `grep 'VERSION 0.5.0' CMakeLists.txt`, `grep 'version = "0.5.0"' conanfile.py`, `Package version 0.5.0` in DESIGN | present |
| `git grep '0\.4\.0' -- CMakeLists.txt conanfile.py README.md` | empty |
| `grep -n 'struct MapperTestAccess {' mapper_test.cpp` / in test_access.h | none / one line |

## Decisions Made
- Only `mapper_test.cpp` and `slam_test.cpp` needed call-site changes; the plan's other listed test files only call the same-named accessors on replicas.
- The call-site rewrite was compiler-driven (build with `-k`, rewrite at each reported column, rebuild), then the six lines that passed 120 columns were wrapped by hand (clang-format is not installed here).
- One open TODO item is added for the deferred Session facade and viewer-on-follower, per the repository rule to record every deferral; no item defers the LoopCloser stats (D-21, D-26).

## Deviations from Plan

None - plan executed exactly as written.

**Total deviations:** 0.

## Issues Encountered
- `SlamTest.DuplicatesFusedInLongSequence` failed once in the first `check_all` run and passed in three isolated runs and in the second `check_all`. The test is timing dependent (live threads, run next to the other integration tests); this plan changes only access to accessors it does not call, so it was not investigated further. Worth watching in 01-11's gates.

## Known Stubs
None.

## Threat Flags
None. T-01-18 is mitigated as planned: twelve accessors private behind `MapperTestAccess`, `ThreadAffinityTest` concept checks, apps use `MapFollower`.

## User Setup Required
None - no external service configuration required.

## Next Phase Readiness
Plan 01-11 can run the R2 release gates for 0.5.0 (check_all, apps, TSan, ASan, `conan create`). Note for its checks: the `git grep` for `tracker().map_replica()` and the Mapper accessors in `examples` / `vizualization` should come back empty; the build already proves no non-test code uses them.

## Self-Check: PASSED

Modified files exist (`mapper.h`, `test_access.h`, `thread_affinity_test.cpp`, `mapper_test.cpp`, `slam_test.cpp`, the docs and version files); commits `b69d573` and `6c8e4dd` are ancestors of HEAD on `refactor/thread-affinity`.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-14*
