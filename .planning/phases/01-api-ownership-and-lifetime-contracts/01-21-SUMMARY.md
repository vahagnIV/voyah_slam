---
phase: 01-api-ownership-and-lifetime-contracts
plan: 21
subsystem: lifecycle-and-thread-affinity
tags: [event-node, lifecycle, deadlock, thread-affinity, test-access, map-follower, c++20, tsan]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "lifecycle state machine (01-12, 01-14), TestAccess pattern (01-08), MapFollower (01-08)"
provides:
  - "EventNode::StopWith(stop_worker, on_worker_thread): own-thread Stop() refused before the state is read, in every state"
  - "ProcessPending() private in Tracker, Mapper, LoopCloser, GlobalBundleAdjuster; *TestAccess::ProcessPending wrappers; HasProcessPending compile-time guards"
  - "Example apps drain the MapFollower every frame and print the pending-at-Stop count"
affects: [01-22 version bump 0.9.2 and test counts, 01-23 release gates and merge, voyah_jni]

actuals:
  tokens: 16000
  tasks: 3
  commits: 5

tech-stack:
  added: []
  patterns:
    - "Own-thread predicate as a required argument of StopWith (no overload without it)"
    - "Would-be-deadlock tests run the blocking call on a helper thread under a deadline and std::_Exit(1) with a DEADLOCK line"
    - "Compile-time contract (static_assert on a concept) committed together with the change that satisfies it"

key-files:
  created: []
  modified:
    - libs/events/include/voyahslam/events/event_node.h
    - libs/events/tests/events/lifecycle_test.cpp
    - libs/events/tests/events/probe_event.h
    - libs/slam/src/tracking/tracker.cpp
    - libs/slam/src/mapping/mapper.cpp
    - libs/slam/src/loop_closing/loop_closer.cpp
    - libs/slam/src/global_ba/global_bundle_adjuster.cpp
    - libs/slam/include/voyahslam/tracking/tracker.h
    - libs/slam/include/voyahslam/mapping/mapper.h
    - libs/slam/include/voyahslam/loop_closing/loop_closer.h
    - libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h
    - libs/slam/tests/test_access.h
    - libs/slam/tests/thread_affinity_test.cpp
    - libs/slam/tests/integration/slam_test.cpp
    - examples/full_slam/binocular_full_slam.cpp
    - examples/full_slam/monocular_full_slam.cpp
    - docs/DESIGN.md
    - docs/TODO.md
    - .planning/phases/01-api-ownership-and-lifetime-contracts/01-SECURITY.md

key-decisions:
  - "StopWith checks on_worker_thread() before the state switch and has no one-argument overload"
  - "ProcessPending() made private behind the TestAccess friends, not a run-time UsageError (compile-time like D-14; a run-time check would be check-then-act against Start())"
  - "Examples call follower.Update() every frame (amends the D-26 example detail, user decision 2026-10-21)"

patterns-established:
  - "Deadlock regression tests fail fast through a watchdog (std::_Exit(1), DEADLOCK on stderr)"

requirements-completed: [OWN-03, OWN-04]

coverage:
  - id: D1
    description: "Stop() on a component's own worker thread throws UsageError in every RunState, also while another thread is in Stop() or Start(); the other thread's call completes and the component ends Stopped"
    requirement: OWN-04
    verification:
      - kind: unit
        ref: "libs/events/tests/events/lifecycle_test.cpp#LifecycleTest.StopFromWorkerThreadWhileAnotherThreadStopsThrows"
        status: pass
      - kind: unit
        ref: "libs/events/tests/events/lifecycle_test.cpp#LifecycleTest.StopFromWorkerThreadWhileStartingThrows"
        status: pass
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.LifecycleComponentStopOnItsThreadWhileSlamStopsThrows"
        status: pass
    human_judgment: false
  - id: D2
    description: "ProcessPending() of the four components is private; the build fails if one becomes public again"
    requirement: OWN-03
    verification:
      - kind: unit
        ref: "libs/slam/tests/thread_affinity_test.cpp#HasProcessPending static_asserts (ThreadAffinityTest.RestrictedAccessorsAreNotPublic)"
        status: pass
      - kind: other
        ref: "cmake --build cmake-build-debug/build/Debug --target check_all (858 tests passed)"
        status: pass
    human_judgment: false
  - id: D3
    description: "Both example apps update the MapFollower every frame; binocular corridor1 reports 0 of 5 transactions applied after Stop() with the 39 / 3 Frames / 777 points result unchanged"
    verification:
      - kind: other
        ref: "binocular_full_slam corridor1 --rate=0 --max-frames=40 (map follower: 0 of 5; 39 FrameTracked; map: 3 Frames, 777 points)"
        status: pass
    human_judgment: false
  - id: D4
    description: "DESIGN, TODO and SECURITY describe the 0.9.2 lifecycle and thread-affinity changes"
    verification: []
    human_judgment: true
    rationale: "Documentation wording is judged by reading; the verify greps only prove the sections exist"

duration: 35min
completed: 2026-10-21
status: complete
plan_head_before: 643cc0e682dc3ee5177fc29870e0d7bc1911b862
plan_head_after: 8e717e925c7e6cd4a9e2f7968ce15bd7fbe8c4c8
commits: 5
---

# Phase 1 Plan 21: Own-thread Stop in every state, private ProcessPending, follower drained in the examples Summary

**`EventNode::StopWith(stop_worker, on_worker_thread)` refuses a component's own-thread `Stop()` before it reads the state (no more Starting/Stopping deadlock), `ProcessPending()` of the four components is private behind `*TestAccess` with compile-time guards, and both example apps drain the `MapFollower` every frame.**

## Performance

- **Duration:** about 35 min
- **Completed:** 2026-10-21
- **Tasks:** 3 (Task 1 tracer, TDD: RED and GREEN commits)
- **Files modified:** 25 (322 insertions, 83 deletions)
- **Branch:** `fix/phase1-gaps` (created from master at the plan's start; nothing merged or pushed)

## Accomplishments

- Verification gap 1 (WR-01, OWN-04) closed: the own-thread check runs first in `StopWith`, the one-argument form is deleted, all four components pass `worker_.OnWorkerThread()`. A refused call changes nothing (`run_state()`, `GlobalBundleAdjuster::stop_requested()` untouched).
- Verification gap 2 (IN-07, OWN-03) closed: `ProcessPending()` moved into the private sections (after `friend struct *TestAccess`), four `*TestAccess::ProcessPending(out_x)` wrappers, 16 test call sites rewritten, `HasProcessPending` concept plus four `static_assert(!...)`.
- WR-06: both apps call `follower.Update()` after the trajectory poll in the frame loop and once more after `Stop()`, printing `map follower: k of N transactions applied after Stop()`.
- DESIGN §4 / §7 / §8 (three 2026-10-21 rows), TODO (one ticked, two open items) and 01-SECURITY.md T-01-21 updated.

## TDD record

**RED** (`db9b901`, log `$LOGS/01-21-red.log` in the executor scratchpad; each test run alone, library untouched):

- `LifecycleTest.StopFromWorkerThreadWhileAnotherThreadStopsThrows`: `DEADLOCK: Stop() did not return within 10 s`, exit 1 (target test hung on the planned behavior, watchdog fired). Semantic assessment: the worker parked until `Stopping`, called `Stop()`, waited in `WaitWhileState(Stopping)` while the other thread joined it; exactly the planned failure.
- `LifecycleTest.StopFromWorkerThreadWhileStartingThrows`: `lifecycle_test.cpp:282: Failure  Value of: node.caught_while_starting()  Actual: false  Expected: true`, `[  FAILED  ]` after 3001 ms (the worker blocked on `Starting` until the start lambda gave up). Planned failure.
- `SlamTest.LifecycleComponentStopOnItsThreadWhileSlamStopsThrows`: `DEADLOCK: Slam::Stop() did not return within 120 s`, exit 1. Planned failure.

**GREEN** (`b140214`): the three tests pass in Debug; `LifecycleTest.*` 14 of 14 and `SlamTest.Lifecycle*` 9 of 9 pass; under ThreadSanitizer `LifecycleTest.*` x50 (50 times "14 tests passed", 0 DEADLOCK / ThreadSanitizer / FAILED lines) and `SlamTest.Lifecycle*` (0 warnings).

**REFACTOR:** none needed.

Task 2 RED (compile-time, not committed, per plan): `thread_affinity_test.cpp` failed to build with four messages: `Tracker::ProcessPending() is driven by the worker thread; tests use TrackerTestAccess`, `Mapper::ProcessPending() ... MapperTestAccess`, `LoopCloser::ProcessPending() ... LoopCloserTestAccess`, `GlobalBundleAdjuster::ProcessPending() ... GlobalBundleAdjusterTestAccess`. The asserts were committed together with the GREEN change.

## Task Commits

1. **Task 1 RED:** `db9b901` (test) failing tests for own-thread Stop while another thread stops or starts
2. **Task 1 GREEN:** `b140214` (fix) Stop() on a component's own worker throws in every state (WR-01)
3. **Task 2:** `c9eec67` (refactor) ProcessPending private behind the TestAccess structs (IN-07)
4. **Task 3 code:** `bdc5eee` (fix) example apps update the MapFollower every frame (WR-06)
5. **Task 3 docs:** `8e717e9` (docs) own-thread Stop, private ProcessPending, follower in the examples

**Plan metadata:** the docs commit "complete plan" that carries this file (after the five above).

## Example-app measurements (WR-06, corridor1, `--rate=0 --max-frames=40`, Debug tree)

| Run | `map follower:` line | `[Mapper] [debug] FrameTracked` | `map:` line | mean load / extract |
|---|---|---|---|---|
| binocular before (update only after Stop) | `5 of 5 transactions applied after Stop()` | 39 | `3 Frames, 777 points` | 25.6 / 216.3 ms |
| binocular after (update every frame) | `0 of 5 transactions applied after Stop()` | 39 | `3 Frames, 777 points` | 21.7 / 220.5 ms |
| monocular after | `0 of 0 transactions applied after Stop()` | n/a | `0 Frames, 0 points`, tracker state `WaitingForMap` (known at 40 frames) | n/a |

The bar (at least 36 FrameTracked and at least 3 Frames; baseline 39 / 3 / 777) is met and unchanged. Neither app printed `terminate called`; both exited 0.

## Verification results

- `cmake --build cmake-build-debug/build/Debug --target check_all`: 100% passed, 0 failed of 858 (817 `unit`, 40 `integration`, 1 `vocabulary` skipped without ORBvoc). Start was 815 unit / 855 total; the new tests add 2 unit and 1 integration. The full Debug tree (examples, tools, benchmarks) builds without errors.
- `awk` private check: all four headers print `private`. `git grep '\.ProcessPending()'` over tests (excluding `test_access.h`), examples: only the concept in `thread_affinity_test.cpp`.
- Task 3 verify commands: the binocular check (FrameTracked >= 36, `map: 3 Frames`, `0 < 5`, no terminate) and the monocular check (`tracker state:` and `map follower:` lines) pass; the DESIGN / SECURITY / TODO greps pass.
- Known flaky tests (DuplicatesFusedInLongSequence, StereoMapGrowsWhileLeavingInitialView, MonoRelocalizesAfterBlackout, StereoRelocalizesAfterBlackout) did not fail in the `check_all` run.

## Deviations from Plan

None - plan executed exactly as written. Two small additions inside the plan's scope: the Stop() comments of the four headers gained the "in every state" sentence the plan asked for, and `worker_thread.h` was left unchanged.

## Issues Encountered

None. The RED run of the Slam-level test takes the full 120 s watchdog (and the first LifecycleTest RED 10 s), as designed.

## Known Stubs

None.

## Threat Flags

None. T-01-35 (own-thread Stop deadlock), T-01-36 (public ProcessPending) and T-01-37 (undrained follower in the examples) are mitigated as planned; T-01-38 (two callbacks stopping each other's component) is accepted and recorded as an open item in `docs/TODO.md` and DESIGN §4.

## Next Phase Readiness

- 01-22 does the version bump to 0.9.2 and re-measures the test counts (CLAUDE.md still says 815 unit / 855 total; now 817 / 858).
- 01-23 runs the release gates and the merge; nothing was merged or pushed here.
- The voyah_jni binding should hear that `ProcessPending()` is no longer public (its API doc already called it tests-only) and that an own-thread `Stop()` now throws in every state.

## Self-Check: PASSED

Files verified present on disk: the 19 modified files above (git status clean for tracked files). Commits verified in the branch history: `db9b901`, `b140214`, `c9eec67`, `bdc5eee`, `8e717e9` (5 commits from `plan_head_before` 643cc0e to `plan_head_after` 8e717e9).

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-21*
