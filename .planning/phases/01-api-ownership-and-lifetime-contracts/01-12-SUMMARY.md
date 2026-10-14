---
phase: 01-api-ownership-and-lifetime-contracts
plan: 12
subsystem: lifecycle
tags: [lifecycle, state-machine, cas, atomic-wait, usage-error, tracker, mapper, loop-closer, global-ba, slam, tsan]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "R1 RunState / Wiring::state / Observable::SetState and R2 (0.5.0) merged into master"
provides:
  - "EventNode::StartWith / StopWith: Created -> Running -> Stopped through an atomic CAS, invalid transitions throw UsageError, Stopped is terminal"
  - "Observable::CompareExchangeState / WaitWhileState; SetState notifies (std::atomic wait/notify, control plane only)"
  - "WorkerThread::Stop() throws UsageError on the worker's own thread (worker_id_); a failed thread start resets running_"
  - "Tracker, Mapper, LoopCloser, GlobalBA on the state machine; Tracker::Track() throws after Stop(), queues before Start()"
  - "Slam composite lifecycle (state_ atomic) with rollback of a failed Start() and an any-thread idempotent Stop()"
  - "EventNode::BeginRun / EndRun removed"
affects: [01-13, 01-14, voyah_jni]

actuals:
  tokens: 40000   # chars/4 over the 20 changed files (about 600 lines added, 85 removed)
  tasks: 2
  commits: 4      # 3 code commits (tracer, RED, GREEN) + the CLAUDE.md count commit; the docs commit that adds this SUMMARY is not counted
plan_head_before: 9ba2cd3c3c7d5e9d2dbe15e69578033552a05405
plan_head_after: f9512bc4fa1a2b9ddbff8a843d86f76af7699d45
commits: 4

tech-stack:
  added: []
  patterns:
    - "StartWith(start_worker) / StopWith(stop_worker) protected template helpers; the component passes a lambda that starts or joins its WorkerThread"
    - "Concurrent Stop() waits on the state atomic (Starting/Stopping) and retries, so every caller returns only after the join"
    - "A refused self-stop (UsageError from the worker thread) puts the state back to Running"

key-files:
  created:
    - libs/events/tests/events/lifecycle_test.cpp
  modified:
    - libs/events/include/voyahslam/events/event_node.h
    - libs/events/include/voyahslam/events/observable.h
    - libs/events/include/voyahslam/events/worker_thread.h
    - libs/events/src/events/worker_thread.cpp
    - libs/events/tests/events/probe_event.h
    - libs/events/tests/CMakeLists.txt
    - libs/slam/include/voyahslam/tracking/tracker.h
    - libs/slam/src/tracking/tracker.cpp
    - libs/slam/tests/tracking/tracker_test.cpp
    - libs/slam/include/voyahslam/mapping/mapper.h
    - libs/slam/src/mapping/mapper.cpp
    - libs/slam/include/voyahslam/loop_closing/loop_closer.h
    - libs/slam/src/loop_closing/loop_closer.cpp
    - libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h
    - libs/slam/src/global_ba/global_bundle_adjuster.cpp
    - libs/slam/include/voyahslam/slam/slam.h
    - libs/slam/src/slam/slam.cpp
    - libs/slam/tests/integration/slam_test.cpp
    - docs/DESIGN.md
    - CLAUDE.md

key-decisions:
  - "GlobalBA::stop_requested_ is not reset in Start(): Stopped is terminal, so it is false exactly until Stop() is called; the plan's reset inside the start lambda was dead and raced with a Stop() waiting on Starting"
  - "GlobalBA raises stop_requested_ inside the stop lambda (before the join, restored to false if the worker refuses a self-stop) and once more after StopWith for the Created -> Stopped path"
  - "A Slam::Stop() refused by a component's own thread restores the Slam state and rethrows; components already stopped stay stopped, and a later Stop() from another thread finishes the job"

requirements-completed: [OWN-04]

duration: "about 1h 20m"
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 12: Lifecycle state machine Summary

**Every component and `Slam` now follow Created -> Running -> Stopped through an atomic CAS in `EventNode::StartWith` / `StopWith`: invalid transitions throw `UsageError`, Stopped is terminal, Start and Stop are safe from any thread, concurrent Stops return only after the join, and a Stop() on a component's own worker thread throws instead of self-joining.**

## Task 1 (tracer): CAS lifecycle in EventNode, end to end on the Tracker

Commit `cd3c1a6` (branch `refactor/lifecycle`, created from master 9ba2cd3).

- `EventNode::StartWith`: CAS Created to Starting, run the worker start, store Running; a failed worker start stores Stopped and rethrows; Start twice / after Stop throws `UsageError` with the contract's messages.
- `EventNode::StopWith`: Created to Stopped; Running to Stopping, stop lambda, Stopped; Starting/Stopping wait (`std::atomic::wait`) and retry; Stopped returns; a throwing stop lambda restores Running.
- `Observable`: `CompareExchangeState`, `WaitWhileState`, `SetState` now `notify_all`s.
- `WorkerThread`: `worker_id_` (`std::atomic<std::thread::id>`), stored by the thread first and cleared on exit; `Stop()` throws on the worker thread. `RestartAfterStop` still passes (the primitive stays restartable, D-26).
- Tracker: `Start()` / `Stop()` through the helpers, destructor calls `Stop()`, `Track()` throws `UsageError("Track() after Stop()")` in Stopping/Stopped and queues before `Start()`.
- Tests: `LifecycleTest.*` (12 tests, new `lifecycle_test.cpp`), `TrackerTest.TrackBeforeStartQueues`, `TrackerTest.TrackAfterStopThrows`; `TestNode` in `probe_event.h` drives the real state machine.
- Tracer gate (auto-evidence, `<verify>` re-run end to end): the plan's verify command passed (LifecycleTest, WorkerThreadTest, Subscription*, DestructionOrderTest: 32 tests; the two Tracker tests), full `voyah_slam_tests` (276) and `voyah_events_tests` (80) green, `LifecycleTest.*` 90 repeats clean in Debug, 20 repeats clean under TSan. Expansion followed.

## Task 2 (TDD): Mapper, LoopCloser, GlobalBA and the Slam composite

### RED (`ef696f7`, `test(01-12)`)

Five `SlamTest.Lifecycle*` tests. Run one by one against the Task 1 tree (Mapper, LoopCloser, GlobalBA and Slam still on `BeginRun` / `EndRun`):

| Test | Result before GREEN | Reason (semantic assessment) |
|------|--------------------|------------------------------|
| `LifecycleStartAfterStopThrows` | FAILED | after the rejected `Start()` the Mapper reported Running (restarted) instead of Stopped; the planned assertion |
| `LifecycleStopBeforeStartIsTerminal` | FAILED | same: Mapper restarted after `Stop()` on a Created Slam |
| `LifecycleStopIsIdempotentFromManyThreads` | hung (timeout 124) | four concurrent `Slam::Stop()` race on the component joins; the planned failure mode (T-01-20) |
| `LifecycleStartTwiceThrows`, `LifecycleTrackAfterStopThrows` | passed already | held by the Tracker from Task 1; kept as regression guards |

The RED evidence is a gtest console report, not one of the machine-classifiable formats (TAP, JUnit, swift-testing, unittest), so `gsd_run check tdd-red-evidence` was not applicable; the semantic assessment above stands in for it and is recorded honestly as such.

### GREEN (`70014e3`, `feat(01-12)`)

- Mapper, LoopCloser, GlobalBA: `Start()` / `Stop()` through `StartWith` / `StopWith`; the once-per-run Stop logs of LoopCloser and GlobalBA moved into the stop lambdas (they run for a Running component only, which is what `was_running` used to check).
- GlobalBA: `stop_requested()` comment is now "true once Stop() was called (Stopped is terminal); any thread".
- Slam: `std::atomic<events::RunState> state_`; `Start()` CAS Created to Starting (same messages as `StartWith`), starts Mapper, GlobalBA, LoopCloser, Tracker, on any exception stops every component (unstarted ones go Created to Stopped), stores Stopped, notifies, rethrows; `Stop()` handles Created (stops every component so they are terminal too) and Running, waits on Starting/Stopping, returns on Stopped. Class comment documents the rules.
- `EventNode::BeginRun()` / `EndRun()` deleted; the "must be called from one controlling thread" comments in `voyah_slam` headers replaced with the D-10..D-12 rules.
- `docs/DESIGN.md` (§6.1 wiring text) no longer mentions `BeginRun` / `EndRun`; the decisions-log row and the restart write-up stay with plan 01-13 as planned.

### Verification

| Gate | Result |
|------|--------|
| `check_all` (Debug) | 822 tests, 100% pass: 786 unit, 35 integration, 1 skipped (`VocabularyEquivalenceTest`, ORBvoc absent). No flaky test failed |
| TSan: `LifecycleTest.*` x20, `TrackerTest.Track*:GlobalBundle*:LoopCloser*` (48), `SlamTest.Lifecycle*` x20 | all pass, no "WARNING: ThreadSanitizer" |
| `git grep -e BeginRun -e EndRun -- libs` | empty |
| `git grep 'must be called from one controlling thread' -- libs/slam/include` | empty |
| `std::atomic<events::RunState> state_` in `slam.h` | present |
| Behaviour check (Tracker/Mapper path touched): `binocular_full_slam` corridor1, `--rate=0 --max-frames=40` | rc 0, 39 `[Mapper] [debug] FrameTracked`, final line `map: 3 Frames, 777 points` (baseline 39 / 3 Frames) |

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] `WorkerThread::Start()` left `running_` true when the thread could not be created**
- **Found during:** Task 1 (designing `LifecycleTest.FailedStartLeavesStopped`)
- **Issue:** `running_` is set before `std::thread` is constructed; a throwing constructor left `running()` true with no thread, so `Stop()`/destructor logic and a later `Start()` no-op were wrong.
- **Fix:** the construction is wrapped; on exception `running_` is reset and the exception propagates (StartWith then marks the component Stopped).
- **Files modified:** `libs/events/src/events/worker_thread.cpp`
- **Commit:** cd3c1a6

**2. [Rule 1 - Bug] The plan's `stop_requested_ = false` inside GlobalBA's start lambda is dead and racy**
- **Found during:** Task 2
- **Issue:** Stopped is terminal, so the flag is false until `Stop()`; resetting it in the start lambda could undo a `Stop()` that set it and was waiting on Starting, leaving a running BA uninterrupted. Also, setting it before `StopWith` would leave a refused self-stop with a permanently interrupted BA.
- **Fix:** no reset in `Start()`; `stop_requested_` is raised inside the stop lambda (restored if the worker refuses a self-stop) and after `StopWith` for the Created to Stopped path (the publish-hook test that calls `Stop()` on a hand-driven GlobalBA relies on it).
- **Files modified:** `libs/slam/src/global_ba/global_bundle_adjuster.cpp`, `libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h`
- **Commit:** 70014e3

**3. [Rule 3 - Blocking/housekeeping] CLAUDE.md test counts**
- CLAUDE.md quotes the `check` / `check_all` counts; they changed (786 unit, 822 total), so they were updated, as the R2 plan did. **Commit:** f9512bc

**Total deviations:** 3 auto-fixed (2 bugs, 1 housekeeping). **Impact:** none on scope; deviation 2 changes a detail the plan prescribed but not its behaviour.

## Known gaps (not deviations)

- A `Slam::Stop()` issued from a component's own thread (for example the Mapper's thread) is refused by that component after the components before it were already stopped: the Slam goes back to its previous state and rethrows, and a later `Stop()` from another thread completes it. Not covered by a test (the single-component self-stop is, `LifecycleTest.StopFromWorkerThreadThrows`); recorded here for plan 01-13's documentation.
- `LifecycleTest` and the Slam tests do not cover the JNI binding's use; the voyah_jni session needs the 0.6.0 notes from plan 01-14.

## Known Stubs

None.

## Threat Flags

None. The threat register's T-01-20 (concurrent Stop before join), T-01-21 (self-join) and T-01-22 (silent restart) are mitigated and tested: `LifecycleTest.ConcurrentStopsReturnOnlyAfterJoin`, `SlamTest.LifecycleStopIsIdempotentFromManyThreads` (TSan clean), `LifecycleTest.StopFromWorkerThreadThrows`, `StartAfterStopThrows` at both layers.

## Issues Encountered

- The first run of `SlamTest.Lifecycle*` against the RED tree hung on the concurrent-Stop test (the intended failure); it had to be killed and the tests run individually under `timeout`. A `pkill -f` pattern also matched the invoking shell once; harmless.

## Next Phase Readiness

Ready for 01-13 (documentation of the lifecycle rules, the restart decision and TODO #6 closure).

## Self-Check: PASSED

- Created/modified files exist on disk (`lifecycle_test.cpp`, `event_node.h`, `slam.cpp` checked).
- Commits `cd3c1a6`, `ef696f7`, `70014e3`, `f9512bc` are ancestors of HEAD (`refactor/lifecycle`).
- TDD gate: `test(01-12)` RED commit (`ef696f7`) precedes the `feat(01-12)` GREEN commit (`70014e3`) for Task 2.
- Acceptance criteria of both tasks re-run and passing (greps empty/present as required, both verify commands green).
