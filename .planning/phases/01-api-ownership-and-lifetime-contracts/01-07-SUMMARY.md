---
phase: 01-api-ownership-and-lifetime-contracts
plan: 07
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plans 01-01..01-06: shared_ptr<const T> events, RAII Subscription, inbox core, pools deleted, DESIGN and version 0.4.0"
provides:
  - "R1 gates for voyahslam 0.4.0 all green (check_all, apps, TSan, ASan, removed-symbol grep, three conan create)"
  - "refactor/event-ownership merged into master (no-ff) and pushed; origin/master == master"
  - "voyahslam/0.4.0 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every API break of 0.4.0"
affects: [voyah_jni, phase-02]

actuals:
  tokens: 0       # no source change in this plan; the merge only joins already-reviewed commits
  tasks: 3
  commits: 1      # the merge commit; measured plan_head_before..plan_head_after on master
plan_head_before: f7b83cb
plan_head_after: 33a270f301725da6e4aa70b0435a918724639cc8
commits: 1

tech-stack:
  added: []
  patterns:
    - "Release gates run on the branch, the merge happens only on the user's literal 'merge and push'"

key-files:
  created: []
  modified: []

key-decisions:
  - "No fix commits were needed: no gate failed, so no change was made on the branch by this plan"

requirements-completed: [OWN-01, OWN-02]

duration: "gates dominated by TSan runs (368 s + 707 s); merge and push under 1 min"
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 7: Release R1 (0.4.0) Summary

**R1 passed every D-21 gate, was merged into master on the user's "merge and push" as 33a270f, and pushed; OWN-01 and OWN-02 ship as voyahslam 0.4.0.**

## Task 1: Release gates (no commit, no gate failed)

| Gate | Result |
|------|--------|
| `check_all` (Debug tree) | 792 tests, 100% pass: 762 unit, 29 integration, 1 skipped (vocabulary test, ORBvoc absent). 24.8 s |
| Example apps, corridor1, `--rate=0 --max-frames=40` | `monocular_full_slam` and `binocular_full_slam` both rc 0, no "terminate", "tracker state:" printed. Mono: WaitingForMap/Initializing, 0 frames 0 points, 40 viewer events. Bino: Tracking/Mapping, 3 frames 777 points, 45 viewer events |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 67 pass; `voyah_slam_tests` (Tracker*, Mapper*, Slam*, GlobalBundle*, LoopCloser*) 186 pass, 368 s; `voyah_integration_tests` 27 pass, 707 s; 0 TSan warnings |
| ASan `voyah_events_tests` | 68 pass including death tests, 0 errors, 10 s |
| Removed-symbol grep (`ObjectPool`, `Recyclable`, `recycle_`, `StopAll`, `pool_overflow`, `event_pool_size`, `transaction_pool_size`, `max_events_in_flight`, `num_dropped_loops`, `num_results_dropped`, `SetWakeSignal`) over libs, examples, vizualization, test_package | empty |
| `conan create` voyahslam/0.4.0 | linux-gcc13 static rc 0 (48 s) and shared rc 0 (49 s); test_package printed "voyahslam 0.4: unproject error 2.48e-14, pose error 8.21e-16, inliers 30"; android-arm64 rc 0 (44 s) |

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/*.log`.

## Task 2: User review

The user's reply, verbatim: "merge and push"

## Task 3: Merge and push

- `git switch master`, `git merge --no-ff refactor/event-ownership` from branch HEAD cfa9a15 (tree clean apart from the orchestrator's untracked `.planning/config.json`, `state.json`, `milestone.lock`).
- Merge commit: **33a270f301725da6e4aa70b0435a918724639cc8**, subject "Merge branch 'refactor/event-ownership': events are shared_ptr<const T>, RAII subscriptions; version 0.4.0". The merge brings in 27 branch commits (80 files, +3385/-2533).
- `git push origin master`: `b0e957c..33a270f  master -> master` succeeded. Local master was 10 commits ahead of origin (the phase-1 planning docs), so those were pushed too. `git rev-parse origin/master master` gave identical SHAs.
- The branch `refactor/event-ownership` was not deleted.

## Deviations from Plan

None in the sense of fixes: no gate failed and no code was changed. Two points flagged during review, both accepted by the user's "merge and push":

1. **Mono app at 40 frames.** `monocular_full_slam` ended in WaitingForMap/Initializing (0 frames, 0 points) after 40 frames: monocular initialization did not complete in that window. The result was not compared against a 0.3.0 run, so it is not proven to match the previous behavior. The gate (rc 0, no terminate, "tracker state:" printed) passed; binocular tracked normally.
2. **`MapperTest.PooledMapTransactionIsReset` deleted** in commit 038f71b. It tested the transaction pool, which no longer exists, so the test has no subject; the count drop is part of the 792-test total above.

## Known Stubs

None.

## Threat Flags

None. T-01-13 mitigated (blocking-human checkpoint, literal reply recorded above); T-01-14 mitigated (minor bump 0.4.0 and the message below).

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.4.0** (`voyahslam/0.4.0`, SOVERSION 0.4), merged and pushed to master as 33a270f. R1 (ownership audit #3 + #2) is an API break for the binding. Every break:

1. **`events::Event*` becomes `events::EventPtr`** (`std::shared_ptr<const Event>`). Drop the pointer to acknowledge the event; there is no pool and no `Recycle`. Holding an event is safe even after the `Slam` is destroyed.
2. **`AsyncObserver::SetWakeSignal` is removed.** Wait on `inbox.wake_signal()` instead.
3. **`AddObserver` / `AddSyncObserver` return a `[[nodiscard]]` `events::Subscription`** that must be kept. The `Subscription` holder must be declared before the `Slam` (so it is destroyed after it).
4. **Connecting or `reset()` while a component is running throws `voyahslam::UsageError`** (map it to `IllegalStateException`). Destroying a `Subscription` of a running component terminates the process, so stop the `Slam` before closing listeners.
5. **Sync handlers take `const Event&`.**
6. **Removed options and counters:** `TrackerOptions::event_pool_size`, `MapperOptions::transaction_pool_size`, the pool overflow counters, `max_events_in_flight`, `num_dropped_loops()`, `num_results_dropped()`. `LoopDetected` and `GlobalBAFinished` are now never dropped.
7. **The session workarounds are no longer needed:** the declaration-order comment and the stop-before-drain workaround in `slam_session.{h,cpp}` can be deleted, as can any `StopAll`-style teardown guard on the binding side.

## Self-Check

Merge commit 33a270f is an ancestor of master HEAD and equals origin/master at push time; the gate numbers above are copied from the Task 1 report. Result: PASSED.
