---
phase: 01-api-ownership-and-lifetime-contracts
plan: 14
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning, lifecycle, run-state, voyah-jni]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plans 01-12..01-13: lifecycle state machine (Created, Running, Stopped; Stopped terminal), DESIGN and version 0.6.0"
provides:
  - "R3 gates for voyahslam 0.6.0 all green on the second full run (check_all, apps, TSan, ASan, lifecycle checks, three conan create)"
  - "Option (a) chosen by the user: Slam::Stop() from a component thread is refused before anything is stopped; ~Slam on a component thread terminates; new Slam::run_state()"
  - "refactor/lifecycle merged into master (no-ff) and pushed; origin/master == master"
  - "voyahslam/0.6.0 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every API break of 0.6.0"
affects: [voyah_jni, phase-01-R4]

actuals:
  tokens: 6000    # chars/4 over the 4 fix commits (option (a) change, tests, docs); the release gates changed no source
  tasks: 3
  commits: 5      # MEASURED: git rev-list --count 9f03314..HEAD at merge time: 4 commits on the branch + the merge commit; the docs commit that adds this SUMMARY is not counted
plan_head_before: 9f03314956c56fd79397687beb3058e946dcfe6b
plan_head_after: 124a668615cdf6b08aee7fb3f816138c664f4bfe
commits: 5

tech-stack:
  added: []
  patterns:
    - "A documented gap found at the release checkpoint is closed on the branch and every gate is re-run (plan rule), not shipped with a note"
    - "WorkerThread::OnWorkerThread() is lock-free; each component exposes OnWorkerThread() so Slam::Stop() can refuse before stopping anything"

key-files:
  created: []
  modified: []

key-decisions:
  - "Option (a) chosen by the user: Slam::Stop() on a component thread throws UsageError and changes nothing (no half-stopped Slam); destroying the Slam on a component thread logs and terminates"
  - "Slam::run_state() added so a binding can observe the lifecycle"
  - "Stopped is terminal for Slam and every component; a new session is a new Slam"

requirements-completed: [OWN-04]

duration: "two full gate runs (TSan integration 724 s, slam 371 s), option (a) change, merge and push under 1 min"
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 14: Release R3 (0.6.0) Summary

**R3 passed every D-21 gate on its second full run (after the user chose to close the Slam::Stop()-from-a-component-thread gap first), was merged into master on the user's "merge and push" as 124a668, and pushed; OWN-04 ships as voyahslam 0.6.0.**

## Task 1: Release gates (two runs)

### Run 1 (branch HEAD 9f03314): all gates passed

| Gate | Result |
|------|--------|
| `check_all` (Debug tree) | 822/822 pass: 786 unit, 35 integration, 1 skipped (`VocabularyEquivalenceTest`, ORBvoc absent) |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, 0 Frames |
| `binocular_full_slam`, same arguments | rc 0, 39 FrameTracked, 3 Frames, 777 points (bar from 01-11: at least 36 FrameTracked, at least 3 Frames) |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 79 pass; `voyah_slam_tests` (filters Tracker*, Mapper*, Slam*, GlobalBundle*, LoopCloser*, MapFollower*) 196 pass; `voyah_integration_tests` 33 pass; 0 TSan warnings |
| ASan `voyah_events_tests` | 80 pass, no AddressSanitizer or LeakSanitizer error |
| `LifecycleTest.*` + grep of the old comment rule | 12/12 pass; grep for 'must be called from one controlling thread' in `libs/slam/include` empty |
| `conan create` voyahslam/0.6.0 | linux-gcc13 static, shared, android-arm64: all pass; test_package printed "voyahslam 0.6" on both linux runs |

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/r3_*.log`.

### Task 2 reply 1 and the option (a) change round

The checkpoint presented the documented gap: `Slam::Stop()` called from a component's own thread (for example a listener callback) half-stopped the Slam. The user's reply, verbatim: "(a) first"

Option (a) was implemented on `refactor/lifecycle`, so nothing is half-stopped, and every gate was re-run (run 2 below).

| Commit | Change |
|--------|--------|
| `d4df304` | `feat(01-14)`: add `Slam::run_state()` |
| `f46763d` | `test(01-14)`: RED `SlamTest.LifecycleStopFromComponentThreadChangesNothing` and `SlamDeathTest.DestroyedOnAComponentThreadTerminates` |
| `3c88365` | `feat(01-14)`: `Slam::Stop()` refuses from a component thread before stopping anything (lock-free `WorkerThread::OnWorkerThread()`, components expose `OnWorkerThread()`); `~Slam` on a component thread logs and terminates |
| `33ab25c` | `docs(01-14)`: DESIGN section 4 and 7, decisions-log row "option (a) chosen by the user, 2026-10-14", TODO ticked, test counts 787 unit / 827 total |

The new tests ran 20 repetitions under TSan: 0 warnings.

### Run 2 (branch HEAD 33ab25c): full re-run, all gates green

| Gate | Result |
|------|--------|
| `check_all` first run | one known flake: `SlamTest.DuplicatesFusedInLongSequence` (`slam_test.cpp:1033`, duplicates 1 vs 0, of 433 points). Three isolated reruns of that test passed. |
| `check_all` full rerun | 827/827 pass: 787 unit, 39 integration, 1 skipped (`VocabularyEquivalenceTest`, ORBvoc absent) |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, 0 Frames |
| `binocular_full_slam`, same arguments | rc 0, 39 FrameTracked, 3 Frames, 777 points (bar at least 36 / at least 3) |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 80 pass; `voyah_slam_tests` 196 pass (371 s); `voyah_integration_tests` 36 pass (724 s); 0 TSan warnings |
| ASan `voyah_events_tests` | 81 pass, no AddressSanitizer or LeakSanitizer error |
| `LifecycleTest.*` + grep | 12/12 pass; grep empty |
| `conan create` voyahslam/0.6.0 | linux-gcc13 static and shared rc 0, both print "voyahslam 0.6: unproject error 2.48e-14, pose error 8.21e-16, inliers 30"; android-arm64 rc 0 |

The flake is the same one recorded in 01-10 (`SlamTest.DuplicatesFusedInLongSequence`); it is not caused by this release and is not new, and no gate was weakened or filtered to get the green rerun.

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/r3b_*.log`.

## Task 2: User review

Two replies were recorded.

- After run 1 (gap in `Slam::Stop()` from a component thread presented with options), the user's reply, verbatim: "(a) first"
- After run 2, the user's reply, verbatim: "merge and push"

## Task 3: Merge and push

- Preconditions checked before merging: branch `refactor/lifecycle` HEAD 33ab25c, tree clean apart from the untracked `.planning/config.json`, `state.json`, `milestone.lock` and `.planning/debug/` (left alone); `master` == `origin/master` == 9ba2cd3.
- `git switch master`, `git merge --no-ff refactor/lifecycle`. Merge commit: **124a668615cdf6b08aee7fb3f816138c664f4bfe**, subject "Merge branch 'refactor/lifecycle': component lifecycle state machine, Stopped is terminal; version 0.6.0". It brings in 13 branch commits (32 files, +1222/-120).
- `git push origin master`: `9ba2cd3..124a668  master -> master` succeeded; `git rev-parse master origin/master` gave identical SHAs.
- The branch `refactor/lifecycle` was not deleted. No force push.

## Deviations from Plan

### User-directed change round

**1. [User decision at the checkpoint] Close the Slam::Stop()-from-a-component-thread gap before merging**
- **Found during:** Task 2 (documented gap from 01-13, presented with options)
- **Issue:** `Slam::Stop()` on a component's own thread stopped part of the Slam and then threw for the component itself.
- **Fix:** option (a), commits `d4df304`, `f46763d`, `3c88365`, `33ab25c`; Task 1 and Task 2 were repeated in full, as the plan requires for a fix on the branch.
- **Files modified:** `libs/events` `worker_thread.{h,cpp}` and its test, `libs/slam` `slam.{h,cpp}`, the Tracker, Mapper, LoopCloser and GlobalBundleAdjuster headers, `slam_test.cpp`, `docs/DESIGN.md`, `docs/TODO.md`, `README.md`, `CLAUDE.md` (counts)

No other deviations.

## Open items

- `SlamTest.DuplicatesFusedInLongSequence` remains a rare flake (duplicates 1 vs 0 of 433 points at `slam_test.cpp:1033`; seen in 01-10 and once in run 2 here). Not investigated in this plan; recorded here so a later plan can seed-pin or fix it.

## Known Stubs

None.

## Threat Flags

None. T-01-24 mitigated: blocking-human checkpoint, literal replies recorded above, minor bump 0.6.0, and the voyah_jni message below.

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.6.0** (`voyahslam/0.6.0`, SOVERSION 0.6), merged and pushed to master as 124a668. R3 (ownership audit #6, OWN-04) is an API break for the binding, on top of 0.5.0. Every break and rule:

1. **`Start()` and `Stop()` of `Slam` and of every component may be called from any thread** (a Java `close()` or a GC cleaner is fine). Each has a lifecycle state, Created to Running to Stopped, kept by an atomic compare-and-swap.
2. **Invalid transitions throw `voyahslam::UsageError`; map it to `IllegalStateException`:**
   - `Start()` twice;
   - `Start()` after `Stop()`: **Stopped is terminal.** A stopped Slam cannot be restarted; create a new `Slam` for a new session. (`WorkerThread` and `MapViewer` stay restartable.)
   - `AddObserver` (and `Subscription::reset()`) while the component is starting, running or stopping: subscribe before `Start()`;
   - `Track()` after `Stop()`.
3. **`Stop()` is idempotent** from any thread and returns only after the workers are joined; a concurrent `Start()` or `Stop()` is waited for. `Stop()` before `Start()` moves Created to Stopped (the object is then unusable).
4. **`Track()` before `Start()` queues** (the frames are processed once `Start()` runs); `Track()` after `Stop()` throws `UsageError`.
5. **`Slam::Stop()` called from a listener callback that runs on a component thread throws `UsageError` (map to `IllegalStateException`) and changes nothing**: nothing is stopped, the Slam keeps running, so the binding can call `Stop()` again from its own thread. Do not stop the Slam from a callback; hand the request to a binding thread.
6. **Destroying the `Slam` on a component thread terminates the process** (it logs and calls `std::terminate`). Never drop the last reference to the `Slam` from a listener callback; release it from a binding thread.
7. **New `Slam::run_state()`** returns the lifecycle state (Created, Running, Stopped) so the binding can check it instead of tracking it separately.
8. A failed `Start()` leaves the component Stopped, and `Slam::Start()` rolls back the components it had already started; after a failed `Start()` create a new `Slam`.

## Self-Check

Merge commit 124a668 is an ancestor of master HEAD and equals origin/master at push time; the fix commits `d4df304`, `f46763d`, `3c88365`, `33ab25c` are ancestors of master; gate numbers above are copied from the run 2 report. Result: PASSED.
