---
phase: 01-api-ownership-and-lifetime-contracts
plan: 05
subsystem: testing
tags: [cpp20, gtest, shared_ptr, subscription, tsan, cmake, nodiscard]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plans 01-01..01-04: self-owning events, Subscription, production migration, all other test sources on the new API"
provides:
  - "slam_test.cpp on EventPtr and held Subscriptions; the last test source that did not link"
  - "Slam-level acceptance tests for OWN-01 (EventsOutliveSlam, StuckObserverHoldsEveryTrackerEvent) and OWN-02 (ObserverDestroyedWhileSlamRunsIsSafe, UnwindPastSubscriptionHolderDoesNotTerminate, AddObserverWhileRunningThrows)"
  - "memory::ObjectPool, Recyclable, ThreadSafePolicy and NoSyncPolicy deleted with their 17 tests"
  - "-Werror=unused-result on every test binary and both example apps"
  - "check_all green for the first time on the branch: 792 tests, 0 failures"
affects: [01-06, 01-07, voyah_jni]

actuals:
  tokens: 14300   # chars/4 over the realized diff (57329 chars, additions + deletions)
  tasks: 2
  commits: 2
plan_head_before: 50eb1f1ee5e8b040699c0a0c1e8b0cc03131bcaa
plan_head_after: 2b2d884555f30af4d668b41dcb6245910bb0b223
commits: 2

tech-stack:
  added: []
  patterns:
    - "Test-side lifetime rule (D-23): a std::vector<events::Subscription> (or the ProtocolRecorder that owns one) is declared before the Slam, so unwinding destroys it after ~Slam has stopped every producer"
    - "Released-event proof: a witness observer yields weak_ptrs to every event; after the run none may be alive, so a destroyed observer cannot have leaked its queue"

key-files:
  created: []
  modified:
    - libs/slam/tests/integration/slam_test.cpp
    - cmake/VoyahTests.cmake
    - examples/full_slam/CMakeLists.txt
    - libs/common/CMakeLists.txt
    - libs/common/tests/CMakeLists.txt
  deleted:
    - libs/common/include/voyahslam/memory/object_pool.h
    - libs/common/include/voyahslam/memory/recyclable.h
    - libs/common/include/voyahslam/memory/sync_policy.h
    - libs/common/src/memory/object_pool.cpp
    - libs/common/tests/memory/object_pool_test.cpp

key-decisions:
  - "ProtocolRecorder owns the Subscriptions it makes (Attach pushes into a private vector declared first); callers already declared the recorder before the Slam, so no call site changed"
  - "EventsOutliveSlam drops the ProtocolRecorder of the pool test: it only polled, and the stuck observers hold what the test checks, so the Slam is built in an inner scope with the Subscription vector outside it"
  - "-Werror=unused-result is restricted to GNU/Clang by generator expression and applied per test binary in voyah_add_test_binary, not globally, so the package build stays warning-tolerant"

patterns-established:
  - "Stuck-observer tests release their events after the producer is gone; Release() is valid at any time"

requirements-completed: [OWN-01, OWN-02]

coverage:
  - id: D1
    description: "Events a consumer still holds stay readable after the Slam is destroyed; held MapTransaction count equals num_transactions_built() read before destruction; point ids contiguous per map (OWN-01)"
    requirement: OWN-01
    verification:
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.EventsOutliveSlam"
        status: pass
    human_judgment: false
  - id: D2
    description: "A never-popping Tracker observer holds one event per frame for 300 static frames and nothing aborts (replaces the pool-overflow test)"
    requirement: OWN-01
    verification:
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.StuckObserverHoldsEveryTrackerEvent"
        status: pass
    human_judgment: false
  - id: D3
    description: "An observer destroyed while Slam runs is never notified again, the run continues and no queued event is leaked (OWN-02)"
    requirement: OWN-02
    verification:
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.ObserverDestroyedWhileSlamRunsIsSafe (Debug and TSan)"
        status: pass
    human_judgment: false
  - id: D4
    description: "An exception thrown after Start() unwinds through ~Slam before the Subscription holder, so nothing terminates (D-23, closes the T-01-05 coverage owed by 01-02)"
    verification:
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.UnwindPastSubscriptionHolderDoesNotTerminate (Debug and TSan)"
        status: pass
    human_judgment: false
  - id: D5
    description: "AddObserver while Slam runs throws UsageError and succeeds after Stop() (D-05)"
    verification:
      - kind: integration
        ref: "libs/slam/tests/integration/slam_test.cpp#SlamTest.AddObserverWhileRunningThrows"
        status: pass
    human_judgment: false
  - id: D6
    description: "Pool classes deleted, zero references in libs, examples, vizualization, test_package, cmake; discarded Subscription is a build error in tests and apps"
    verification:
      - kind: other
        ref: "git grep -n -e ObjectPool -e Recyclable -e SyncPolicy -e ThreadSafePolicy -e NoSyncPolicy -e object_pool -- libs examples vizualization test_package cmake  (empty); flags.make of every test binary and both apps contains -Werror=unused-result; probe file with a discarded nodiscard value fails to compile"
        status: pass
    human_judgment: false
  - id: D7
    description: "check_all green in cmake-build-debug"
    verification:
      - kind: integration
        ref: "cmake --build cmake-build-debug/build/Debug --target check_all: 100% passed, 792 tests, 1 skipped (vocabulary test, needs ORBvoc)"
        status: pass
    human_judgment: false

duration: 13min
completed: 2026-10-13
status: complete
---

# Phase 1 Plan 05: Slam-level lifetime tests, pool deletion and check_all Summary

**`slam_test.cpp` is on self-owning events and held Subscriptions with five new Slam-level tests (events outliving the Slam, an observer destroyed mid-run, unwinding past a Subscription holder, connecting while running), the three pool headers and their 17 tests are deleted, a discarded `Subscription` is a build error in tests and apps, and `check_all` is green at 792 tests.**

## Performance

- **Duration:** 13 min
- **Started:** 2026-10-13T14:17:00Z
- **Completed:** 2026-10-13T14:30:00Z
- **Tasks:** 2
- **Files modified:** 10 (5 changed, 5 deleted, 0 created)

## Accomplishments

- `slam_test.cpp`: `ProtocolRecorder` holds its Subscriptions and pops `EventPtr` with `const` casts; `ExternalObserversCanBeAddedToEveryComponent` holds its three Subscriptions in a vector declared before the Slam; the `global_ba_results_dropped` counter, its assignment, print and `record(...)` line are gone; `StuckObserver` uses `size_approx()` and `TryPop(EventPtr&)` and can release after the Slam is destroyed.
- Five new tests, all passing in Debug; the four timing-independent ones (`EventsOutliveSlam`, `StuckObserverHoldsEveryTrackerEvent`, `ObserverDestroyedWhileSlamRunsIsSafe`, `UnwindPastSubscriptionHolderDoesNotTerminate`) also pass under TSan with no `WARNING: ThreadSanitizer`.
- `ObserverDestroyedWhileSlamRunsIsSafe` goes beyond "does not crash": a witness observer on the same Tracker records a `weak_ptr` to every event, and after the run none of them is alive, so the destroyed observer's queue was released and nothing leaked.
- `UnwindPastSubscriptionHolderDoesNotTerminate` is the executed coverage the 01-02 summary owed for the unwinding edge: `~Slam` stops every producer before the Subscription holder is destroyed.
- Pools deleted (`object_pool.h`, `recyclable.h`, `sync_policy.h`, `object_pool.cpp`, `object_pool_test.cpp` with its 17 tests); `column_buffer.h` stays. The zero-reference grep is empty.
- `-Werror=unused-result` is in `flags.make` of every test binary and both apps; a probe with a discarded `[[nodiscard]]` value fails to compile under g++-13 with `-Werror=unused-result`. The whole Debug tree, apps included, builds with the flag.

## Task Commits

1. **Task 1: slam_test.cpp on the new API, plus the Slam-level lifetime and wiring tests** - `30afb41` (test)
2. **Task 2: Delete the pools, make discarded Subscriptions a build error, check_all green** - `2b2d884` (refactor)

**Plan metadata:** docs commit follows this summary (`docs(01-05): complete ...`).

## Files Created/Modified

- `libs/slam/tests/integration/slam_test.cpp` - migrated; two pool tests replaced, three tests added
- `cmake/VoyahTests.cmake` - `-Werror=unused-result` in `voyah_add_test_binary`
- `examples/full_slam/CMakeLists.txt` - the same option on `monocular_full_slam` and `binocular_full_slam`
- `libs/common/CMakeLists.txt`, `libs/common/tests/CMakeLists.txt` - pool source and test removed from the lists
- Deleted: the three `memory/` pool headers, `object_pool.cpp`, `object_pool_test.cpp`

## Decisions Made

- `ProtocolRecorder` owns its Subscriptions (a vector declared before its observers); every caller already declares the recorder before the Slam it attaches to, so the D-23 order holds without touching call sites.
- `EventsOutliveSlam` has no `ProtocolRecorder`: the pool test used it only to poll, and the stuck observers already hold everything the checks read. The Slam lives in an inner scope; the Subscription vector and the observers sit outside it.
- The compile option is a generator expression for GNU/Clang and is per target, so the installed package is not affected.

## TDD Gate Compliance

Task 1 is marked `tdd="true"`, but the behaviour under test was implemented by plans 01-01 and 01-02, and `voyah_integration_tests` did not compile before this plan (`slam_test.cpp` used the removed pool API). A genuine RED (a test that compiles and fails on the planned assertion) therefore could not exist: the migrated file and the new tests arrive in one `test(01-05)` commit, and there is no `feat(01-05)` commit because no production code changed. The plan-level `type` is `execute`, not `tdd`, so no gate is violated; this is recorded for the verifier. The new tests were judged for strength instead: each fails if the production behaviour regresses (a Subscription destroyed while running terminates, `AddObserver` while running must throw, queued events must expire).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Stale wording] ObjectPool named in a comment outside the removed code**
- **Found during:** Task 2 (zero-reference grep)
- **Issue:** the audit-item comment above `StuckObserverHoldsEveryTrackerEvent` mentioned `ObjectPool::Create`, which makes the plan's zero-reference grep print a line.
- **Fix:** reworded to "a bounded event pool ran dry".
- **Files modified:** `libs/slam/tests/integration/slam_test.cpp`
- **Verification:** the negated `git grep` exits 0 with no output.
- **Committed in:** `2b2d884`

**2. [Rule 1 - Stale wording] "dropped result" comment in RunSequence**
- **Found during:** Task 1
- **Issue:** a comment named a dropped global-BA result, a failure mode deleted by D-25.
- **Fix:** "a result that never arrives".
- **Files modified:** `libs/slam/tests/integration/slam_test.cpp`
- **Committed in:** `30afb41`

---

**Total deviations:** 2 auto-fixed (both comment wording, Rule 1)
**Impact on plan:** none. The assertion-preservation command is silent; the EXPECT_/ASSERT_ multiset outside the seven named tests equals the branch base.

## Issues Encountered

None. The TSan run of `EventsOutliveSlam` takes 14 s (instrumented stereo run); no data race was reported.

## Known Stubs

None.

## Threat Flags

None. T-01-10 is mitigated by `EventsOutliveSlam` and `ObserverDestroyedWhileSlamRunsIsSafe` (Debug and TSan); T-01-11 by `-Werror=unused-result` plus the `[[nodiscard]]` class.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- The branch is fully on the new API and `check_all` is green (792 tests). The Release build and the sanitizer trees were not rebuilt in full here, only `voyah_integration_tests` under TSan.
- Docs still owed to later plans: DESIGN §6/§6.1 and the decisions log (pools to `shared_ptr`, `Subscription`, unbounded `LoopDetected`/`GlobalBAFinished`), TODO entries for the removed options, the README/ROADMAP test counts in CLAUDE.md (746/773 are now 762 unit / 792 total), and the version bump at the end of R1.

## Self-Check: PASSED

Commits `30afb41` and `2b2d884` are ancestors of HEAD; the five deleted files are absent and `column_buffer.h` exists; `unused-result` is listed in both CMake files; `check_all` reports 100% passed (792 tests, 1 vocabulary test skipped); the TSan filter run shows no `WARNING: ThreadSanitizer`; the assertion-multiset command is empty for `slam_test.cpp`.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-13*
