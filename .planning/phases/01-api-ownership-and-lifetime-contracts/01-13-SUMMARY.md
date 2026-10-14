---
phase: 01-api-ownership-and-lifetime-contracts
plan: 13
subsystem: lifecycle-docs
tags: [lifecycle, documentation, decisions-log, version-0.6.0, tsan, usage-error]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "Plan 01-12: lifecycle state machine (StartWith/StopWith, Slam state_, UsageError rules) on branch refactor/lifecycle"
provides:
  - "DESIGN section 4 'Lifecycle' paragraph: states, any-thread Start/Stop, UsageError cases, Track rules, failed-Start rollback, Stopped is terminal (Stop then Start unsupported), WorkerThread and MapViewer stay restartable"
  - "DESIGN decisions-log rows 'Ownership audit #6 implemented' and 'Package version 0.6.0'"
  - "Known gap documented: Slam::Stop() on a component's own thread (DESIGN section 4 and a docs/TODO.md open item with options)"
  - "Version 0.6.0 in CMakeLists.txt, conanfile.py, README.md and the DESIGN layout/SOVERSION lines"
  - "Phase 1 criterion 4 and OWN-04 aligned with locked D-12"
  - "50-repetition ThreadSanitizer stress of LifecycleTest.* and SlamTest.Lifecycle*: clean"
affects: [01-14, voyah_jni]

actuals:
  tokens: 6000   # chars/4 over the 8 changed files (109 lines added, 23 removed)
  tasks: 2
  commits: 1     # MEASURED: git rev-list --count plan_head_before..HEAD at SUMMARY time; Task 2 changed no source
plan_head_before: 6843abd91967b1278fc09f0bb8d3acd807f60d18
plan_head_after: c7f92ca45f8c77bea2dcb449558220c728a0942a
commits: 1

tech-stack:
  added: []
  patterns:
    - "Documented behavior is read from event_node.h / slam.cpp / 01-12-SUMMARY, not only from the plan wording"

key-files:
  created: []
  modified:
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CMakeLists.txt
    - conanfile.py
    - .planning/ROADMAP.md
    - .planning/REQUIREMENTS.md

key-decisions:
  - "Stop() then Start() is not supported: Stopped is terminal for Slam and its components, a new session is a new Slam (D-10); WorkerThread and MapViewer stay restartable (D-26)"
  - "The Slam::Stop()-from-a-component-thread gap is documented, not fixed; options (a)-(d) are in docs/TODO.md for the user to decide"
  - "Package version 0.6.0 (SOVERSION 0.6)"

requirements-completed: [OWN-04]

duration: "about 25 min"
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 13: Lifecycle documentation and 0.6.0 Summary

**The lifecycle rules and the restart decision (Stopped is terminal, a new session is a new `Slam`) are written into DESIGN, TODO, ROADMAP and the planning docs, the package is at 0.6.0, and `LifecycleTest.*` and `SlamTest.Lifecycle*` ran 50 clean repetitions each under ThreadSanitizer.**

## Task 1: Document the lifecycle, align the Phase 1 criterion and OWN-04 with D-12, bump to 0.6.0

Commit `c7f92ca` (branch `refactor/lifecycle`).

- `docs/DESIGN.md` section 4: new "Lifecycle" paragraph (five states, one atomic per component and in `Slam`; `Start()` / `Stop()` from any thread; the `UsageError` cases: Start twice, Start after Stop, AddObserver / `Subscription::reset()` while Starting/Running/Stopping, `Track()` after `Stop()`, Stop on a component's own worker thread; Stop before Start moves Created to Stopped; Stop idempotent and returns only after the join; `Track()` before `Start()` queues; failed Start leaves Stopped and `Slam::Start()` rolls back; **Stopped is terminal, Stop then Start unsupported**; `WorkerThread` and `MapViewer` stay restartable). Section 6.1 now points to it; the section 7 exception policy lists the lifecycle cases; layout and SOVERSION lines say 0.6.
- Decisions log: "Ownership audit #6 implemented" and "Package version 0.6.0"; "Next" list and test-suite note updated (822 tests, 786 unit, from `ctest -N`).
- "Known gap" documented as the notes required: `Slam::Stop()` from a component's own thread stops the components before it, is then refused with `UsageError`, the Slam restores its state and rethrows, earlier components stay stopped until a later `Stop()` from another thread. Consequence for callers (a JNI callback must not call `Slam::Stop()` on a component thread; hand it to another thread) is in DESIGN section 4; `docs/TODO.md` has the open item with options (a) check the calling thread against every component before stopping anything, (b) add a Slam-level test and keep it documented, (c) make it a request-stop completed elsewhere, (d) keep as is and tell voyah_jni. No code changed.
- `docs/TODO.md` ticks #6; `docs/ROADMAP.md` marks #6 done (0.6.0), remaining #7, #8, #9.
- Version 0.6.0: `CMakeLists.txt`, `conanfile.py`, `README.md` (package and shared-library names, and the stale `check` / `check_all` counts, now 786 / 822).
- `.planning/ROADMAP.md` Phase 1 criterion 4 and `.planning/REQUIREMENTS.md` OWN-04 reworded with scoped edits: `Track()` after `Stop()` throws, `Track()` before `Start()` queues; the decision that Stop then Start is not supported is stated in the criterion.
- Verify: all greps pass and `check` is green (786 of 786). Acceptance: `git grep 0.5.0 -- CMakeLists.txt conanfile.py README.md` is empty; the one remaining `controlling thread` line in DESIGN (viewer section) concerns `MapViewer`, which is not a component, and now says so; TODO shows #6 ticked; `before .Start(). queues` matches one line in each planning file.
- CLAUDE.md needed no change: its counts (786 unit, 822 total) already matched `ctest -N` after plan 01-12.

## Task 2: ThreadSanitizer stress of the lifecycle tests

No source change (no race found). Tree `cmake-build-tsan`, rebuilt for `voyah_events_tests` and `voyah_integration_tests`:

| Run | Repetitions | Result |
|-----|-------------|--------|
| `voyah_events_tests --gtest_filter='LifecycleTest.*'` | 50 (12 tests each) | exit 0; 50 "PASSED 12 tests"; 0 "WARNING: ThreadSanitizer"; 0 "FAILED" |
| `voyah_integration_tests --gtest_filter='SlamTest.Lifecycle*'` | 50 (5 tests each) | exit 0; 50 "PASSED 5 tests"; 0 "WARNING: ThreadSanitizer"; 0 "FAILED" |

Both under `setarch -R`. No defect found; T-01-23 mitigated.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical docs] README test counts and viewer wording**
- **Found during:** Task 1
- **Issue:** README still said 739 / 764 tests (stale since earlier releases), and the DESIGN viewer paragraph kept "Start()/Stop() must be called from one controlling thread" next to the new any-thread component rule, which would read as a contradiction.
- **Fix:** README counts set to 786 / 822; the viewer sentence now says it is the viewer's rule and that components are any-thread (section 4).
- **Files modified:** `README.md`, `docs/DESIGN.md`
- **Commit:** c7f92ca

**Total deviations:** 1 auto-fixed (docs consistency). **Impact:** none on scope.

## Known Stubs

None.

## Threat Flags

None. No code changed.

## Issues Encountered

None. Note for the user: the Slam-level self-stop gap is an open decision (docs/TODO.md), not resolved here.

## Next Phase Readiness

Ready for 01-14 (release gates: full check_all, packages, voyah_jni notes). The notes for the binding are in the 0.6.0 decisions-log row: `UsageError` maps to `IllegalStateException`; do not call `Slam::Stop()` on a component thread.

## Self-Check: PASSED

- Modified files exist; commit `c7f92ca` is an ancestor of HEAD (`refactor/lifecycle`).
- Task 1 verify command (greps plus `check`, 786 of 786) re-run and passing; the tracked acceptance greps were checked.
- Task 2 repeated runs exit 0 with no TSan report and no failure.
