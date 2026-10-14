---
phase: 01-api-ownership-and-lifetime-contracts
plan: 18
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning, logger, dist-sink, voyah-jni]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-17: logger shared dist sink, version 0.8.0 on fix/logger-init-order (HEAD ed876c1)"
provides:
  - "R5 gates for voyahslam 0.8.0 all green on the first run (check_all, apps, TSan, ASan, LoggerTest, three conan create)"
  - "fix/logger-init-order merged into master (no-ff, 93c00ce) and pushed; origin/master == master"
  - "voyahslam/0.8.0 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every API break of 0.8.0"
affects: [voyah_jni, phase-01-R6, phase-05-PLAT-01]

actuals:
  tokens: 0       # chars/4 over the diff this plan changed: no source file was changed, only the merge
  tasks: 3
  commits: 1      # MEASURED: git rev-list --count ed876c1..HEAD at the merge (the no-ff merge); the branch commits belong to 01-17
plan_head_before: ed876c1
plan_head_after: 93c00ce44ce5fdf7b56ad57e3bee8bd25f334c66
commits: 1

tech-stack:
  added: []
  patterns:
    - "Release plan with no fixes: gates run once on the branch HEAD, the merge is the only commit"

key-files:
  created: []
  modified: []

key-decisions:
  - "Merged on the user's literal 'merge and push'; the branch is kept, no force push"

requirements-completed: [OWN-06]

duration: "gates about 25 min (TSan slam 6m12s, integration 11m56s), merge and push under 1 min"
completed: 2026-10-15
status: complete
---

# Phase 1 Plan 18: Release R5 (0.8.0) Summary

**R5 passed every D-21 gate on its first run, was merged into master on the user's "merge and push" as 93c00ce, and pushed; OWN-06 ships as voyahslam 0.8.0 (every logger writes through one shared sink, so `Logger::Initialize` re-targets components built before it).**

## Task 1: Release gates (branch HEAD ed876c1, one run, no fixes)

| Gate | Result |
|------|--------|
| `check_all` (Debug tree) | 838/838 pass (798 unit, 39 integration, 1 skipped: `VocabularyEquivalenceTest`, ORBvoc absent); no flake; 26 s |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, 0 Frames, 0 FrameTracked (known baseline: two-view init rejected for low parallax) |
| `binocular_full_slam`, same arguments | rc 0, 39 FrameTracked, 3 Frames, 777 points (bar from 01-11: at least 36 FrameTracked, at least 3 Frames) |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 80 pass (0.4 s); `voyah_slam_tests` 196 pass (6m12s); `voyah_integration_tests` 36 pass (11m56s, ran to completion); `LoggerTest.*` 6 pass (0.2 s); 0 ThreadSanitizer warnings |
| ASan | `voyah_events_tests` 81 pass; `LoggerTest.*` 6 pass; no AddressSanitizer or LeakSanitizer error |
| `LoggerTest.*`, Debug | 6 pass; `sinks_` is gone from `logger.h` |
| `conan create` voyahslam/0.8.0 | linux-gcc13 static (47.5 s) and shared (49.1 s) rc 0, both print "voyahslam 0.8: unproject error 2.48e-14, pose error 8.21e-16, inliers 30"; android-arm64 (53.3 s) rc 0 |

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/r5_gate_*.log`.

## Task 2: User review

The user's reply, verbatim: "merge and push"

## Task 3: Merge and push

- Preconditions checked before merging: branch `fix/logger-init-order` HEAD ed876c1, tree clean apart from the untracked `.planning/config.json`, `state.json`, `milestone.lock` and `.planning/debug/` (left alone); `master` == `origin/master` == 1106e49 (after `git fetch`).
- `git switch master`, `git merge --no-ff fix/logger-init-order`. Merge commit: **93c00ce44ce5fdf7b56ad57e3bee8bd25f334c66**, subject "Merge branch 'fix/logger-init-order': loggers share one dist sink, Initialize re-targets existing loggers; version 0.8.0". It brings in the branch commits of plan 01-17 and its docs (15 files, +377/-44).
- `git push origin master`: `1106e49..93c00ce  master -> master` succeeded; `git rev-parse master origin/master` gave identical SHAs.
- The branch `fix/logger-init-order` was not deleted. No force push.

## Deviations from Plan

None - plan executed exactly as written. No gate failed, no fix commit was needed.

## Open items

- Remaining ownership-audit item: #9 (`images::Image`, OWN-07, plans 01-19/01-20).
- The Android logcat default sink stays in PLAT-01 (Phase 5); the binding passes spdlog's `android_sink` through `Logger::Initialize` meanwhile.

## Known Stubs

None.

## Threat Flags

None. T-01-30 mitigated: blocking-human checkpoint, the user's literal reply above, minor bump 0.8.0, and the voyah_jni message below.

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.8.0** (`voyahslam/0.8.0`, SOVERSION 0.8), merged and pushed to master as 93c00ce. R5 (ownership audit #8, OWN-06) changes logging behaviour for the binding, on top of 0.7.0. What changed and what to do:

1. **`Logger::Initialize(sinks)` may now be called at any time**, also after `Slam` and its components exist. It **re-targets every logger already created** (all loggers write through one shared spdlog `dist_sink_mt`). Pass spdlog's `android_sink` from the binding; an **empty vector restores stdout**. The old requirement to call `Initialize` before constructing any component is gone, and so is the silent loss of output for components built earlier.
2. **`Logger::SetLevel` applies to every existing logger**, not only to loggers created afterwards. The level is atomic.
3. **Public signatures are unchanged**, but the private static sink-vector and level members left the `Logger` class in `logger.h`. The class layout changed, so **rebuild the binding against 0.8.0**; no source change should be needed.
4. Logger creation is now race-free (two threads creating the same logger name get one logger), and `Initialize` while other threads log is safe (TSan clean). Log format and the stdout colour sink are unchanged.
5. A sink pushed directly onto one `logger->sinks()` still works but stays attached for that logger's life; use `Initialize` for the binding's sinks.
6. Nothing else changed for the binding: lifecycle, subscription, ownership and `OrbFeatureExtractor::Workspace` rules of 0.4.0 to 0.7.0 stand. Recreate the Conan package `voyahslam/0.8.0` (linux-gcc13 and android-arm64 are in the local cache of the build machine).

## Self-Check

Merge commit 93c00ce is an ancestor of master HEAD and equals origin/master at push time; the branch HEAD `ed876c1` is an ancestor of master; gate numbers above are copied from the Task 1 report. Result: PASSED.
