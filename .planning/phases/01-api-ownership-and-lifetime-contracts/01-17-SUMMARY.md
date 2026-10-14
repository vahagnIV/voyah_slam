---
phase: 01-api-ownership-and-lifetime-contracts
plan: 17
subsystem: logging
tags: [logger, spdlog, dist-sink, thread-safety, tsan, release, versioning]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-16: R4 (0.7.0) merged and pushed to master"
provides:
  - "Logger::Initialize re-targets loggers created before it (one shared spdlog dist_sink_mt)"
  - "Atomic level, serialized logger creation, LoggerTest.* (6 tests, TSan clean)"
  - "Version 0.8.0 on branch fix/logger-init-order (merge pending)"
affects: [voyah_jni, phase-01-R5-release, phase-05-PLAT-01]

actuals:
  tokens: 6400    # chars/4 over the 25739-byte realized diff (git diff 1106e49..HEAD)
  tasks: 3
  commits: 4
plan_head_before: 1106e4907a4829a9b658acb03e3b338bb2edde39
plan_head_after: b7f5d8a0fe3a01509e41bcb9a1309c7f2e29329a
commits: 4

tech-stack:
  added: []
  patterns:
    - "Process-wide logger state lives in function-local statics in logger.cpp (library machinery in src/, DESIGN section 7), not in static class members"
    - "All loggers share one dist_sink_mt; re-targeting is set_sinks on that one sink"

key-files:
  created:
    - libs/common/tests/logging/logger_test.cpp
  modified:
    - libs/common/include/voyahslam/logging/logger.h
    - libs/common/src/logging/logger.cpp
    - libs/common/tests/CMakeLists.txt
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CLAUDE.md
    - CMakeLists.txt
    - conanfile.py

key-decisions:
  - "One shared dist_sink_mt (function-local static) instead of rewriting each registered logger's sink vector: a single object to re-target, protected by spdlog's own mutex (D-18)"
  - "Level is a function-local std::atomic; creation is serialized by a function-local mutex held across spdlog::get and register_logger (control plane, not the map or event path)"
  - "Android logcat default sink not added; it stays in PLAT-01 (D-19)"

requirements-completed: [OWN-06]

duration: "about 15 min"
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 17: Release R5 (0.8.0) Logger Summary

**Every logger now writes through one shared spdlog `dist_sink_mt`, so `Logger::Initialize` re-targets components built before it; the level is atomic and logger creation is race-free; version 0.8.0 on branch `fix/logger-init-order` (not merged).**

## What was done

| Task | Commit | Content |
|------|--------|---------|
| 1 (tracer, TDD RED) | `8f258a7` | `LoggerTest` x3 (component created before `Initialize`, `Initialize` twice, `Initialize({})`); all 3 failed on the copy-at-creation logger |
| 1 (GREEN) | `24065a2` | `logger.cpp`: function-local shared `dist_sink_mt` with one stdout colour child; `Initialize` calls `set_sinks` (empty vector: one fresh stdout sink); `CreateLogger` builds every logger on that sink; private static sink vector and level removed from the `Logger` class; level in a function-local `std::atomic` |
| 2 | `e818a19` | `CreateLogger` holds a function-local mutex across `spdlog::get` and `register_logger`; `LoggerTest` x3 (`SetLevelAppliesToExistingLoggers`, `ConcurrentCreateOfOneNameReturnsOneLogger`, `ConcurrentLogAndInitialize`). The concurrent-create test failed on every one of 5 runs before the mutex and passed 5 of 5 after |
| 3 | `b7f5d8a` | DESIGN "Logging (ownership audit #8, 0.8.0)" section, decisions rows "Ownership audit #8 implemented" and "Package version 0.8.0", Next/test-suite lines; TODO ticks #8 (logcat item left open with a PLAT-01 pointer); ROADMAP remaining #9; version 0.8.0 in `CMakeLists.txt`, `conanfile.py`, README (4 places), DESIGN layout and SOVERSION lines; CLAUDE.md counts 798 unit / 838 total |

The public `Logger` API and `CLASS_LOGGER` are unchanged. The code is in `src/` (`logger.cpp`), not in a header, so the -O0 example apps instantiate nothing of it (the R2 lesson).

## Verification

| Check | Result |
|-------|--------|
| `LoggerTest.*`, Debug | 6 pass (5 repeated runs, all green) |
| `LoggerTest.*`, TSan (`cmake-build-tsan`, `setarch -R`) | 6 pass, 0 `WARNING: ThreadSanitizer` |
| `check_all` | 838 tests, 100% pass (798 unit, 39 integration, 1 vocabulary skipped: `VocabularyEquivalenceTest`, ORBvoc absent). No flaky test failed |
| Acceptance greps | `dist_sink_mt` in `logger.cpp`; `logging/logger_test.cpp` in `libs/common/tests/CMakeLists.txt`; `std::atomic<spdlog::level::level_enum>` in `logger.cpp`; branch `fix/logger-init-order`; `git grep '0\.7\.0' -- CMakeLists.txt conanfile.py README.md` prints nothing (`test_package/build/...` generated files are untracked build output) |

### Behaviour and performance check (apps, corridor1, `--rate=0 --max-frames=40`, Debug tree)

The Tracker, Mapper and `OptimizePose` log a debug line per frame and `dist_sink_mt` adds a mutex per log call, so both apps were run on the branch base (`1106e49`, before the change) and on the branch HEAD.

| App | Run | `FrameTracked` lines | `[OptimizePose]` lines | final `map:` line | log lines |
|-----|-----|----------------------|------------------------|-------------------|-----------|
| binocular | before | 39 | 390 | `map: 3 Frames, 777 points` | 541 |
| binocular | after | 39 | 390 | `map: 3 Frames, 777 points` | 541 |
| monocular | before | 0 | 0 | `map: 0 Frames, 0 points` | 123 |
| monocular | after | 0 | 0 | `map: 0 Frames, 0 points` | 123 |

Both apps exit rc 0. Binocular stays at the bar from R2 (at least 36 FrameTracked, at least 3 Frames). Monocular is unchanged from R1 (WaitingForMap, 0 Frames). Log output is unchanged: a diff of the (component tag, level) histogram before versus after is empty, and the `[YYYY-MM-DD HH:MM:SS.mmm] [Component] [level]` format and the stdout colour sink still appear. Logs: `scratchpad/r5_base_*.log`, `r5_after_*.log`.

## Deviations from Plan

None - plan executed exactly as written. (TDD was applied to Task 1 as a separate RED commit before the GREEN commit, and to Task 2 as RED observed before the mutex; the RED state of Task 2 was observed but not committed separately, because the tests and the fix are one small unit.)

## Known Stubs

None.

## Threat Flags

None. T-01-28 mitigated (`dist_sink_mt`, atomic level, serialized creation; `ConcurrentLogAndInitialize` and `ConcurrentCreateOfOneNameReturnsOneLogger` clean under TSan). T-01-29 mitigated (`ComponentCreatedBeforeInitializeUsesNewSinks`). T-01-SC accepted: no package added or upgraded.

## Notes for the release (R5 gates and merge, a later plan)

- The branch is not merged and nothing was pushed. The merge needs the user's literal approval per D-21.
- One behavioural point for the voyah_jni binding: `Logger::Initialize` may now be called at any time (also after a `Slam` exists) and re-targets existing components; pushing a sink directly onto one `logger->sinks()` (as `subscription_test.cpp` does) still works but stays attached for the logger's life, as before.
- Known flaky tests were not triggered in the `check_all` run above.

## Self-Check

Files exist: `libs/common/tests/logging/logger_test.cpp`, `libs/common/src/logging/logger.cpp` (contains `dist_sink_mt` and `std::atomic<spdlog::level::level_enum>`). Commits `8f258a7`, `24065a2`, `e818a19`, `b7f5d8a` are ancestors of HEAD on `fix/logger-init-order`. `commits: 4` measured with `git rev-list --count 1106e49..HEAD` before this SUMMARY commit. Result: PASSED.
