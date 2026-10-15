---
phase: 01-api-ownership-and-lifetime-contracts
plan: 20
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning, image, voyah-jni]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-19: images::Image view fixes, overflow guard, version 0.9.0 on fix/image-views"
provides:
  - "R6 gates for voyahslam 0.9.0 all green on the first run (check_all, apps, TSan, ASan, Image checks, golden digests, three conan create)"
  - "fix/image-views merged into master (no-ff, 9e57bfa) and pushed; origin/master == master"
  - "voyahslam/0.9.0 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every API break of 0.9.0"
  - "Phase 1 release history table (R1 to R6)"
affects: [voyah_jni, phase-01-verification]

actuals:
  tokens: 184       # chars/4 over git diff 18f2984..648b3ba (736 chars: the README soname example); the merge adds no new content
  tasks: 3
  commits: 2        # MEASURED: git rev-list --count 18f2984..9e57bfa (648b3ba README fix + the no-ff merge); the branch commits belong to 01-19
plan_head_before: 18f2984
plan_head_after: 9e57bfa4043a8742f1eeba86237ca32bf151d9c3
commits: 2

tech-stack:
  added: []
  patterns:
    - "Release plan with no code fixes: gates run on the branch HEAD, the merge is the only code-bearing commit"

key-files:
  created: []
  modified:
    - README.md

key-decisions:
  - "Merged on the user's literal 'merge and push'; the branch is kept, no force push"

requirements-completed: [OWN-07]

duration: "gates about 45 min (TSan slam 6m11s, integration 11m56s; one integration attempt cut off by the background time limit and re-run), merge and push under 1 min"
completed: 2026-10-15
status: complete
---

# Phase 1 Plan 20: Release R6 (0.9.0) Summary

**R6 passed every D-21 gate, was merged into master on the user's "merge and push" as 9e57bfa, and pushed; OWN-07 ships as voyahslam 0.9.0 (`images::Image` is empty by default, view-correct in Convert / operator*= / operator* / CopyTo, and rejects overflowing sizes).**

## Task 1: Release gates (functional code identical to acc0742; doc commit 648b3ba after)

| Gate | Result |
|------|--------|
| `check_all` (Debug tree) | 852/852 pass (812 unit, 39 integration, 1 skipped: `VocabularyEquivalenceTest`, ORBvoc absent); no flake; 25.8 s |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, 0 Frames, 0 FrameTracked (known baseline: two-view init rejected for low parallax) |
| `binocular_full_slam`, same arguments | rc 0, 39 FrameTracked, 3 Frames, 777 points (bar from 01-11: at least 36 FrameTracked, at least 3 Frames) |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 80 pass (0.39 s); `voyah_slam_tests` 196 pass (6m11s); `voyah_integration_tests` 36 pass (11m56s); 0 ThreadSanitizer warnings |
| ASan | `voyah_events_tests` 81 pass; `ImageTest.*` 14 pass; no AddressSanitizer or LeakSanitizer error |
| `ImageTest.*`, Debug | 14 pass |
| `OrbExtractorGoldenTest.Corridor1OutputUnchanged` | pass (568 ms), not skipped, digests unchanged |
| `conan create` voyahslam/0.9.0 | linux-gcc13 static (46.9 s) and shared (48.4 s) rc 0, both print "voyahslam 0.9: unproject error 2.48e-14, pose error 8.21e-16, inliers 30"; android-arm64 (44.8 s) rc 0 |
| Version grep | the only `0.8.0` left in `CMakeLists.txt`, `conanfile.py`, `README.md` is `conanfile.py:75` `yaml-cpp/0.8.0` (third-party, left) |

Notes:
- The first TSan `voyah_integration_tests` attempt was cut off by the background time limit before it finished (log `r6_gate_tsan_integration_cutoff.log`, no TSan warning in the partial output). It was re-run alone to completion (36 pass, 11m56s, 0 warnings); the gate was not weakened.
- Gates ran on code identical to acc0742; the only commit after that was the doc fix 648b3ba (README soname example `so.0.8` to `so.0.9`), see Deviations. Functional gates are not affected by a README line.

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/r6_gate_*.log`.

## Task 2: User review

The user's reply, verbatim: "merge and push"

## Task 3: Merge and push

- Preconditions checked before merging: branch `fix/image-views` HEAD 648b3ba, tree clean apart from the untracked `.planning/config.json`, `state.json`, `milestone.lock` and `.planning/debug/` (left alone); `master` == `origin/master` == 0f0b97a (after `git fetch`).
- `git switch master`, `git merge --no-ff fix/image-views`. Merge commit: **9e57bfa4043a8742f1eeba86237ca32bf151d9c3**, subject "Merge branch 'fix/image-views': Image views honour offset and stride, empty default image; version 0.9.0". It brings in the branch commits of plan 01-19, its docs and the README fix (13 files, +666/-64).
- `git push origin master`: `0f0b97a..9e57bfa  master -> master` succeeded; `git rev-parse master origin/master` gave identical SHAs at that point.
- The branch `fix/image-views` was not deleted. No force push.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] README soname example still said 0.8**
- **Found during:** Task 1 (version grep)
- **Issue:** README showed the shared-library soname example as `so.0.8` after the bump to 0.9.0.
- **Fix:** changed to `so.0.9`.
- **Files modified:** README.md
- **Commit:** 648b3ba

### Plan-wording notes (no code impact)

- Task 3 and the 01-19 version grep cannot be literally empty: `conanfile.py:75` requires `yaml-cpp/0.8.0` (third-party version).

**Total deviations:** 1 auto-fixed (doc). **Impact on plan:** none.

## Phase 1 release history

| Release | Version | Audit item / scope | Merge commit on master |
|---------|---------|--------------------|------------------------|
| R1 | 0.4.0 | events are `shared_ptr<const T>`, RAII subscriptions (OWN-01/02) | 33a270f |
| R2 | 0.5.0 | MapFollower read path, thread-affine accessors test-only (OWN-03) | f1ecb28 |
| R3 | 0.6.0 | component lifecycle state machine, Stopped is terminal (OWN-04) | 124a668 |
| R4 | 0.7.0 | caller-owned `OrbFeatureExtractor::Workspace` (OWN-05) | 823470d |
| R5 | 0.8.0 | shared logger dist sink, `Initialize` re-targets (OWN-06) | 93c00ce |
| R6 | 0.9.0 | `images::Image` views, overflow guard, empty default (OWN-07) | 9e57bfa |

(0.2.0 and 0.3.0, ownership audit items #1 and #4, were merged before this phase's plans.)

## Open items

- Phase 1 verification is next (run by the orchestrator); the phase is not marked complete here.
- Single-row `operator[]` meaning (pixel versus row) stays a TODO item (docs/TODO.md).

## Known Stubs

None.

## Threat Flags

None. T-01-34 mitigated: blocking-human checkpoint, the user's literal reply above, minor bump 0.9.0, and the voyah_jni message below.

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.9.0** (`voyahslam/0.9.0`, SOVERSION 0.9), merged and pushed to master as 9e57bfa. R6 (ownership audit #9, OWN-07) fixes `images::Image`, on top of 0.8.0. This is the last item of the ownership audit. What changed and what to do:

1. **`images::Image` (and `TWorkingImage`) is empty when default-constructed**: width, height, stride, offset 0 and a null buffer. A **moved-from image is empty** as well. Previously the members were uninitialised.
2. **`Image(width, height)` throws `std::length_error` when `width * height * channels` overflows `size_t`**, before allocating. The binding passes caller-supplied sizes, so **it must catch `std::length_error`** (and may see `std::bad_alloc` family exceptions for sizes that do not overflow but cannot be allocated) instead of letting it cross the JNI boundary.
3. **`Set()` is no longer callable on a const image** (it is non-const now). Code that called it through a `const Image&` stops compiling; use a non-const image.
4. **Sub-image views alias the parent's buffer** (now documented): writing through a view changes the parent, and a view must not outlive the buffer's owner. This was always the behaviour; nothing changed, but the binding must not assume a view is a copy.
5. **`Convert`, `operator*=`, `operator*` and `CopyTo` now respect views** (offset, stride, channels): they touch only the view's pixels, row by row. Before, `Convert` ignored offset and stride and `*=` / `*` ran over the first `width * height * C` elements of the buffer (wrong pixels, writes outside the view). `CopyTo` on strided float images copied a quarter of each row; it now copies `sizeof(T)` bytes per element. Empty images stay empty.
6. **`Offset()` returns `size_t`** (was `unsigned`); view offsets are `size_t` end to end. A binding that stored it in a 32-bit field must widen it.
7. `operator[]` on a 1x1 image indexes channels (`img[y][x][c] == At(y, x, c)`), pinned by a test; its single-row behaviour is unchanged and tracked in TODO.
8. The class layout of `Image` changed (initialisers, `size_t` offset): **rebuild the binding against 0.9.0**. Nothing else changed for the binding: lifecycle, subscription, ownership, `Workspace` and logger rules of 0.4.0 to 0.8.0 stand. Recreate the Conan package `voyahslam/0.9.0` (linux-gcc13 static and shared and android-arm64 are in the local cache of the build machine).

## Self-Check: PASSED

Merge commit 9e57bfa is an ancestor of master HEAD and equals origin/master at push time; the branch HEAD 648b3ba is an ancestor of master; gate numbers above are copied from the Task 1 report.
