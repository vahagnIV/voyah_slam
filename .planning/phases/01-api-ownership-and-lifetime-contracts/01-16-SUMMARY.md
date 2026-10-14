---
phase: 01-api-ownership-and-lifetime-contracts
plan: 16
subsystem: release
tags: [release, conan, tsan, asan, merge, versioning, orb, extractor, workspace, voyah-jni]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-15: OrbFeatureExtractor::Workspace, version 0.7.0 on refactor/extractor-workspace (HEAD 36a2e56)"
provides:
  - "R4 gates for voyahslam 0.7.0 all green on the first run (check_all, apps, TSan, ASan, ORB golden digests, three conan create)"
  - "refactor/extractor-workspace merged into master (no-ff, 823470d) and pushed; origin/master == master"
  - "voyahslam/0.7.0 in the local Conan cache (linux-gcc13 static and shared, android-arm64)"
  - "Message for the voyah_jni session listing every API break of 0.7.0"
affects: [voyah_jni, phase-01-R5]

actuals:
  tokens: 0       # chars/4 over the diff this plan changed: no source file was changed, only the merge
  tasks: 3
  commits: 1      # MEASURED: git rev-list --first-parent --count 36a2e56..823470d (the no-ff merge); the 6 branch commits belong to 01-15
plan_head_before: 36a2e56
plan_head_after: 823470d51dd25d3b7fd3dbd0632a85bdc8d18712
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

requirements-completed: [OWN-05]

duration: "gates about 35 min (TSan slam 6m11s, integration 11m57s), merge and push under 1 min"
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 16: Release R4 (0.7.0) Summary

**R4 passed every D-21 gate on its first run, was merged into master on the user's "merge and push" as 823470d, and pushed; OWN-05 ships as voyahslam 0.7.0 (one shared `OrbFeatureExtractor`, one `Workspace` per extraction thread).**

## Task 1: Release gates (branch HEAD 36a2e56, one run, no fixes)

| Gate | Result |
|------|--------|
| `check_all` (Debug tree) | 832/832 pass, 1 skipped (`VocabularyEquivalenceTest`, ORBvoc absent); no flake; 26 s |
| `monocular_full_slam`, corridor1, `--rate=0 --max-frames=40` | rc 0, WaitingForMap, 0 Frames, 0 FrameTracked (known baseline: two-view init rejected for low parallax) |
| `binocular_full_slam`, same arguments | rc 0, 39 FrameTracked, 3 Frames, 777 points (bar from 01-11: at least 36 FrameTracked, at least 3 Frames) |
| TSan (`setarch -R`, death tests excluded) | `voyah_events_tests` 80 pass (0.4 s); `voyah_slam_tests` (Tracker*, Mapper*, Slam*, GlobalBundle*, LoopCloser*, MapFollower*) 196 pass (6m11s); `voyah_integration_tests` 36 pass (11m57s); `OrbExtractorWorkspaceTest.*` 5 pass (10.7 s); 0 ThreadSanitizer warnings |
| ASan `voyah_events_tests` | 81 pass, no AddressSanitizer or LeakSanitizer error |
| `OrbExtractorGoldenTest.Corridor1OutputUnchanged` | OK in 662 ms, not skipped, digests not re-recorded (`VOYAHSLAM_RECORD_ORB_GOLDEN` unset) |
| `git grep 'mutable orb::ImagePyramid' -- libs` | no match |
| `conan create` voyahslam/0.7.0 | linux-gcc13 static (47 s) and shared (48 s) rc 0, both print "voyahslam 0.7: unproject error 2.48e-14, pose error 8.21e-16, inliers 30"; android-arm64 (45 s) rc 0 |

Logs: `/tmp/claude-1000/-home-vahagn-git-voyah-slam/64b7bb91-2b6a-4f03-b34e-c6d9d4f1e116/scratchpad/r4_*.log`.

## Task 2: User review

The user's reply, verbatim: "merge and push"

## Task 3: Merge and push

- Preconditions checked before merging: branch `refactor/extractor-workspace` HEAD 36a2e56, tree clean apart from the untracked `.planning/config.json`, `state.json`, `milestone.lock` and `.planning/debug/` (left alone); `master` == `origin/master` == 0122fd7 (after `git fetch`).
- `git switch master`, `git merge --no-ff refactor/extractor-workspace`. Merge commit: **823470d51dd25d3b7fd3dbd0632a85bdc8d18712**, subject "Merge branch 'refactor/extractor-workspace': caller-provided extractor Workspace, DetectAndExtract thread-safe; version 0.7.0". It brings in 6 branch commits (18 files, +507/-48).
- `git push origin master`: `0122fd7..823470d  master -> master` succeeded; `git rev-parse master origin/master` gave identical SHAs.
- The branch `refactor/extractor-workspace` was not deleted. No force push.

## Deviations from Plan

None - plan executed exactly as written. No gate failed, no fix commit was needed.

## Open items

- Remaining ownership-audit items: #8 (Logger, OWN-06, plans 01-17/01-18) and #9 (`images::Image`, OWN-07, plans 01-19/01-20).
- `docs/ROADMAP.md` and the other docs describe 0.7.0 as merged only through this plan's STATE/ROADMAP update; `docs/DESIGN.md` was already updated in 01-15.

## Known Stubs

None.

## Threat Flags

None. T-01-27 mitigated: blocking-human checkpoint, the user's literal reply above, minor bump 0.7.0, and the voyah_jni message below.

## Message for the voyah_jni session

Forward this to the voyah_jni session. `voyahslam` is now **0.7.0** (`voyahslam/0.7.0`, SOVERSION 0.7), merged and pushed to master as 823470d. R4 (ownership audit #7, OWN-05) is an API change for the binding, on top of 0.6.0. What changed and what to do:

1. **`OrbFeatureExtractor` is now const-correct and shareable.** It holds no mutable state after construction. **One extractor may be shared by several threads**, as long as **each thread (or each camera stream) uses its own `OrbFeatureExtractor::Workspace`.**
2. **New overload `OrbFeatureExtractor::DetectAndExtract(image, n, Workspace& out_workspace) const`.** The `Workspace` holds the image pyramid. In the frame loop, hold one `OrbFeatureExtractor::Workspace` per extraction thread (one per camera for stereo) and pass it on every call. The Workspace is **not thread-safe** itself: never use one Workspace from two threads at once. It re-initialises on its own when the image size, the number of pyramid levels or the scale factor changes, and returns exactly what a fresh Workspace returns, so it is safe to keep across frames.
3. **The allocating overload `DetectAndExtract(image, n)` still works** (source compatible) but allocates a pyramid on every call. Use it only for one-off calls; in the `TrackFrame` path keep one Workspace under the binding's existing mutex.
4. **`orb::ImagePyramid`** (public in `voyah_common`) gained `Width()`, `Height()`, `Fits()` and a `const operator[]`; `Initialize()` can be called again on an already-initialised pyramid. Only relevant if the binding uses the pyramid directly.
5. **Extractor output is bit-identical to 0.6.0** (golden digests unchanged), so no re-tuning is needed. Extraction cost per frame is unchanged within noise.
6. Nothing else changed for the binding: lifecycle, subscription and ownership rules of 0.4.0 to 0.6.0 stand. Recreate the Conan package `voyahslam/0.7.0` (linux-gcc13 and android-arm64 are in the local cache of the build machine).

## Self-Check

Merge commit 823470d is an ancestor of master HEAD and equals origin/master at push time; the branch commits `c548b1d`, `de00c27`, `69eed20`, `db4d20e`, `af70f85`, `36a2e56` are ancestors of master; gate numbers above are copied from the Task 1 report. Result: PASSED.
