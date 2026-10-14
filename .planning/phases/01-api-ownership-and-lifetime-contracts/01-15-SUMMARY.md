---
phase: 01-api-ownership-and-lifetime-contracts
plan: 15
subsystem: features
tags: [orb, extractor, workspace, thread-safety, tsan, ownership-audit, release, versioning]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-14: R3 merged (0.6.0), master == origin/master == 0122fd7"
provides:
  - "OrbFeatureExtractor::Workspace and DetectAndExtract(image, n, Workspace& out_workspace) const; the extractor has no mutable member and can be shared by threads"
  - "orb::ImagePyramid: const operator[], Width(), Height(), Fits(), re-entrant Initialize()"
  - "Allocating DetectAndExtract(image, n) overload kept (tests, one-offs)"
  - "Mono example holds one Workspace, stereo example two"
  - "DESIGN section 'ORB Extractor Workspace' plus the rows 'Ownership audit #7 implemented' and 'Package version 0.7.0'; TODO #7 ticked; version 0.7.0"
affects: [voyah_jni, phase-01-R4-merge, EXTR-01]

actuals:
  tokens: 9400    # chars/4 over the diff of the 4 commits (37690 chars incl. docs and the new test file)
  tasks: 3
  commits: 4      # MEASURED: git rev-list --count 0122fd7..HEAD before this SUMMARY commit (test, feat, feat, docs)
plan_head_before: 0122fd7c20c634c4e7266b4a57116d1ca519dae3
plan_head_after: db4d20e87509528da5800aa8b335d45bcb20ba31
commits: 4

tech-stack:
  added: []
  patterns:
    - "Per-thread scratch is a caller-owned type passed by non-const reference (out_workspace); the algorithm object stays immutable and shareable"
    - "A reusable scratch checks that it fits the call (Fits: size, level count, scale factors) and re-initialises instead of asserting"

key-files:
  created:
    - libs/algs/tests/features/orb_extractor_workspace_test.cpp
  modified:
    - libs/common/include/voyahslam/features/orb_feature_extractor.h
    - libs/common/include/voyahslam/features/orb/image_pyramid.h
    - libs/algs/src/features/orb_feature_extractor.cpp
    - libs/algs/src/features/orb/image_pyramid.cpp
    - libs/algs/tests/CMakeLists.txt
    - examples/full_slam/monocular_full_slam.cpp
    - examples/full_slam/binocular_full_slam.cpp
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CLAUDE.md
    - CMakeLists.txt
    - conanfile.py

key-decisions:
  - "Workspace chosen over a non-const DetectAndExtract and over thread_local storage (CONTEXT D-15): the caller already owns one thread per extraction loop, so it owns the scratch"
  - "Workspace re-initialises when the scale factors differ too, not only on size or level count (ImagePyramid::Fits); same depth with another scale factor would otherwise reuse wrongly sized levels"

patterns-established:
  - "Ownership audit #7 pattern: const-correct shared algorithm object, mutable state in a documented not-thread-safe per-thread workspace"

requirements-completed: [OWN-05]

coverage:
  - id: D1
    description: "DetectAndExtract takes a caller-owned Workspace; the extractor has no mutable member and two threads on one extractor give the single-thread result"
    requirement: OWN-05
    verification:
      - kind: unit
        ref: "voyah_algs_tests --gtest_filter=OrbExtractorWorkspaceTest.ConcurrentCallsEqualSingleThreadResult (Debug and TSan, no ThreadSanitizer warning)"
        status: pass
    human_judgment: false
  - id: D2
    description: "A Workspace reused across frames, image sizes and level counts equals a fresh Workspace; the allocating overload equals the workspace overload"
    requirement: OWN-05
    verification:
      - kind: unit
        ref: "voyah_algs_tests --gtest_filter=OrbExtractorWorkspaceTest.*"
        status: pass
    human_judgment: false
  - id: D3
    description: "Extractor output is bit-identical to before (golden digests not re-recorded, not skipped)"
    requirement: OWN-05
    verification:
      - kind: integration
        ref: "voyah_algs_dataset_tests --gtest_filter=OrbExtractorGoldenTest.Corridor1OutputUnchanged"
        status: pass
    human_judgment: false
  - id: D4
    description: "Examples reuse one Workspace (mono) and two (stereo); tracking and extraction cost unchanged"
    verification:
      - kind: other
        ref: "binocular_full_slam / monocular_full_slam corridor1 --rate=0 --max-frames=40, before and after (numbers in this SUMMARY)"
        status: pass
    human_judgment: false
  - id: D5
    description: "DESIGN, TODO, ROADMAP, README, CLAUDE.md updated; version 0.7.0; check_all green"
    verification:
      - kind: integration
        ref: "cmake --build cmake-build-debug/build/Debug --target check_all (832 tests, 0 failed)"
        status: pass
    human_judgment: false

duration: 35min
completed: 2026-10-14
status: complete
---

# Phase 1 Plan 15: R4 extractor Workspace (audit #7, 0.7.0) Summary

**OrbFeatureExtractor made immutable: the image pyramid moved from a `mutable` member into a caller-owned `Workspace`, so one extractor is safe to share between threads (TSan clean), with bit-identical features and unchanged tracking, version 0.7.0.**

## Performance

- **Duration:** about 35 min
- **Completed:** 2026-10-14T17:40Z
- **Tasks:** 3 (1 tracer with TDD, 2 auto)
- **Files modified:** 15 (one new test file)

## Accomplishments

- `OrbFeatureExtractor::Workspace` and `DetectAndExtract(image, n, Workspace& out_workspace) const`; the `mutable` pyramid member is gone and `ComputeKeyPointsOctTree` / `ComputePointsPerLevel` take a `const orb::ImagePyramid&`.
- The Workspace is documented as not thread-safe, one per extraction thread; it re-initialises on a new image size, level count or scale factor and returns exactly what a fresh Workspace returns.
- The allocating `DetectAndExtract(image, n)` overload stays; its comment says a loop should reuse a Workspace.
- Both examples hold Workspaces (mono one, stereo `left_workspace` and `right_workspace`).
- Docs and version: DESIGN section "ORB Extractor Workspace", decisions rows "Ownership audit #7 implemented" and "Package version 0.7.0", TODO #7 ticked, ROADMAP, README, CLAUDE.md counts, `CMakeLists.txt` and `conanfile.py` at 0.7.0.

## TDD Gate Compliance

- **RED (`c548b1d`):** `OrbExtractorWorkspaceTest.*` (5 tests) plus a compile-only stub of the Workspace overload that forwards to the old mutable-pyramid path. Run one test at a time: `ConcurrentCallsEqualSingleThreadResult` and `WorkspaceReinitialisesOnNewImageSize` failed on the planned behaviour (abort at `ImagePyramid::Build` assertion `image_pyramid_[0].Width() == image.Width()`: the shared pyramid cannot follow a size change, and two threads share it). `AllocatingOverloadEqualsWorkspaceOverload`, `ReusedWorkspaceEqualsFreshWorkspace` and `WorkspaceReinitialisesForAnotherLevelCount` passed under the stub (each extractor object had its own pyramid there); they guard the GREEN change against regressions. Semantic assessment: the two failing tests executed and failed on the intended reason (shared mutable pyramid), not on a build or fixture fault. The failures are assert aborts, not gtest assertion lines, so the `gsd check tdd-red-evidence` classifier was not used (plan type is `execute`, not `tdd`).
- **GREEN (`de00c27`):** the Workspace implementation; all 5 tests, `OrbLevelsTest.*`, `OrbDescriptorTest.*` and the golden test pass; TSan run clean (0 ThreadSanitizer warnings).
- **REFACTOR:** none.

## Task Commits

1. **Task 1: tracer, Workspace overload** - `c548b1d` (test, RED), `de00c27` (feat, GREEN)
2. **Task 2: examples hold one Workspace per extraction thread** - `69eed20` (feat)
3. **Task 3: document R4 and bump to 0.7.0** - `db4d20e` (docs)

**Plan metadata:** the commit that adds this SUMMARY, STATE.md and ROADMAP.md.

## Behaviour and performance check (corridor1, `--rate=0 --max-frames=40`, Debug tree with optimized libs)

| | Before (branch base 0122fd7) | After (db4d20e) |
|---|---|---|
| binocular `[Mapper] [debug] FrameTracked` lines | 39 | 39 |
| binocular final map line | `map: 3 Frames, 777 points` | `map: 3 Frames, 777 points` |
| binocular tracker state | Tracking | Tracking |
| binocular mean extract (left+right, per frame) | 257.0 ms | 262.5, 245.6, 252.3 ms (3 runs) |
| binocular features per frame | 2977.2 | 2977.2 |
| monocular FrameTracked | 0 | 0 |
| monocular final map line | `map: 0 Frames, 0 points` (WaitingForMap) | identical |
| monocular mean extract | 129.9 ms | 130.5 ms |
| monocular features per frame | 1485.38 | 1485.38 |

The binocular result stays above the required floor (>= 36 FrameTracked, >= 3 Frames). Tracking is byte-for-byte the same as before (same feature counts, same map). Mono does not initialize within the first 40 frames on either side (two-view reconstruction rejected for low parallax); that is the baseline, not a change. Per-frame extraction cost is within run-to-run noise (-4.5% to +2.1%); reusing the Workspace removes the pyramid allocation, a per-call allocation is not measurable.

## Verification run

- `OrbExtractorWorkspaceTest.*:OrbLevelsTest.*:OrbDescriptorTest.*`: 8 passed. `OrbExtractorGoldenTest.Corridor1OutputUnchanged`: passed, not skipped (run on the dataset); `git diff master -- libs/algs/tests/features/data` is empty.
- TSan (`cmake-build-tsan`, `setarch -R`, `OrbExtractorWorkspaceTest.*`): 5 passed, 0 `WARNING: ThreadSanitizer`.
- `check_all` (run twice, the second after the 0.7.0 reconfigure): 832 tests, 0 failed (792 unit, 39 integration, 1 vocabulary skipped without ORBvoc). None of the known flaky tests failed.
- Acceptance: `git grep mutable` in the extractor header prints nothing; `Workspace& out_workspace` is declared; `0.6.0` is gone from `CMakeLists.txt`, `conanfile.py` and `README.md`.

## Decisions Made

- Workspace (caller-owned scratch) chosen over a non-const method and over `thread_local` storage, as planned (D-15).
- `ImagePyramid::Fits` also compares the scale factors, so a Workspace moved between two extractors with the same level count but another scale factor re-initialises (a gap in the plan's "size or level count" wording).

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical functionality] Re-initialise on a different scale factor**
- **Found during:** Task 1 (GREEN)
- **Issue:** The plan re-initialises on image size or level count only. Two extractors with the same level count and different scale factors have differently sized pyramid levels; reusing one Workspace would read wrongly sized levels (the same out-of-bounds hazard as T-01-26).
- **Fix:** `ImagePyramid::Fits(width, height, scale_factors)` compares size and every inverse scale factor; `DetectAndExtract` re-initialises when it is false.
- **Files modified:** `libs/common/include/voyahslam/features/orb/image_pyramid.h`, `libs/algs/src/features/orb/image_pyramid.cpp`, `libs/algs/src/features/orb_feature_extractor.cpp`
- **Committed in:** `de00c27`

**2. [Wording] Comment avoided the substring "mutable"**
- The header comment first said "immutable"; the acceptance criterion `git grep mutable` must print nothing, so the comment now reads "Nothing in the extractor changes after construction". Committed in `de00c27`.

---

**Total deviations:** 1 auto-fixed (Rule 2), 1 wording
**Impact on plan:** Fix is within the plan's threat T-01-26 mitigation; no scope creep.

## Issues Encountered

- One shell command of mine ended with a stray `git checkout -q master --`; the harness denied it and it ran nothing. Working tree and branch were verified unchanged afterwards.
- Not covered by this branch's tests (untouched, still compile): `libs/algs/benchmarks/fast_benchmark.cpp` and `fast_corridor1_test.cpp` use `ImagePyramid` directly through the unchanged `Initialize` / `Build` / `operator[]` API; the full Debug build and `check_all` compiled and passed them.

## Known Stubs

None.

## Threat Flags

None. T-01-25 (data race on the pyramid) is mitigated by the per-thread Workspace and proven by `ConcurrentCallsEqualSingleThreadResult` under TSan; T-01-26 by re-initialisation tests (size, level count) and `Fits`.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Branch `refactor/extractor-workspace` (4 commits plus the SUMMARY commit) is ready for the R4 release plan: not merged, not pushed. The voyah_jni binding should hold one `Workspace` per extraction thread.
- Remaining ownership-audit items: #8 (Logger), #9 (`images::Image`).
- EXTR-01 (FAST per level, blur buffers, border handling) is unchanged; the pyramid part of its "preallocate" clause is now done.

## Self-Check: PASSED

- Files: orb_extractor_workspace_test.cpp, orb_feature_extractor.h, image_pyramid.h/.cpp and both examples verified present in the branch history.
- Commits `c548b1d`, `de00c27`, `69eed20`, `db4d20e` are ancestors of HEAD (`git rev-list --count 0122fd7..HEAD` = 4).

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-14*
