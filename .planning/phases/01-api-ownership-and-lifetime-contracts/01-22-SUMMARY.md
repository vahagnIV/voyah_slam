---
phase: 01-api-ownership-and-lifetime-contracts
plan: 22
subsystem: input-validation-and-release-0.9.2
tags: [image, orb-extractor, argument-validation, std-invalid-argument, std-out-of-range, exception-policy, version-0.9.2, c++20]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "own-thread Stop, private ProcessPending, drained follower (01-21); Image fixes (01-19); extractor Workspace (01-15)"
provides:
  - "images::Image(parent, x, y, w, h) throws std::out_of_range, CopyTo throws std::invalid_argument, in every build"
  - "OrbFeatureExtractor constructor and both DetectAndExtract overloads throw std::invalid_argument; MinLevelSize() = 66; shared orb::ImagePyramid::LevelSize()"
  - "DESIGN section 7 argument-error policy; package version 0.9.2 (SOVERSION 0.9); TODO items for the deferred review findings"
affects: [01-23 release gates and merge, voyah_jni (error mapping)]

actuals:
  tokens: 8570
  tasks: 3
  commits: 5

tech-stack:
  added: []
  patterns:
    - "Argument values that can never succeed throw std::invalid_argument / out_of_range / length_error; UsageError is for call sequences"
    - "A pre-check and the code it protects share one helper (ImagePyramid::LevelSize), so they cannot disagree"
    - "Validation runs before any state (workspace) is touched"

key-files:
  created: []
  modified:
    - libs/common/include/voyahslam/images/image.h
    - libs/common/tests/images/image_test.cpp
    - libs/algs/src/features/orb_feature_extractor.cpp
    - libs/common/include/voyahslam/features/orb_feature_extractor.h
    - libs/common/include/voyahslam/features/orb/image_pyramid.h
    - libs/algs/src/features/orb/image_pyramid.cpp
    - libs/algs/tests/features/feature_extractor_levels_test.cpp
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CLAUDE.md
    - CMakeLists.txt
    - conanfile.py

key-decisions:
  - "Argument-value errors use the standard std::logic_error subclasses, UsageError stays for call sequences (DESIGN section 7)"
  - "Image At/Set/operator[]/operator T/operator=(float) stay unchecked: per-pixel paths of a header compiled into the -O0 apps"
  - "The ORB size check loops over every scale factor with LevelSize (the review's suggestion looked only at the last level)"
  - "OrbFeatureExtractor::MinLevelSize() is a public static constexpr (66) so callers and the cpp static_assert share the number"
  - "Patch release 0.9.2, SOVERSION stays 0.9"

patterns-established:
  - "Rejected calls leave caller-owned scratch (Workspace) untouched, pinned by a bit-exact before/after test"

requirements-completed: [OWN-05, OWN-07]

coverage:
  - id: D1
    description: "Image view outside its parent throws std::out_of_range in every build; views on the borders and empty views inside the bounds are accepted; the bound is the immediate parent"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.ViewOutsideParentThrows"
        status: pass
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.ViewsInsideTheBoundsAreAccepted"
        status: pass
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.ViewOfAnEmptyImage"
        status: pass
    human_judgment: false
  - id: D2
    description: "Image::CopyTo into a target of another size throws std::invalid_argument before copying a byte, also for an empty source; ImageTest clean under AddressSanitizer"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.CopyToSizeMismatchThrows"
        status: pass
      - kind: other
        ref: "cmake-build-asan/libs/common/tests/voyah_common_tests --gtest_filter='ImageTest.*' (18 passed)"
        status: pass
    human_judgment: false
  - id: D3
    description: "OrbFeatureExtractor::DetectAndExtract throws std::invalid_argument ('too small') for any pyramid level below 66 px (224x224, 234x480, 480x234, empty), accepts 235x235, and a rejected call leaves the Workspace usable with bit-identical later output"
    requirement: OWN-05
    verification:
      - kind: unit
        ref: "libs/algs/tests/features/feature_extractor_levels_test.cpp#OrbInputTest.ImageTooSmallForThePyramidThrows"
        status: pass
      - kind: unit
        ref: "libs/algs/tests/features/feature_extractor_levels_test.cpp#OrbInputTest.SmallestAcceptedImageExtracts"
        status: pass
      - kind: unit
        ref: "libs/algs/tests/features/feature_extractor_levels_test.cpp#OrbInputTest.RejectedImageLeavesTheWorkspaceUsable"
        status: pass
    human_judgment: false
  - id: D4
    description: "OrbFeatureExtractor constructor throws std::invalid_argument for levels 0 and for scale 1.0, 0.5, 0.0, -1.2, NaN, +infinity before any member is computed"
    requirement: OWN-05
    verification:
      - kind: unit
        ref: "libs/algs/tests/features/feature_extractor_levels_test.cpp#OrbInputTest.InvalidConstructorArgumentsThrow"
        status: pass
    human_judgment: false
  - id: D5
    description: "ORB output and the binocular app are unchanged: golden digests not re-recorded, corridor1 40 frames 39 FrameTracked / 3 Frames / 777 points before and after; check_all 866 of 866 passed (1 skipped vocabulary test)"
    verification:
      - kind: integration
        ref: "voyah_algs_dataset_tests --gtest_filter='OrbExtractorGoldenTest.*' (passed, not skipped)"
        status: pass
      - kind: other
        ref: "cmake --build cmake-build-debug/build/Debug --target check_all (100% tests passed, 0 failed out of 866)"
        status: pass
    human_judgment: false
  - id: D6
    description: "DESIGN, TODO, ROADMAP, README, CLAUDE.md and the package version describe and carry 0.9.2"
    verification: []
    human_judgment: true
    rationale: "Documentation wording and the choice of deferred-finding text are judged by reading; the verify greps only prove the sections, versions and counts exist"

duration: 13min
completed: 2026-10-21
status: complete
plan_head_before: 1c0ae007779f35e1faa398a4c3507439b2e76c84
plan_head_after: 55aaeb3606bc88c42905844afdc1056dc3e2827d
commits: 5
---

# Phase 1 Plan 22: Checked Image views and CopyTo, validated ORB extractor, documentation and version 0.9.2 Summary

**`images::Image` views and `CopyTo` and the `OrbFeatureExtractor` constructor and `DetectAndExtract` now throw `std::out_of_range` / `std::invalid_argument` in every build (no heap overflow, no SIGFPE on a 224x224 image), the argument-error policy is recorded in DESIGN section 7, and the package is versioned 0.9.2 on `fix/phase1-gaps`.**

## Performance

- **Duration:** about 13 min
- **Started:** 2026-10-21T15:22:51Z
- **Completed:** 2026-10-21T15:36Z
- **Tasks:** 3 (Task 1 tracer and Task 2 TDD: RED and GREEN commits each)
- **Files modified:** 14 (473 insertions, 24 deletions)
- **Branch:** `fix/phase1-gaps` (nothing merged or pushed)

## Accomplishments

- CR-02 closed: `Image(parent, x, y, w, h)` computes `x + w` and `y + h` in `size_t` through a private `CheckedViewOffset` helper called in the member-initializer list, so no object is built for a bad rectangle; the bound is the immediate parent. `CopyTo` throws `std::invalid_argument` naming both sizes before the empty-image early return, so an empty-to-nonempty copy is a mismatch and empty-to-empty stays a no-op. The two `assert`s are gone; the per-pixel accessors are untouched and the class comment says why.
- CR-01 closed: `orb::ImagePyramid::LevelSize(size, scale)` holds the exact arithmetic of `Initialize` (multiply by `1 / scale`, `std::round`, cast to `unsigned`) and `Initialize` calls it; `OrbFeatureExtractor::CheckImageSize` (first statement of the workspace overload, so also of the allocating one) takes the minimum over every scale factor and throws a "too small" `std::invalid_argument` naming image size, level count, scale factor and the 66 px minimum. The constructor validates in the first member initializer (`ValidatedScaleFactor`): levels 0, scale not finite or not above 1. The `assert(scale_factor != 0)` is deleted; the asserts in `ComputePointsPerLevel` stay as internal invariants.
- 0.9.2 documentation: DESIGN section 7 "Argument-error policy" with the list of throw sites, the "Checked views and copies" and "Input validation" bullets, four 2026-10-21 decisions rows (ORB validation, Image checks, exception policy, package version 0.9.2 with every behaviour change of plans 01-21 and 01-22), section 10 status and a test-suite entry; TODO (CR-01/CR-02 ticked, portrait-image observation, nine deferred review findings); ROADMAP sentence; version 0.9.2 in `CMakeLists.txt`, `conanfile.py`, README (three places) and the DESIGN layout list with SOVERSION 0.9 unchanged; CLAUDE.md counts re-measured.

## TDD record

**Task 1 RED** (`8dfeb89`, log `$LOGS/01-22-t1-red.log` in the executor scratchpad; each new test run alone, header untouched):

- `ImageTest.ViewOutsideParentThrows`: seven `Expected: ... throws an exception of type std::out_of_range. Actual: it throws nothing.` failures (lines 228-239 of the test, one per rectangle including the 0xFFFFFFFF starts and the view of a view), `[  FAILED  ]`. Planned failure.
- `ImageTest.ViewOfAnEmptyImage`: `Expected: (Image<uint8_t, 3>(empty, 0, 0, 1, 1)) throws ... std::out_of_range. Actual: it throws nothing.` Planned failure.
- `ImageTest.CopyToSizeMismatchThrows`: `image.h:170: ... CopyTo ...: Assertion 'other.Height() == Height()' failed.`, exit 134. The assert-only guard this task replaces.
- `ImageTest.ViewsInsideTheBoundsAreAccepted` passed in RED, as intended: it pins the cases that must keep working (border view, whole parent, empty view on the far corner, view of a view on its borders) and is the regression guard for the GREEN change.

Semantic assessment: all three failing tests executed and failed on the planned assertion; none is a build, discovery or fixture failure. One earlier build attempt of the test file failed to compile (a lambda without capture of `kSentinel`); fixed before any RED run, so no RED evidence came from it.

**Task 1 GREEN** (`edec200`): `ImageTest.*` 18 of 18 pass in Debug and under AddressSanitizer; `OrbExtractorGoldenTest.Corridor1OutputUnchanged` passes (not skipped).

**Task 2 RED** (`cc60c40`, log `$LOGS/01-22-t2-red.log`, each test alone):

- `ImageTooSmallForThePyramidThrows`, `SmallestAcceptedImageExtracts` (on its 65x66 case) and `RejectedImageLeavesTheWorkspaceUsable`: `orb_feature_extractor.cpp:562: ... ComputePointsPerLevel ...: Assertion ... failed`, exit 134 (the `assert(n_cols > 0)` that in a release build is the SIGFPE).
- `InvalidConstructorArgumentsThrow`: segmentation fault, exit 139 (levels 0 writes `scale_factors[0]` of an empty vector). Planned failure.

**Task 2 GREEN** (`9bf1c92`): `OrbInputTest.*` 4 of 4, `OrbLevelsTest`, `OrbDescriptorTest`, `OrbExtractorWorkspaceTest` (12 tests together) pass; golden test passes without re-recording. `OrbInputTest.*` also pass in `cmake-build-asan` (built `voyah_algs_tests` there for the first time: no AddressSanitizer report, no g2o abort, exit 0).

**REFACTOR:** none needed.

## Task Commits

1. **Task 1 RED:** `8dfeb89` (test) failing tests for Image views outside their parent and CopyTo size mismatch (CR-02)
2. **Task 1 GREEN:** `edec200` (fix) Image views and CopyTo are checked in every build (CR-02)
3. **Task 2 RED:** `cc60c40` (test) failing tests for ORB extraction on images too small for the pyramid and invalid configurations (CR-01)
4. **Task 2 GREEN:** `9bf1c92` (fix) OrbFeatureExtractor validates its configuration and the image size (CR-01)
5. **Task 3:** `55aaeb3` (docs) argument-error policy, CR-01/CR-02, deferred review findings; version 0.9.2

**Plan metadata:** the docs commits "summary" and "state and roadmap" after these five.

## Binocular app, before and after the header change (R2 lesson)

Command: `binocular_full_slam <corridor1> --rate=0 --max-frames=40`, Debug tree (the `-O0` app).

| Run | `[Mapper] [debug] FrameTracked` | `map:` line | mean load / mean extract |
|---|---|---|---|
| before the header change (`bino_before_image.log`) | 39 | `3 Frames, 777 points` | 19.7 / 203.8 ms |
| after (`bino_after_image.log`, final) | 39 | `3 Frames, 777 points` | 23.9 / 251.1 ms |
| after, three more runs | 39 | `3 Frames, 777 points` | extract 249.6 / 246.0 / 258.0 ms |
| OLD header restored, three runs, same later series | 39 | `3 Frames, 777 points` | extract 298.5 / 288.8 / 294.6 ms |

The bar (at least 36 FrameTracked and at least 3 Frames; baseline 39 / 3 / 777) is met and unchanged. The mean extract time moved from 204 to about 250 ms between the first and the later runs, which looked like a regression, so the same series was repeated with the old header restored (rebuild, three runs): 289 to 298 ms, i.e. slower than the new header. The machine load average was 4 to 6 during the later runs (uptime), so the difference is background load and not the header. No `terminate called` in any log; every run exited 0.

## Verification results

- `cmake --build cmake-build-debug/build/Debug --target check_all`: 100% tests passed, 0 failed of 866 (the one skipped test is the vocabulary test without ORBvoc). The full Debug tree (all libraries, examples, tools, benchmarks) builds with 0 warnings.
- `ctest -N` per label, Debug tree after building every test target: `unit` 825, `integration` 40, all 866 (1 `vocabulary`). Start of the plan was 817 / 858; the 8 new tests (4 `ImageTest`, 4 `OrbInputTest`) are all `unit`. CLAUDE.md now says 825 and 866; DESIGN section 10 quotes the same numbers.
- Task 1 verify commands: ImageTest 18 of 18 (Debug), golden passed (not skipped), ASan `ImageTest.*` 18 of 18 with no sanitizer report, binocular check (39 FrameTracked, `map: 3 Frames`, no terminate).
- Task 2 verify commands: all four `OrbInputTest` cases and the other ORB suites pass, golden passed, ASan `OrbInputTest.*` clean. Acceptance greps: 3 `std::invalid_argument` in `orb_feature_extractor.cpp`, `LevelSize` used in `image_pyramid.cpp` and the extractor, 0 `assert(scale_factor`.
- Task 3 verify commands: all greps pass (`VERSION 0.9.2`, `version = "0.9.2"`, README references only `voyahslam/0.9.2`, README soname example still `.so.0.9`, "Package version 0.9.2", CR-01, CR-02, `std::out_of_range`, WR-02), then `check_all` as above.
- Known flaky tests (DuplicatesFusedInLongSequence, StereoMapGrowsWhileLeavingInitialView, MonoRelocalizesAfterBlackout, StereoRelocalizesAfterBlackout) did not fail in the `check_all` run.

## Decisions Made

- The review's CR-01 fix looked at the last scale factor only; the check loops over all of them (equal result for a validated increasing configuration, no assumption about order).
- `OrbFeatureExtractor::MinLevelSize()` is a public `static constexpr` (66) rather than a file-local constant, so the header documents the number the exceptions mention; the `.cpp` has `static_assert(PATCH_SIZE + FAST_CELL_SIZE == MinLevelSize())` so the constants cannot drift apart.
- `ImagePyramid::LevelSize` takes `size_t` and recomputes `1. / scale_factor` per call, the same double operations `Initialize` performed on the stored `inv_scale_factors_`, so the golden digests are unchanged (verified by the golden test, not re-recorded).
- TODO: IN-03 is not listed as a new item because the existing `MapViewer` hardening item already tracks "`Start()` thread-creation failure leaves the state stuck"; the new section says so. The hook-setter item already existed from plan 01-21.

## Deviations from Plan

None - plan executed exactly as written. Small additions inside the plan's scope: the public `MinLevelSize()` accessor and the `static_assert` named under Decisions, and `<algorithm>`, `<limits>`, `<cmath>`, `<stdexcept>`, `<string>` includes in the extractor source for the new code.

## Issues Encountered

- A first compile of the new `CopyToSizeMismatchThrows` test failed (`kSentinel` not captured by a `[]` lambda); changed to `[&]` before the RED run.
- The first `OrbInputTest` RED run showed `SmallestAcceptedImageExtracts` aborting; that is its 65x66 case (the 235x235 case is the first statement and the GREEN run shows it extracts), so the RED evidence is valid for the planned reason.

## Known Stubs

None.

## Threat Flags

None. T-01-39 (SIGFPE on small images), T-01-40 (levels 0 and bad scale factors), T-01-41 (out-of-bounds view and CopyTo) and T-01-42 (slower `-O0` apps) are mitigated as planned; T-01-SC accepted (no package added).

## Next Phase Readiness

- 01-23 runs the release gates (ThreadSanitizer, AddressSanitizer, the conan packages for 0.9.2) and the merge; nothing was merged or pushed here. Everything for the gates is committed on `fix/phase1-gaps`.
- Message for the binding is in the 01-23 plan: both new exceptions are `std::logic_error`s, map `std::out_of_range` before `std::logic_error` in `Guarded`.

## Self-Check: PASSED

Files verified present: the 14 modified files above. Commits verified in the branch history: `8dfeb89`, `edec200`, `cc60c40`, `9bf1c92`, `55aaeb3` (5 commits from `plan_head_before` 1c0ae00 to `plan_head_after` 55aaeb3).

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-21*
