---
phase: 01-api-ownership-and-lifetime-contracts
plan: 19
subsystem: images
tags: [image, views, stride, overflow, asan, tdd, release-r6, voyah-jni]

requires:
  - phase: 01-api-ownership-and-lifetime-contracts
    provides: "plan 01-18: R5 (0.8.0) merged and pushed; master 0f0b97a"
provides:
  - "images::Image fixed in place (D-20): empty default and moved-from state, row-wise view-correct Convert / operator*= / operator*, non-const Set, CopyTo with sizeof(T), overflow-checked size constructor, size_t offsets, documented aliasing and operator[]"
  - "ImageTest.* (14 tests) clean in Debug and under AddressSanitizer"
  - "Branch fix/image-views at version 0.9.0 (merge into master pending, release plan R6)"
affects: [voyah_jni, phase-01-R6-release]

actuals:
  tokens: 8531      # chars/4 over git diff 0f0b97a..acc0742 (34125 chars)
  tasks: 3
  commits: 5        # MEASURED: git rev-list --count 0f0b97a..acc0742 (before this SUMMARY commit)
plan_head_before: 0f0b97a32f50bc5c1f8b40f69227b46837fcaf63
plan_head_after: acc0742e7fd7da0bf4c3d4b69fa1d97e68ddf7ef
commits: 5

tech-stack:
  added: []
  patterns:
    - "Row-wise loops over a view (offset_ + y * stride_ * C, width_ * C elements) instead of flat loops over width * height"
    - "Allocation size computed in size_t with a divide-based overflow check, throw before new[]"
    - "Compile-time contract tests with a requires-expression (Set not callable on const Image)"

key-files:
  created:
    - libs/common/tests/images/image_test.cpp
  modified:
    - libs/common/include/voyahslam/images/image.h
    - libs/common/tests/CMakeLists.txt
    - docs/DESIGN.md
    - docs/TODO.md
    - docs/ROADMAP.md
    - README.md
    - CLAUDE.md
    - CMakeLists.txt
    - conanfile.py

key-decisions:
  - "Fix Image in place, no ImageView type (D-20)"
  - "operator[] 1x1 channel indexing kept and pinned by IndexChainMatchesAt; the single-row pixel-versus-row meaning has no caller and is a TODO item"
  - "SizeOverflowThrows checks the byte-count half with Image<float, 1>, not Image<uint8_t, 1>: a ~2^64-byte request is fatal under AddressSanitizer, while the array-new size check for float throws std::bad_array_new_length before any allocation"
  - "Empty Convert / operator* return a default (null-buffer) image instead of allocating a zero-element buffer"

patterns-established:
  - "Tracer task commits as RED test then GREEN feat, like any TDD task"

requirements-completed: [OWN-07]

coverage:
  - id: D1
    description: "A default-constructed Image is empty (sizes 0, null data) and so is a moved-from Image"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.DefaultIsEmpty, ImageTest.MovedFromIsEmpty"
        status: pass
    human_judgment: false
  - id: D2
    description: "Convert, operator*= and operator* on a sub-image view read and write only the view's pixels, row by row, all channels, including a view touching the parent's right and bottom borders; empty images stay empty"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.ConvertOfViewReadsOnlyTheView, ConvertOfBorderViewIncludesLastRowAndColumn, ConvertOfEmptyIsEmpty, MultiplyAssignOnViewChangesOnlyTheView, MultiplyOfViewReturnsScaledView, MultiplyOfEmptyIsEmptyAndMultiplyAssignLeavesItEmpty"
        status: pass
    human_judgment: false
  - id: D3
    description: "Set is non-callable on a const Image; CopyTo of strided float views copies every element; CopyTo between empty images is a no-op"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.SetOnConstDoesNotCompile, CopyToFloatViewCopiesWholeRows, CopyToBetweenEmptyImagesIsANoOp"
        status: pass
    human_judgment: false
  - id: D4
    description: "Image(width, height) throws std::length_error on size_t overflow before allocating; view offsets are size_t end to end"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.SizeOverflowThrows, ImageTest.OffsetIsSizeT"
        status: pass
    human_judgment: false
  - id: D5
    description: "img[y][x][c] equals At(y, x, c) on a multi-row 3-channel view"
    requirement: OWN-07
    verification:
      - kind: unit
        ref: "libs/common/tests/images/image_test.cpp#ImageTest.IndexChainMatchesAt"
        status: pass
    human_judgment: false
  - id: D6
    description: "No slowdown or behaviour change of the -O0 example apps (header compiled into them) and unchanged ORB output"
    verification:
      - kind: other
        ref: "binocular_full_slam corridor1 --rate=0 --max-frames=40: 39 FrameTracked, 390 OptimizePose, 3 Frames, 777 points before and after; OrbExtractorGoldenTest.Corridor1OutputUnchanged passes"
        status: pass
    human_judgment: false
  - id: D7
    description: "DESIGN Images section, decisions rows (audit #9, package 0.9.0), TODO #9 ticked with the single-row operator[] item open, ROADMAP API hardening complete, version 0.9.0"
    verification:
      - kind: other
        ref: "grep checks of the 01-19 Task 3 verify command; check_all 852 tests pass"
        status: pass
    human_judgment: true
    rationale: "Documentation wording is not asserted by any test"

duration: 45min
completed: 2026-10-15
status: complete
---

# Phase 1 Plan 19: Image views, overflow guard and 0.9.0 Summary

**images::Image fixed in place: empty default and moved-from state, row-wise stride-correct Convert / operator*= / operator*, non-const Set, CopyTo with sizeof(T), size_t-checked size constructor throwing std::length_error, size_t offsets; 14 ImageTest cases clean under ASan, version 0.9.0**

## Performance

- **Duration:** about 45 min
- **Started:** 2026-10-14T20:50Z (approximate; the start time was not recorded)
- **Completed:** 2026-10-14T21:40Z by the system clock (dated 2026-10-15 in the project docs)
- **Tasks:** 3
- **Files modified:** 10 (one created)

## Accomplishments

- Every `Image` operation respects views. Before, `Convert` ignored the offset, stride and channels, and `operator*=` / `operator*` ran over the first `width * height * C` elements of the buffer, so a view's operations touched the wrong pixels and (`*=`) could write outside the view.
- Closed a heap-overflow path: `Image(width, height)` used unsigned arithmetic for `width * height * C`, so caller-supplied sizes (JNI) could wrap the allocation while a later full-size copy used the real size. It now throws `std::length_error` before allocating.
- Extra defects found by research fixed: `CopyTo` strided branch copied `width * C` bytes instead of `* sizeof(T)` (float views copied a quarter of each row); `<cstring>` was missing; view offsets were truncated to `unsigned` in the private constructor and the public constructor's arithmetic, and `Offset()` returned `unsigned`; moved-from images kept stale sizes; `Set` was `const`.
- Pinned `operator[]` (1x1 view selects the channel, so `img[y][x][c] == At(y, x, c)`) with a test and documented its shape-dependence; the single-row pixel-versus-row meaning is a TODO item.
- Binocular corridor1 run (40 frames, `--rate=0`, Debug apps at -O0) is identical before and after: 39 FrameTracked, 390 OptimizePose, "map: 3 Frames, 777 points"; mean extract 250 ms before and 213 ms after (run to run noise, no slowdown); total 11.2 s before and 9.7 s after. Monocular: 0 FrameTracked and "map: 0 Frames, 0 points" (tracker WaitingForMap, mapper Initializing) both before and after, so unchanged (this app does not initialise within 40 frames on this dataset at the base either).
- `OrbExtractorGoldenTest.Corridor1OutputUnchanged` passes without re-recording (not skipped).

## Task Commits

1. **Task 1: Tracer, empty default image and view-correct Convert**
   - RED `4712390` (test): DefaultIsEmpty failed with sizes read as 0xABABABAB, the Convert tests with wrong values
   - GREEN `ac0db68` (feat): in-class zero initialisers, row-wise `Convert`, class comment (aliasing)
   - Tracer gate: automated `<verify>` only, `HUMAN_VERIFY_MODE` end-of-phase: `<verify>` re-run green, expanded
2. **Task 2: operator*=, operator*, Set, CopyTo, overflow guard, moved-from, operator[]**
   - RED `72a71e2` (test): MultiplyAssignOnViewChangesOnlyTheView, MultiplyOfViewReturnsScaledView, CopyToFloatViewCopiesWholeRows, SizeOverflowThrows, MovedFromIsEmpty failed on the assertions; the empty-image and IndexChain tests pin behaviour that already held
   - GREEN `4966c89` (feat): the header fixes plus the two compile-time tests (OffsetIsSizeT, SetOnConstDoesNotCompile), which cannot be expressed as a runtime-failing RED
3. **Task 3: Document R6 and bump to 0.9.0** - `acc0742` (docs)

**Plan metadata:** the SUMMARY commit that follows this file (docs: complete plan).

## TDD Gate Compliance

RED evidence (Debug tree, `voyah_common_tests --gtest_filter='ImageTest.*'`, exit non-zero): Task 1 RED showed `DefaultIsEmpty` failing on `Width()` equal to 2880154539 (0xABABABAB, storage pre-filled with garbage so uninitialised members are exposed) and the Convert tests on wrong values (then a crash on the empty-default path); Task 2 RED (with a temporary `#include <cstring>` at the top of the test, because the header did not include it, removed in GREEN) showed 5 FAILED of 12: the five named above, each on an assertion for the planned behaviour (not a compile or fixture error). Semantic assessment: each failing test executed and failed on the intended defect. The classifier `gsd_run check tdd-red-evidence` was not run (gtest console output is not one of its supported report formats).

## Files Created/Modified

- `libs/common/include/voyahslam/images/image.h` - the fixes above; class comment on aliasing, empty state, overflow and `size_t` offsets
- `libs/common/tests/images/image_test.cpp` - 14 `ImageTest` cases
- `libs/common/tests/CMakeLists.txt` - `images/image_test.cpp` in `voyah_common_tests`
- `docs/DESIGN.md` - new "Images" section, two decisions rows, next-steps and test-count entry, version strings
- `docs/TODO.md` - #9 ticked, single-row `operator[]` item open
- `docs/ROADMAP.md` - API hardening list complete
- `README.md`, `CMakeLists.txt`, `conanfile.py` - version 0.9.0
- `CLAUDE.md` - test counts 812 unit / 852 total

## Decisions Made

- Fix in place, no `ImageView` type (D-20), as planned.
- The byte-count half of the overflow check uses `Image<float, 1>` (array-new size check, `std::bad_alloc` family) rather than `Image<uint8_t, 1>`: a request of about 2^64 bytes would abort under AddressSanitizer instead of throwing, and the plan requires the test to pass under ASan.
- Empty `Convert` / `operator*` return a default image (null buffer) so no zero-element buffer is allocated and no memory is touched.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical functionality] Extra tests for plan must-haves that the plan's test list did not name**
- **Found during:** Task 2
- **Issue:** The truths "Edge (empty): operator* and operator*= on an empty image ..., CopyTo between two empty images is a no-op" had no test in the plan's behavior list.
- **Fix:** Added `MultiplyOfEmptyIsEmptyAndMultiplyAssignLeavesItEmpty` and `CopyToBetweenEmptyImagesIsANoOp`. ImageTest therefore has 14 cases, not the 12 the acceptance criterion counts (4 + 8 named); all pass in Debug and ASan.
- **Files modified:** libs/common/tests/images/image_test.cpp
- **Committed in:** 72a71e2

**2. [Rule 1 - Bug] Header lacked `<cstring>`, `<limits>`, `<stdexcept>`, `<utility>`, `<cstdint>`, `<cstddef>`; `memcpy` unqualified**
- **Found during:** Task 2 RED (the test TU failed to compile `CopyTo` when it included the header first)
- **Fix:** includes added to the header (as the plan asks for `<cstring>`, `<stdexcept>`, `<limits>`), `std::memcpy`.
- **Committed in:** 4966c89

### Plan-wording notes (no code impact)

- Task 3 acceptance `git grep -n '0\.8\.0' -- CMakeLists.txt conanfile.py README.md prints nothing` is not literally satisfiable: `conanfile.py` line 75 requires `yaml-cpp/0.8.0`, a third-party dependency version, unrelated to the package version. All package-version occurrences are 0.9.0.
- Task 2 verify names the binary `voyah_algs_tests` for the golden test; the golden test lives in `voyah_algs_dataset_tests` (run there, passes).

**Total deviations:** 2 auto-fixed (1 missing critical, 1 bug), 2 plan-wording notes.
**Impact on plan:** No scope creep.

## Issues Encountered

None blocking. `check_all` ran green on the first run (852 tests, 0 failed, the vocabulary test skipped without ORBvoc as before; none of the known flaky tests failed).

## Known Stubs

None.

## Threat Flags

None. T-01-31 (heap overflow from wrapped size) mitigated by the `size_t` guard and `SizeOverflowThrows`; T-01-32 by row-wise operations, `sizeof(T)` in `CopyTo` and the view tests under ASan; T-01-33 by the zero initialisers and `DefaultIsEmpty`.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Release plan R6 (merge fix/image-views into master, gates, Conan packages for 0.9.0) is next; the merge is the user's decision and was not done here.
- voyah_jni note: `Image(width, height)` throws `std::length_error` on overflow, `Offset()` returns `size_t`, and `Set` needs a non-const image.

## Self-Check: PASSED

- Created files exist: image.h, image_test.cpp.
- Commits `4712390`, `ac0db68`, `72a71e2`, `4966c89`, `acc0742` are ancestors of HEAD on `fix/image-views`.
- Acceptance greps pass (`std::length_error`, `sizeof(T)` on both CopyTo memcpy lines, `size_t Offset() const`, `unsigned width_ = 0`, test registered in CMake). ImageTest 14/14 in Debug and under ASan; golden test passes (not skipped); `check_all` 852 tests, 0 failed.

---
*Phase: 01-api-ownership-and-lifetime-contracts*
*Completed: 2026-10-15*
