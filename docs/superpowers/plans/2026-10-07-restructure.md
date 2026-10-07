# Restructure into Four Libraries Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split `voyahslam` into `voyah_common`, `voyah_events`, `voyah_algs` and `voyah_slam`, each with its own include root (same include paths) and test binary. Add `unit`/`integration` labels, `check`/`check_all` targets and an optimized test build. No behaviour change.

**Architecture:** `libs/<lib>/{include/voyahslam/…, src/…, tests/…}`. Include roots are visible only to the library and its dependents, so a wrong-direction include fails to compile. Move commits contain only `git mv`. `voyahslam` becomes an INTERFACE umbrella, and `voyahviz` links common + events only.

**Tech Stack:** C++20, CMake ≥ 3.22, Conan 2, GoogleTest (`gtest_discover_tests`), Eigen, spdlog, concurrentqueue, OpenCV/stb/g2o (PRIVATE).

**Spec:** `docs/superpowers/specs/2026-10-07-restructure-design.md`

## Global Constraints

- **Branch:** `feat/restructure`. Commit per step as stated. Never merge or push.
- **Unchanged:** no behaviour change, no `#include` path change, no namespace change.
- **Move commits** contain only `git mv`: `git show --stat` shows renames with 0 changed lines. CMake and content edits go in separate commits.
- **Dependencies** (PUBLIC unless noted):
  - common → Eigen, spdlog;
  - events → common, concurrentqueue;
  - algs → common, plus OpenCV, stb and g2o (all PRIVATE);
  - slam → common, events, algs.
- **No lower layer may see an upper layer's include root.**
- **Test count:** 458 in today's `voyahslam_tests` plus 43 in `voyahviz_tests`. The sum over all binaries stays exactly 501 (`ctest -N | tail -1`).
- **Build/test** (the cmake binary is `/home/vahagn/Programs/clion/clion-2025.1.3/bin/cmake/linux/x64/bin/cmake`):
  - Debug tree: `cmake-build-debug/build/Debug`, built with `cmake --build cmake-build-debug/build/Debug -j8`.
  - Release tree: `build/build/Release` must build.
  - Until Task 6 the full suite is `ctest --test-dir cmake-build-debug/build/Debug -j8 --timeout 120`; from Task 6 on, use `check_all`.
- **Header check:** clang-18 with libc++, `-std=c++20 -fsyntax-only`, using only the library's own include roots plus its dependencies' roots.
- **Docs:** `docs/DESIGN.md`, `CLAUDE.md`, `README.md`, `docs/ROADMAP.md` and `docs/TODO.md` change only in Task 8.

## Review Focus

1. **Explicit instantiations missing after the split.** An app or test gets an undefined reference to a template instantiated only in a library it no longer links.
   - Expected: every target links.
   - Test: Task 5 Step 4 builds `hello_voyahslam`, `monocular_full_slam`, `binocular_full_slam` and `voyahviz_tests`, in Debug and in Release.
2. **Silently dropped tests.** A test file that is not added to any binary disappears without failing anything.
   - Expected: the total stays 501.
   - Test: every task's verification step compares `ctest -N` against the number before the task.
3. **A transitive PUBLIC link or include directory leaking an upper root.** For example, `voyah_common` getting `src/` or another library's include directory.
   - Expected: the wrong-include experiment fails to compile.
   - Test: Task 8 Step 1.
4. **Static or singleton state duplicated across static libraries** (logger registry, pools). This would change runtime behaviour.
   - Expected: the smoke numbers match `DESIGN.md`.
   - Test: Task 8 Step 4.
5. **The optimized build defining `NDEBUG`** and silently removing invariant checks.
   - Expected: asserts stay active.
   - Test: Task 7 Step 2, a `voyah_common_tests` case `BuildConfigTest.AssertsEnabled` that checks `NDEBUG` is not defined in the Debug tree.

---

### Task 1: Baseline timings and the test-support target

**Files:**
- Create: `docs/superpowers/specs/2026-10-07-restructure-timings.md`, `tests/support/CMakeLists.txt` (or a block in `tests/CMakeLists.txt`)
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- **Produces:**
  - The target `voyah_test_support`: INTERFACE if it is header-only, STATIC if `tests/utils_test.cpp` holds helpers rather than tests. If that file holds TEST cases, they move with the library they test. Its include directories are `tests/` (for `support/*.h` and `utils_test.h`). It links the libraries its headers need, which today are just `voyahslam`.
  - The timings file, holding a table of per-test times for (a) today's Debug build and (b) the Debug build with `-O2 -g` added ad hoc, plus the **integration list**: every test above 1 s in (b), and every `SlamTest.*` test.

- [ ] **Step 1: Measure (a).** Run `ctest --test-dir cmake-build-debug/build/Debug -j1 --timeout 300 --output-junit <scratch>/a.xml`, using `-j1` for clean per-test times. Record the total and per-test times above 0.5 s.
- [ ] **Step 2: Measure (b).** Use a scratch build directory copied from the Debug configure with `CMAKE_CXX_FLAGS_DEBUG="-O2 -g"`, then run the same command. Record the same numbers. Delete the scratch tree afterwards.
- [ ] **Step 3: Write the timings file**, including the integration list (test names).
- [ ] **Step 4: Add `voyah_test_support`** and switch `voyahslam_tests` to use it. Run the full suite. Expected: 458 + 43 pass.
- [ ] **Step 5: Commit** `git commit -m "Restructure: baseline test timings; voyah_test_support target"`.

### Task 2: `voyah_common`

**Files:**
- Move (`git mv`, one commit) every common header from spec §2 into `libs/common/include/voyahslam/…`, with the same relative path.
  - Sources: `algebra.cpp`, `camera.cpp`, `logger.cpp`, `dummy_distortion_model.cpp`, `kannala_brandt_distortion_model.cpp`, `covisibility_index.cpp` → `libs/common/src/…`.
  - Tests:
    - `tests/{algebra/*, geometry/camera_test, geometry/rigid_transformation_test, geometry/distortion_models/*}`;
    - `tests/{memory/*, map/{id_types,map_point_storage,frame,observation_storage,covisibility_index}_test}`;
    - `tests/features/{monocular_measurement,image_features,stereo_rig,binocular_measurement}_test`;
    - `tests/test_voyahslam.cpp`, but only if the version functions move to common; otherwise it stays for Task 5;
    - all of the above go to `libs/common/tests/…`.
- Create: `libs/CMakeLists.txt`, `libs/common/CMakeLists.txt`, `libs/common/tests/CMakeLists.txt`
- Modify: root `CMakeLists.txt` (`add_subdirectory(libs)` before `src`), `src/CMakeLists.txt` (drop the moved sources and link `voyah_common` PUBLIC), `tests/CMakeLists.txt` (drop the moved tests)

**Interfaces:**
- **Produces:**
  - Target `voyah_common`, a STATIC library.
    - PUBLIC include directory: `libs/common/include`.
    - PUBLIC links: `Eigen3::Eigen`, `spdlog::spdlog`.
  - Test binary `voyah_common_tests`, linking `voyah_common`, `GTest::gtest_main` and, only if needed, `voyah_test_support`.
- **Extractor split:**
  - `orb_feature_extractor.h` and `sift_feature_extractor.h` move to common unchanged.
  - Their `.cpp` files stay in `src/` for now and move to algs in Task 4.
  - If an extractor header includes an algs-only header (for example `images/filter.h` or `orb/*`), move only the declarations it needs, per the spec's placement rule. Record each such move as a ruling.
- **Placement surprises:** the spec's placement rule applies. Report every file whose library differs from spec §2.

- [ ] **Step 1: Move commit.** Use only `git mv`. Expected: the build is broken at this commit. That is acceptable, because the next commit restores it, and the broken state is never left behind when the task ends.
- [ ] **Step 2: CMake commit.** Add the targets and the test binary, and make `voyahslam` link `voyah_common` PUBLIC. Build Debug and Release. Run the full suite. Expected: 501 tests (`ctest -N`), all pass.
- [ ] **Step 3: Header check.** Run clang over every `libs/common/include` header with only `libs/common/include` and the Eigen/spdlog include directories. Expected: clean.
- [ ] **Step 4: Commit** `git commit -m "Restructure: voyah_common"`. If you split it, the move commit is "Restructure: move voyah_common files".

### Task 3: `voyah_events`

**Files:**
- Move: `include/voyahslam/events/*` → `libs/events/include/voyahslam/events/`; `src/events/worker_thread.cpp` → `libs/events/src/events/`; `tests/events/*` → `libs/events/tests/events/`
- Create: `libs/events/CMakeLists.txt`, `libs/events/tests/CMakeLists.txt`
- Modify: `libs/CMakeLists.txt`, `src/CMakeLists.txt`, `tests/CMakeLists.txt`

**Interfaces:**
- **Produces:**
  - Target `voyah_events`, STATIC.
    - PUBLIC include directory: `libs/events/include`.
    - PUBLIC links: `voyah_common`, `${VOYAHSLAM_CONCURRENTQUEUE_TARGET}`.
  - Test binary `voyah_events_tests`.

- [ ] **Step 1: Move commit** (`git mv` only).
- [ ] **Step 2: CMake commit.** Build Debug and Release. Run the full suite. Expected: 501 tests, all pass.
- [ ] **Step 3: Header check** with only the common and events roots, plus the dependencies. Expected: clean.
- [ ] **Step 4: Commit** `git commit -m "Restructure: voyah_events"`.

### Task 4: `voyah_algs`

**Files:**
- **Move:** every algs header and source from spec §2 into `libs/algs/…`. That includes `src/features/orb/*` (private headers stay under `libs/algs/src/`), `src/opencv_bindings/opencv_bindings.h` and `src/optimization/projection.h`.
- **Move these tests to `libs/algs/tests/…`:**
  - `tests/{estimators/*, images/*}`;
  - `tests/geometry/{geometry,two_view_decomposition,two_view_reconstruction,two_view_scene}_test`;
  - `tests/features/{feature_extractor_levels,keypoint_matches,feature_grid,initialization_matcher,projection_matcher,stereo_matcher}_test`;
  - `tests/map/point_statistics_test`;
  - `tests/mapping/{keyframe_decision,triangulation}_test`;
  - `tests/optimization/*`.
- Create: `libs/algs/CMakeLists.txt`, `libs/algs/tests/CMakeLists.txt`
- Modify: `libs/CMakeLists.txt`, `src/CMakeLists.txt`, `tests/CMakeLists.txt`

**Interfaces:**
- **Produces:**
  - Target `voyah_algs`, STATIC.
    - PUBLIC include directory: `libs/algs/include`.
    - PRIVATE include directory: `libs/algs/src`.
    - PUBLIC link: `voyah_common`.
    - PRIVATE links: `stb::stb`, `opencv_core`, `opencv_imgproc`, `opencv_features`, and `g2o::core g2o::stuff g2o::types_sba g2o::types_slam3d g2o::solver_eigen`.
  - Test binary `voyah_algs_tests`.
    - Its PRIVATE include directory is `libs/algs/src`, for the tests of private headers such as `projection.h`.
    - It links OpenCV the way `voyahslam_tests` does today.
- **Explicit instantiation:**
  - Extractor `.cpp`s and algs templates instantiate as they do today.
  - If a `voyah_algs` `.cpp` needs `INSTANTIATE_CLASS`, it includes `features/instantiation.h` from common.

- [ ] **Step 1: Move commit** (`git mv` only).
- [ ] **Step 2: CMake commit.** Build Debug and Release. Run the full suite. Expected: 501 tests, all pass.
- [ ] **Step 3: Header check** with only the common and algs roots, plus the dependencies. OpenCV and g2o must not be needed. Expected: clean.
- [ ] **Step 4: Commit** `git commit -m "Restructure: voyah_algs"`.

### Task 5: `voyah_slam`, umbrella, viewer relink

**Files:**
- **Move:**
  - Everything left in `include/voyahslam/` and `src/` goes to `libs/slam/…`: tracking, the remaining mapping files, loop_closing, slam, `voyahslam.h`, `voyahslam.cpp`, and the private `paired_inliers.h`.
  - Then delete the empty `include/` and `src/` directories.
- Modify: root `CMakeLists.txt` (drop `add_subdirectory(src)`), `libs/CMakeLists.txt`, `vizualization/CMakeLists.txt`, `tests/CMakeLists.txt` (it now builds only from the remaining files until Task 6)
- Create: `libs/slam/CMakeLists.txt`

**Interfaces:**
- **Produces:**
  - Target `voyah_slam`, STATIC.
    - PUBLIC include directory: `libs/slam/include`.
    - PRIVATE include directory: `libs/slam/src`.
    - PUBLIC links: `voyah_common`, `voyah_events`, `voyah_algs`.
  - `voyahslam`, an INTERFACE target that links all four libraries PUBLIC.
  - `voyahviz` links `voyah_common` and `voyah_events` PUBLIC instead of `voyahslam`. If the build shows `voyahviz` really needs something else, stop and report it; that would contradict the spec.

- [ ] **Step 1: Move commit** (`git mv` only).
- [ ] **Step 2: CMake commit.** Build Debug and Release. Run the full suite. Expected: 501 tests, all pass.
- [ ] **Step 3: Header check** of `libs/slam/include` with all four roots. Run the same check on `vizualization/include` with only the common and events roots. Expected: clean.
- [ ] **Step 4: Link check** (Review Focus 1). Build `hello_voyahslam`, `monocular_full_slam`, `binocular_full_slam` and `voyahviz_tests` in Debug and Release. Expected: all link.
- [ ] **Step 5: Commit** `git commit -m "Restructure: voyah_slam; voyahslam umbrella; voyahviz links common+events"`.

### Task 6: Test binaries, labels, `check` targets

**Files:**
- Move: the remaining `tests/*` files go to `libs/slam/tests/…`. The integration tests (from the Task 1 integration list) go to `libs/slam/tests/integration/…`.
- Create: `libs/slam/tests/CMakeLists.txt`, `cmake/VoyahTests.cmake`
- Modify: every `libs/*/tests/CMakeLists.txt` and `vizualization/tests/CMakeLists.txt` (labels); root `CMakeLists.txt`
- Delete: `tests/CMakeLists.txt` and the `voyahslam_tests` binary. `tests/support/` stays as the home of `voyah_test_support`, or moves to `libs/test_support/`; pick one and record it.

**Interfaces:**
- **Produces:**
  - `voyah_slam_tests`, labelled `unit`.
  - `voyah_integration_tests`, labelled `integration`.
  - Every other binary is labelled `unit`.
  - The helper `voyah_add_test_binary(<name> LABEL <unit|integration> SOURCES … LIBS …)` in `cmake/VoyahTests.cmake`. It wraps `add_executable`, `target_link_libraries(... GTest::gtest_main)` and `gtest_discover_tests(<name> PROPERTIES LABELS <label> DISCOVERY_TIMEOUT 60)`.
  - Custom targets:
    - `check` runs `ctest -L unit -j8 --timeout 120 --output-on-failure`;
    - `check_all` runs `ctest -j8 --timeout 300 --output-on-failure`.
    - Both use `WORKING_DIRECTORY ${CMAKE_BINARY_DIR}` and depend on every test binary.
- **Splitting files:** an integration test whose file also holds unit tests is moved by splitting the file. The integration test cases go to `…/integration/<same name>_test.cpp`. Fixtures shared by both halves move to `voyah_test_support`.

- [ ] **Step 1: Move commit** (`git mv` only, plus the file splits as a separate content commit).
- [ ] **Step 2: CMake commit.** Build, then run `check_all`. Expected: 501 tests (`ctest -N`), all pass. `ctest -L unit -N` plus `ctest -L integration -N` = 501.
- [ ] **Step 3: Time `check`** (build excluded). Expected: < 15 s. If it is over, move tests above 1 s to integration per the timings file, and record what moved.
- [ ] **Step 4: Commit** `git commit -m "Restructure: per-library test binaries, unit/integration labels, check targets"`.

### Task 7: `VOYAHSLAM_OPTIMIZE_TESTS`

**Files:**
- Modify: root `CMakeLists.txt`, `cmake/VoyahTests.cmake`
- Test: `libs/common/tests/build_config_test.cpp`

**Interfaces:**
- **Produces:** `option(VOYAHSLAM_OPTIMIZE_TESTS "Compile voyah libraries and tests with -O2 in Debug" ON)`.
  - When the option is ON and the build type is Debug, `target_compile_options(<t> PRIVATE -O2)` is applied to the four libraries and every test binary.
  - The compile flags `-g` and `NDEBUG` are left unchanged.
  - Dependencies are untouched.

- [ ] **Step 1: Write the test** `BuildConfigTest.AssertsEnabled`, which fails when `NDEBUG` is defined:
  ```cpp
  TEST(BuildConfigTest, AssertsEnabled) {
  #ifdef NDEBUG
      GTEST_SKIP() << "Release build";
  #else
      SUCCEED();
  #endif
  }
  ```
  It is guarded so that the Release tree skips it. In the Debug tree it must report PASSED, not SKIPPED. Check this with `--gtest_filter=BuildConfig*` and verify the output says `[  PASSED  ]` with 0 skipped.
- [ ] **Step 2: Implement the option.** Build Debug and run `check_all`. Expected: all pass, and `BuildConfigTest.AssertsEnabled` passes rather than skips (Review Focus 5).
- [ ] **Step 3: Measure** `check` and `check_all` wall time in the Debug tree with the option ON.
  - If `check_all` is not at least 2× faster than 76 s, measure `check_all` in the Release tree. Record which tree meets the target; that becomes the documented `check_all` tree.
  - Record all numbers in the timings file.
- [ ] **Step 4: Commit** `git commit -m "Restructure: VOYAHSLAM_OPTIMIZE_TESTS; measured test times"`.

### Task 8: Verification and docs

**Files:**
- Modify: `docs/DESIGN.md`, `CLAUDE.md`, `README.md`, `docs/ROADMAP.md`, `docs/TODO.md`, `docs/superpowers/specs/2026-10-07-restructure-design.md` (fix the §3 `paired_inliers` row: it belongs to `voyah_slam_tests`, because its header is private to `voyah_slam`)

- [ ] **Step 1: Layering experiment** (Review Focus 3).
  - Temporarily add `#include <voyahslam/features/projection_matcher.h>` to `libs/common/include/voyahslam/map/frame.h`. Build `voyah_common_tests`. Expected: compile error, "file not found".
  - Revert. Record the error line in the report.
- [ ] **Step 2: ThreadSanitizer.** Make a scratch TSan tree from the Debug toolchain (`-fsanitize=thread -O1 -g`). Run `setarch $(uname -m) -R` on `voyah_slam_tests` and `voyah_integration_tests`. Expected: no warnings.
- [ ] **Step 3: Header check** of every library with its own roots. Expected: clean.
- [ ] **Step 4: Smoke runs** (Release, corridor1 `/home/vahagn/Downloads/dataset-corridor1_1024_16`). Run mono for 400 frames and stereo for 200 frames.
  - Expected: the same maps, tracked frames and re-initializations as the latest `DESIGN.md` smoke table, within the run-to-run variation recorded there.
  - Report the numbers side by side.
- [ ] **Step 5: Docs.**
  - **`DESIGN.md`:**
    - the library layout table and its dependency rule;
    - the `libs/` tree;
    - how to run `check` / `check_all` (and which tree for `check_all`, from Task 7);
    - a decisions-log entry dated 2026-10-07 covering approach C, the extractor declarations in common, the integration threshold of 1 s and `VOYAHSLAM_OPTIMIZE_TESTS`;
    - updated path mentions (`src/…` → `libs/<lib>/src/…`).
  - **`CLAUDE.md`:** the build/test commands.
  - **`README.md`:** the build steps and the library list.
  - **`ROADMAP.md`:** mark step 0 done, with the measured `check` / `check_all` times.
  - **`TODO.md`:** the descriptor-traits refactor and Conan packaging of the separate libraries.
- [ ] **Step 6: Run** `check_all`, then build Release. Expected: green, 501 tests.
- [ ] **Step 7: Commit** `git commit -m "Restructure: verification and docs"`.
