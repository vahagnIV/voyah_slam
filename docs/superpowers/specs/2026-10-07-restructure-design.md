# Restructure into Four Libraries and a Fast Test Loop — Design Spec

Date: 2026-10-07
Status: approved in conversation, pending spec review
Roadmap: step 0 in `docs/ROADMAP.md`

## Context and Scope

voyahslam is one library (`voyahslam`, 30 `.cpp` files) with one test binary
(`voyahslam_tests`, Debug). The full suite has 458 tests (+ 43 in
`voyahviz_tests`) and takes 76 s at `ctest -j8`. (amended: the 458 already
include the 43 `voyahviz_tests` and the 11 `dataset_reader_tests`; the top-level
`ctest -N` total was 458, Ruling F.) About five tests dominate
that time, at 33–63 s each (synthetic end-to-end, Tracker sequences, init
BA). `voyahviz` links the whole library.

**Goals (user):**

- a faster edit-and-test loop;
- enforced layer boundaries;
- a small `voyah_events` library for outside observers such as the viewer.

**Constraints:**

- no behaviour change;
- every test passes, with the same count;
- OpenCV, stb and g2o stay private;
- no `#include` path changes;
- no lower layer may include an upper layer.

**Out of scope:**

- behaviour changes and namespace or include-path renames;
- templating on descriptor traits instead of extractors;
- Conan packaging of the separate libraries;
- DBoW2 (roadmap step 2).

## 1. Layout: separate include roots, same paths

Each library has its own directory:

```
libs/<lib>/include/voyahslam/...   public headers (same relative paths as today)
libs/<lib>/src/...                 sources and private headers
libs/<lib>/tests/...               unit tests
```

- An `#include <voyahslam/features/…>` line stays as it is.
- A target's include path holds only its own root and the roots of the
  libraries it depends on. A lower layer that includes an upper-layer header
  therefore **fails to compile**. That is the enforcement.
- History: commits that move files contain only `git mv`. Content edits go in
  separate commits.

## 2. Libraries

| Library | Public headers (under `voyahslam/`) | Sources | Depends on |
|---|---|---|---|
| `voyah_common` | `defs.h`; `memory/*`; `logging/*`; `algebra/*`; `geometry/{rigid_transformation, camera, two_view_correspondences, distortion_models/*}`; `images/{image, image_traits}`; `features/{image_features, monocular_measurement, binocular_measurement, measurement, stereo_rig, stereo_matches, keypoint_matches, instantiation, orb_feature_extractor, sift_feature_extractor, orb/image_pyramid, distortion_models/*}` (forwarding headers) (amended: `orb/image_pyramid.h` is in common, because `OrbFeatureExtractor` holds an `orb::ImagePyramid` by value; its `.cpp` is in algs, Ruling G); `map/{frame_id, map_id, map_point_id, point_reference, frame, map_point_storage, observation_storage, covisibility_index}` | `algebra.cpp`, `camera.cpp`, `logger.cpp`, distortion model `.cpp`s, `covisibility_index.cpp` | Eigen, spdlog (PUBLIC) |
| `voyah_events` | `events/*` | `worker_thread.cpp` | common; concurrentqueue (PUBLIC) |
| `voyah_algs` | `estimators/*`; `geometry/{geometry, two_view_decomposition, two_view_reconstruction}`; `images/{filter, gaussian_blur, resizer, bilinear_interpolation}`; `io/read_image`; `features/{feature_grid, rotation_histogram, brute_force_matcher, initialization_matcher, projection_matcher, stereo_matcher}` (amended: `orb/image_pyramid.h` moved to common, Ruling G); `map/point_statistics`; `mapping/{keyframe_decision, triangulation}`; `optimization/*` | the extractor `.cpp`s and `orb/*` (including `image_pyramid.cpp`), `feature_grid.cpp`, `geometry.cpp`, two-view `.cpp`s, estimator `.cpp`s, `read_image.cpp`, `point_statistics.cpp`, `keyframe_decision.cpp`, optimization `.cpp`s; private `opencv_bindings.h`, `optimization/projection.h` | common; OpenCV, stb, g2o (PRIVATE) |
| `voyah_slam` | `tracking/*`; `mapping/{mapper, map_initializer}`; `loop_closing/*`; `slam/*`; `voyahslam.h` | tracker, mapper, map initializer, loop closer, slam, `voyahslam.cpp`; private `paired_inliers.h` | common, events, algs |

- **Extractor classes.** The declarations of `OrbFeatureExtractor` and
  `SIFTFeatureExtractor` stay in common, because every template (including
  events and the viewer) is parameterised on the extractor type. Their
  implementations live in algs.
  - Cost: code linking only common that calls `DetectAndExtract` gets a link
    error, not a compile error.
  - The descriptor-traits refactor goes to TODO.
- **Explicit instantiation** (`INSTANTIATE_CLASS`) stays in the `.cpp` of the
  library that defines each template.
- **Placement rule for surprises.** If a move makes a lower layer need an
  upper-layer header:
  - a needed data type moves down;
  - a dependent algorithm moves up.

  Each such move is recorded as a ruling.
- **`voyahslam`** becomes an INTERFACE target that links all four libraries.
  The apps and outside users don't change.
- **`voyahviz`** links only `voyah_common` and `voyah_events`.
- **`voyah_test_support`** is a header-only INTERFACE target holding today's
  `tests/support/*` and `tests/utils_test.*`. (amended: it is a STATIC
  library, because `utils_test.cpp` holds helper definitions, Ruling C; it
  lives in `libs/test_support` and is common-level: it links only
  `voyah_common`. `support/synthetic_sequence.h` needs algs and only compiles
  in binaries that link `voyah_algs`; see `docs/TODO.md`.)

## 3. Tests

| Binary | Contents | Label |
|---|---|---|
| `voyah_common_tests` | camera, rigid transformation, distortion models, memory, ids, map/point/observation storage, covisibility index, frame, measurements, image features, stereo rig, algebra | `unit` |
| `voyah_events_tests` | event system, wake signal, worker thread | `unit` |
| `voyah_algs_tests` | estimators, two-view geometry, images, extractor levels, feature grid, keypoint matches, all matchers except the projection matcher, triangulation, keyframe decision, point statistics, projection, BA, pose optimization (amended: paired inliers and the projection matcher tests are in `voyah_slam_tests`) | `unit` |
| `voyah_slam_tests` | map replica, local map, Tracker and Mapper unit tests, map initializer, loop closer (amended: also `paired_inliers_test`, whose header `mapping/paired_inliers.h` is private to `voyah_slam`, Ruling B; and `projection_matcher_test`, whose cases build points from `MapReplica`, Ruling J; the shared Tracker fixture is `libs/slam/tests/tracking/tracker_test_fixture.h`, Ruling K) | `unit` |
| `voyah_integration_tests` | `SlamTest.*`, threaded Tracker/Mapper sequences, and every test above **1 s** in the optimized build (amended: as measured, the 9 `SlamTest.*` tests plus `TrackerTest.TracksSyntheticStereo`; `ObjectPoolDeathTest.PoolExhaustion`, 1.09 s of fixed death-test cost, stays `unit`, Ruling E) | `integration` |
| `voyahviz_tests` (existing) | viewer tests | `unit` |

- A test's binary follows its code's library.
- The integration list comes from measured per-test times (§5 step 1), not
  from guesses.
- **Targets:**
  - `check` runs `ctest -L unit -j8`. Target: **< 15 s**, excluding the build.
  - `check_all` runs every test. It is used at the end of each task and
    before every merge.
- **Discovery:** `gtest_discover_tests(... PROPERTIES LABELS <label>)` per
  binary.

## 4. Optimized test build

- **Option:** `VOYAHSLAM_OPTIMIZE_TESTS`, default ON. In a Debug
  configuration it adds `-O2 -g` to the four libraries and every test
  binary.
- **Asserts stay on:** `NDEBUG` is not defined.
- **Conan dependencies keep their build type,** so g2o stays Debug.
- **Fallback:** if g2o Debug still dominates `check_all`, `check_all` runs
  from the Release tree (`build/build/Release`). Record which one applies.
- **Debugging:** turn the option off to step through code.

## 5. Migration order

Every step leaves the tree building and all tests green.

1. **Baseline:** per-test times in today's Debug build, plus the same with
   `-O2` applied ad hoc. This produces the integration list and the "before"
   numbers.
2. **`voyah_common`:** moves (`git mv` only), then CMake, then the extractor
   declaration/implementation split, then its tests. Tests that haven't
   moved stay in `voyahslam_tests` until step 6.
3. **`voyah_events`.**
4. **`voyah_algs`.**
5. **`voyah_slam`,** the `voyahslam` INTERFACE target, and `voyahviz`
   relinked.
6. **Test binaries:** the slam and integration binaries, labels,
   `check` / `check_all`. The old `voyahslam_tests` binary is removed.
7. **`VOYAHSLAM_OPTIMIZE_TESTS`,** measured.
8. **Docs** (§7).

## 6. Acceptance

- **Builds:** the Debug and Release trees build.
- **Test count:** 458 + 43 tests, all green, unchanged. (amended: the
  invariant top-level total was 458, viz included, Ruling F; Task 7 added
  `BuildConfigTest.AssertsEnabled`, so it is **459**: common 102, events 29,
  algs 174, slam 90, integration 10, viz 43, dataset_reader 11.)
- **Speed:** `check` < 15 s; `check_all` at least 2× faster than 76 s. Both
  are measured and recorded.
- **Layering:** a one-off experiment in which a common header includes an
  algs header must fail to compile. It is recorded in the report and not
  kept in the code.
- **Viewer linkage:** `voyahviz` links only common and events, verified with
  `target_link_libraries` and a build.
- **ThreadSanitizer:** `voyah_slam_tests` and `voyah_integration_tests` run
  clean.
- **clang-18/libc++:** every public header of each library passes
  `-fsyntax-only` with only that library's include roots.
- **Smoke runs** (corridor1, Release):
  - mono 400 frames and stereo 200 frames;
  - same numbers as the latest `DESIGN.md` smoke table, up to the known
    run-to-run variation.

## 7. Docs

- **`docs/DESIGN.md`:**
  - the library layout and its dependency rule;
  - how to run `check` / `check_all`;
  - a decisions-log entry dated 2026-10-07, covering approach C, the
    extractor declarations staying in common, `VOYAHSLAM_OPTIMIZE_TESTS`,
    and the integration threshold of 1 s.
- **`CLAUDE.md`:** build and test commands (`check`, `check_all`).
- **`README.md`:** build instructions and the library list.
- **`docs/ROADMAP.md`:** step 0 marked done, with the measured numbers.
- **`docs/TODO.md`:**
  - the descriptor-traits refactor (so common no longer declares extractor
    classes);
  - Conan packaging of the separate libraries.
