---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
# Codebase Structure

**Analysis Date:** 2026-10-12

## Directory Layout

```
voyah_slam/
├── .planning/                  # GSD planning artifacts (this document)
├── cmake/                      # CMake modules (helpers, conan integration)
├── docs/
│   ├── DESIGN.md              # **Primary reference:** architecture, principles, decisions
│   ├── ROADMAP.md             # Ordered next steps
│   ├── TODO.md                # Pending work items
│   └── superpowers/           # Spike/plan docs (detailed features, implementation notes)
│       ├── specs/             # Detailed specifications (2026-10-03-*.md, etc.)
│       ├── plans/             # Implementation plans (2026-10-*.md, etc.)
│       └── spikes/            # Proof-of-concept investigations
├── libs/                      # Four-library stack + umbrella target
│   ├── common/                # **Tier 1:** IDs, geometry, memory, features, logging
│   │   ├── include/voyahslam/
│   │   │   ├── map/           # MapId, FrameId, MapPointId, Frame, MapPointStorage, CovisibilityIndex
│   │   │   ├── geometry/      # Camera, Sim3, RigidTransformation, DistortionModels
│   │   │   ├── features/      # ImageFeatures, MonocularMeasurement, BinocularMeasurement, StereoRig
│   │   │   ├── images/        # Image type (RAII wrapper)
│   │   │   ├── memory/        # ObjectPool, Recyclable, ColumnBuffer, SyncPolicy
│   │   │   ├── logging/       # Logger interface
│   │   │   ├── algebra/       # Matrix/vector utilities (forw. headers)
│   │   │   └── bow/           # Bag-of-Words vectors
│   │   ├── src/
│   │   │   ├── map/           # Frame.cpp, MapPointStorage implementation
│   │   │   ├── geometry/      # Camera, Sim3, DistortionModels implementation
│   │   │   ├── memory/        # ObjectPool, Recyclable implementation
│   │   │   └── logging/       # Logger initialization
│   │   └── tests/             # Unit tests for all common layers
│   │
│   ├── events/                # **Tier 2:** Event types, pub/sub, worker threads
│   │   ├── include/voyahslam/events/
│   │   │   ├── event_types.h  # Base Event, EventType enum, type tag
│   │   │   ├── observable.h   # Sync/async observer registration
│   │   │   ├── async_observer.h # Lock-free inbox, priority counter
│   │   │   ├── frame_tracked.h      # FrameTracked event
│   │   │   ├── map_transaction.h    # MapTransaction event (all map changes)
│   │   │   ├── loop_detected.h      # LoopDetected event
│   │   │   ├── global_ba_finished.h # GlobalBAFinished event
│   │   │   ├── tracking_lost.h      # TrackingLost event
│   │   │   ├── initialization_measurement.h # InitializationMeasurement event
│   │   │   ├── event_node.h   # Mixin: Observable + inbox + AddObserver
│   │   │   ├── worker_thread.h # Loop: snapshot → ProcessPending → wait
│   │   │   ├── wake_signal.h  # Atomic counter + C++20 wait/notify
│   │   │   └── listener.h     # RAII queue iterator, auto-recycles
│   │   ├── src/events/        # Event pool instantiation, listener impl
│   │   └── tests/             # Event lifecycle, pool overflow, priority
│   │
│   ├── algs/                  # **Tier 3:** Algorithms (vision, geometry, optimization)
│   │   ├── include/voyahslam/
│   │   │   ├── features/      # ORB extractor, FAST, matchers (brute-force, BoW, projection, stereo, fuse)
│   │   │   ├── geometry/      # Estimators (essential, homography, Sim3 solver), two-view reconstruction, PnP
│   │   │   ├── estimators/    # RANSAC, TwoViewEstimator base
│   │   │   ├── optimization/  # PoseOptimization, BundleAdjustment, EssentialGraph, Sim3Optimization
│   │   │   ├── mapping/       # KeyframeDecision, MapInitializer, Triangulation, MapQuality
│   │   │   ├── bow/           # FrameDatabase (loop detection), Vocabulary
│   │   │   ├── images/        # GaussianBlur, Resizer, BilinearInterpolation, Filter
│   │   │   ├── io/            # ReadImage
│   │   │   └── map/           # PointStatistics
│   │   ├── src/
│   │   │   ├── features/      # ORB, FAST, matcher implementations
│   │   │   │   ├── orb/
│   │   │   │   └── fast/
│   │   │   ├── geometry/
│   │   │   ├── estimators/
│   │   │   ├── optimization/
│   │   │   ├── mapping/
│   │   │   ├── bow/
│   │   │   ├── images/
│   │   │   └── io/
│   │   ├── benchmarks/        # Performance tests (reference ORB-SLAM3 comparisons)
│   │   └── tests/             # Feature extractors, matchers, estimators, geometric verifications
│   │
│   ├── slam/                  # **Tier 4:** Tracker, Mapper, LoopCloser, GlobalBA, Slam orchestrator
│   │   ├── include/voyahslam/
│   │   │   ├── slam/
│   │   │   │   └── slam.h     # **Entry point:** Slam<TFeatureExtractor> template
│   │   │   ├── tracking/
│   │   │   │   ├── tracker.h           # Tracker<FE>, state machine, Track()
│   │   │   │   ├── atlas_replica.h    # Tracker's map replica (applies MapTransactions)
│   │   │   │   ├── map_replica.h      # Single map state replica
│   │   │   │   ├── local_map.h        # Local neighborhood computation
│   │   │   │   └── relocalizer.h      # Pose recovery from BoW (2026-10-08)
│   │   │   ├── mapping/
│   │   │   │   ├── mapper.h           # Mapper<FE>, state machine, Atlas (writer)
│   │   │   │   └── map_initializer.h # Mono/stereo initialization
│   │   │   ├── loop_closing/
│   │   │   │   ├── loop_closer.h           # LoopCloser<FE>, place recognition
│   │   │   │   └── common_region.h        # Loop candidate structure, Sim3 verification
│   │   │   └── global_ba/
│   │   │       ├── global_bundle_adjuster.h # GlobalBA<FE>, loop correction optimization
│   │   │       └── global_ba.h              # BA problem formulation, solver
│   │   ├── src/
│   │   │   ├── slam/
│   │   │   ├── tracking/     # Tracker loop, pose optimization, state machine
│   │   │   ├── mapping/      # Map initialization, keyframe creation, BA, loop closure
│   │   │   ├── loop_closing/ # Loop detection, geometric verification, Sim3 computation
│   │   │   └── global_ba/    # Global BA setup, execution, pose/point updates
│   │   └── tests/
│   │       ├── tracking/     # Pose estimation, state transitions
│   │       ├── mapping/      # Keyframe creation, triangulation, fusion
│   │       ├── loop_closing/ # Loop detection, validation
│   │       ├── global_ba/    # BA convergence, interrupt handling
│   │       ├── integration/  # End-to-end SLAM with datasets
│   │       └── features/     # Feature extraction for tracking/mapping tests
│   │
│   ├── test_support/         # Shared test utilities (assertions, mocks, fixtures)
│   │   └── support/
│   │
│   └── CMakeLists.txt        # Four targets: voyah_common, voyah_events, voyah_algs, voyah_slam, voyahslam (umbrella)

├── vizualization/            # MapViewer (dev tool, Pangolin + OpenCV rendering)
│   ├── include/voyahviz/
│   │   └── map_viewer.h      # SceneState, event subscription
│   ├── src/
│   └── tests/

├── examples/                 # Test apps and dataset readers
│   ├── full_slam/            # Release build apps (SLAM on real video/datasets)
│   │   └── full_slam_app.cpp (or multiple entry points)
│   └── dataset_reader/       # Dataset streaming utility
│       ├── include/dataset_reader/
│       ├── src/
│       └── tests/

├── tools/                    # Utilities (not part of main library)
│   ├── convert_vocabulary/   # BoW vocabulary format converter
│   └── dbow2_reference/      # DBoW2 reference implementation

├── third_party/              # Vendored dependencies
│   └── g2o/                  # Graph optimizer (submodule), local recipe

├── profiles/                 # Conan profiles
│   └── linux-gcc13           # Default development profile (C++20, Release default)

├── cmake-build-debug/        # Build tree (Debug, -O2 -g with asserts)
├── build/                    # Build tree (Release, -O3)
├── CMakeLists.txt            # Root CMake
├── CMakeUserPresets.json     # Includes Conan-generated CMakePresets.json
├── conanfile.py              # Conan recipe + package definition
├── conanfile.lock            # Lock file (if used)

├── .git/
├── .idea/                    # CLion settings
├── CLAUDE.md                 # **Project instructions** (rules, conventions, build commands)
└── README.md                 # Setup, build, install, prerequisites
```

## Directory Purposes

**Root-level key files:**

- **`CLAUDE.md`**: Project rules, naming conventions (Google style), library layout, build/test commands. Read first.
- **`README.md`**: Prerequisites, one-time setup (OpenCV, Pangolin, g2o Conan), build and test command reference.
- **`docs/DESIGN.md`**: **Architecture bible.** Principles, components, event types, replicas, data layer, full details. Always consult for decisions.
- **`docs/TODO.md`**: Pending work, blockers, deferred items.
- **`docs/ROADMAP.md`**: Ordered next steps (phased feature roadmap).

**Four Libraries (`libs/*/`):**

Each library: `{include, src, tests}` + `CMakeLists.txt`.

- **`libs/common/`**: Foundation. All other libs depend on it. Types, memory, geometry, no external deps (Eigen, spdlog only).
- **`libs/events/`**: Pub/sub infrastructure. Event types, pools, workers. Depends on common only.
- **`libs/algs/`**: Algorithms. Feature extraction, matching, geometry solvers, optimization. Depends on common (g2o, stb private).
- **`libs/slam/`**: SLAM components. Tracker, Mapper, LoopCloser, GlobalBA. Depends on common, events, algs.

**Build & Installation:**

- **`cmake/`**: CMake helper modules (find_package, etc.).
- **`profiles/`**: Conan profiles (Linux GCC 13, compiler settings).
- **`third_party/g2o/`**: Optimization backend (vendored, packaged via Conan).
- **`CMakeLists.txt` + `conanfile.py`**: Build orchestration (Conan 2, C++20).

**Examples & Tools:**

- **`examples/full_slam/`**: Full SLAM application on real data (Release build).
- **`examples/dataset_reader/`**: Dataset acquisition and feature extraction (before SLAM).
- **`tools/convert_vocabulary/`**: Utility to convert BoW vocabularies.
- **`vizualization/`**: MapViewer (development-only; Pangolin, OpenCV).

## Key File Locations

**Entry Points:**
- `libs/slam/include/voyahslam/slam/slam.h` — `Slam<TFeatureExtractor>` class; app main entry point.
- `libs/slam/src/slam/slam.cpp` — Constructor, `Start()`, `Stop()`.

**Configuration & Options:**
- `libs/slam/include/voyahslam/tracking/tracker.h` — `TrackerOptions`.
- `libs/slam/include/voyahslam/mapping/mapper.h` — `MapperOptions`.
- `libs/slam/include/voyahslam/loop_closing/loop_closer.h` — `LoopCloserOptions`.
- `libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h` — `GlobalBaOptions`.

**Core Logic:**

*Initialization & Tracking:*
- `libs/slam/src/mapping/map_initializer.cpp` — Monocular/stereo initialization.
- `libs/slam/src/tracking/tracker.cpp` — Tracking loop, state machine.
- `libs/slam/src/mapping/mapper.cpp` — Keyframe creation, local mapping, loop closure.

*Loop Closing:*
- `libs/slam/src/loop_closing/loop_closer.cpp` — Place recognition, Sim3 verification.
- `libs/slam/src/mapping/mapper.cpp` (section: loop handling) — Loop closure and map merging.

*Global Optimization:*
- `libs/slam/src/global_ba/global_bundle_adjuster.cpp` — Post-loop-closure BA.

*Feature Extraction & Matching:*
- `libs/algs/include/voyahslam/features/orb_feature_extractor.h` — ORB extraction.
- `libs/algs/src/features/brute_force_matcher.cpp` — Feature matching.
- `libs/algs/src/features/projection_matcher.cpp` — Local-map matching during tracking.

*Geometry & Optimization:*
- `libs/algs/include/voyahslam/geometry/two_view_reconstruction.h` — Initialization reconstruction.
- `libs/algs/include/voyahslam/optimization/bundle_adjustment.h` — BA (local and global).
- `libs/algs/include/voyahslam/optimization/pose_optimization.h` — Motion-only BA for tracking.

**Testing:**
- `libs/*/tests/` — Unit tests (one per component).
- `libs/slam/tests/integration/` — End-to-end SLAM tests with datasets.
- `cmake-build-debug/` — Debug tree; run `cmake --build ... --target check`.
- `build/` — Release tree; run tests via `ctest`.

## Naming Conventions

**Files:**

- Headers: `feature_name.h` (snake_case). Location: `libs/<lib>/include/voyahslam/<topic>/`.
- Sources: `feature_name.cpp`. Location: `libs/<lib>/src/<topic>/`.
- Tests: `feature_name_test.cpp` or `feature_name_unittest.cpp`. Location: `libs/<lib>/tests/<topic>/`.
- CMakeLists.txt: One per directory that builds (common, events, algs, slam, examples, vizualization).

**Directories:**

- Topic folders: snake_case (`tracking/`, `mapping/`, `loop_closing/`, `global_ba/`).
- Namespaces mirror directory structure: `voyahslam::tracking`, `voyahslam::mapping::mapper`, etc.

**Identifiers (Google C++ style per CLAUDE.md):**

- Types: `PascalCase` (`Tracker`, `MapTransaction`, `FrameTracked`).
- Functions: `PascalCase` (`ProcessPending()`, `Start()`).
- Variables: `snake_case` (`current_map_id`, `num_inliers`).
- Member variables: `snake_case_` (`tracker_`, `map_replica_`).
- Out-parameters: `out_` prefix on non-const reference parameters.
- Const references: no marker; const is in the signature.

## Where to Add New Code

**New Feature (e.g., a novel loop-closure detector):**

1. **Algorithm/standalone:** Create in `libs/algs/include/voyahslam/<topic>/new_algo.h` + `libs/algs/src/<topic>/new_algo.cpp` + test.
   - Example: `libs/algs/include/voyahslam/bow/frame_database.h` (place recognition DB).

2. **Component-level** (integrating into Tracker/Mapper/LoopCloser):
   - Header: `libs/slam/include/voyahslam/<component>/new_feature.h`.
   - Implementation: `libs/slam/src/<component>/new_feature.cpp`.
   - Test: `libs/slam/tests/<component>/new_feature_test.cpp`.
   - Update component class to call the new code.
   - Example: `libs/slam/src/tracking/relocalizer.h` (added 2026-10-08).

3. **Data structure** (new map property, point field):
   - If core: `libs/common/include/voyahslam/map/new_data.h` + `libs/common/src/map/new_data.cpp`.
   - Update `MapPointStorage`, `Frame`, `CovisibilityIndex`, or `MapState` as needed.
   - Update serialization in `MapTransaction` if needed (if part of events).

**New Event Type:**

1. Create `libs/events/include/voyahslam/events/my_event.h` (concrete class, inherits `Recyclable<MyEvent>`, implements `Reset()`).
2. Update `libs/events/include/voyahslam/events/event_types.h` to add enum and factory.
3. Update `libs/events/include/voyahslam/events/events.h` to include it.
4. Instantiate pool in `libs/events/src/events/` (per INSTANTIATE_CLASS pattern).
5. Create observer handler in component's `ProcessPending()`.
6. Add test in `libs/events/tests/`.

**New Test:**

- Unit test: `libs/<lib>/tests/<topic>/<feature>_test.cpp`.
  - Include `<gtest/gtest.h>` and your header.
  - Use `TEST(ClassName, MethodName)` or `TEST_F(Fixture, MethodName)`.
  - Link against gtest (automatic via CMakeLists.txt).
  
- Integration test: `libs/slam/tests/integration/slam_<scenario>_test.cpp`.
  - Create `Slam<ORBFeatureExtractor>`, call `Start()`, feed measurements, check replicas.

**New Config Option:**

1. Add field to `TrackerOptions`, `MapperOptions`, etc. in component header.
2. Set default value in constructor/initializer.
3. Document in DESIGN.md (decisions log, relevant section).
4. Add getter/setter methods if needed.

**Library vs. Application Code:**

- **Library code** (`libs/`): Exported headers in `include/voyahslam/`, no CLI args, no file I/O (except examples), templated on `TFeatureExtractor`.
- **Application code** (`examples/`): Uses the library; handles file I/O, CLI, dataset iteration, visualization integration.

## Special Directories

**`.planning/codebase/`:**
- Purpose: GSD orchestrator's codebase analysis (this document + ARCHITECTURE.md).
- Generated: Automatically by `/gsd-map-codebase`.
- Committed: Yes (part of repo; reviewed in changes).

**`.superpowers/`:**
- Purpose: Project memory (spike notes, decision logs).
- Generated: Manually or by GSD.
- Committed: Yes.

**`cmake-build-debug/`, `build/`:**
- Purpose: CMake build trees.
- Generated: By `conan install` + `cmake --preset`.
- Committed: No (gitignored).

**`third_party/g2o/`:**
- Purpose: Vendored g2o (optimization backend).
- Generated: Git submodule.
- Committed: Yes (submodule reference).

**`profiles/`:**
- Purpose: Conan compiler/build type profiles.
- Generated: User-created or from template.
- Committed: Yes (shared across team).

---

*Structure analysis: 2026-10-12*
