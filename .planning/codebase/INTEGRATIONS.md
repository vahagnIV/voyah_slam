---
last_mapped_commit: b0e957ce59ec197bb96ec9ccbcc97cf037513c8e
last_mapped_at: 2026-10-12
---
# External Integrations

**Analysis Date:** 2026-10-12

## Overview

This is a pure SLAM (Simultaneous Localization and Mapping) library with **no external service integrations**. It is a local-compute, lock-free, event-driven library that:
- Accepts camera frame inputs (local or from files)
- Processes 3D reconstruction entirely in-process
- Outputs a map and pose estimates (no network transmission)

All dependencies are local libraries (Conan packages or system builds). All I/O is filesystem-based (config files, image sequences, test data).

## APIs & External Services

**Not applicable** — The library does not connect to remote APIs, cloud services, or external systems. Public APIs are C++ headers only:
- `voyahslam::Slam` - Main SLAM pipeline (`libs/slam/include/voyahslam/slam.h`)
- `voyahslam::events::Observable<T>` - Event pub/sub for observer pattern (`libs/events/include/voyahslam/events/observable.h`)
- Per-component public headers under `libs/*/include/voyahslam/`

## Data Storage

**Local filesystem only:**
- No database backend (no SQL, NoSQL, or cloud storage)
- Configuration: YAML files for example apps (dataset paths)
  - Example: `examples/dataset_reader/` reads EuRoC dataset YAML
  - Parser: `yaml-cpp` (optional, examples only)
- Test data: EuRoC dataset format (image sequences, ground truth)
  - Not bundled; downloaded separately (copyright: ETH Zurich)
- Output: In-memory map structure
  - No persistent storage; applications must serialize if needed

**In-memory state:**
- Per-thread map replicas (lock-free design): each thread owns a copy
- Frame, KeyFrame, MapPoint, and Measurement data structures
- No serialization layer (intentional: "Map management without lock" design)

## Authentication & Identity

**Not applicable** — No authentication required; no identity/credential management. Library is completely offline.

## Monitoring & Observability

**Error tracking:** None (no remote collection)

**Logs:** Local console via spdlog
- Logger: `voyahslam::Logger` (singleton)
- Output: stdout/stderr with timestamps and severity levels
- Configuration: Via `spdlog::set_pattern()` in application code
- Example: `examples/full_slam/` configures logging on startup
- Compile-time: Release builds still include logging (no NDEBUG stripping)

## CI/CD & Deployment

**Hosting:** Not applicable (library, not application)

**Building & testing:**
- CI: No GitHub Actions or external CI configured in this repo
- Package: Conan (`conan create` locally or in pipeline)
- Test artifacts: GTest binaries run locally; no upload to TestRail or similar
- Deployment: Consumer integrates as Conan dependency or CMake find_package

**Cross-platform testing:**
- x86_64 Linux (primary development and test target)
- arm64 Android (cross-compile via profiles/android-arm64)
- CI pipeline typically handles profile selection and test filtering

## Environment Configuration

**Runtime configuration:** None (pure library)

**Application-level config (examples):**
- YAML for dataset paths: `examples/dataset_reader/` (`voyahslam::EurocDatasetReader`)
- Camera calibration: Passed programmatically, not from env/config files
- Logging level: Set in application code (no env var control)

**Build-time config:**
- Conan options: `with_tests`, `with_examples`, `shared`, `fPIC` (see STACK.md)
- CMake cache variables: `VOYAHSLAM_OPTIMIZE_TESTS`, `VOYAHSLAM_BUILD_*` (see STACK.md)
- Compiler flags: `-O2` in test/debug builds (VOYAHSLAM_OPTIMIZE_TESTS), standard C++20 flags

**Required environment variables (development only):**
- `ANDROID_NDK_HOME` - Android NDK root path (for cross-compile)
  - Default fallback: `~/Android/Sdk/ndk/30.0.16248370`
  - Only needed for `profiles/android-arm64` builds
  - Not needed for Linux development

**Secrets location:** N/A (no credentials stored or used)

## Webhooks & Callbacks

**Not applicable** — No webhooks or callback integrations.

**Event pub/sub (internal only):**
- Observer pattern for decoupled thread communication
- Classes: `Observable<T>`, `AsyncObserver<T>`, `WorkerThread` (`libs/events/`)
- Events: Frame tracking, map updates, loop closures (internal SLAM events)
- Scope: In-process only, no network transmission

## Third-Party Dependencies (Non-Integration)

**Vendored source:**
- **g2o** (pose graph optimization) - Vendored pristine in `third_party/g2o/src/`
  - Upstream: https://github.com/RainerKuemmerle/g2o, tag `20241228_git`
  - Local Conan recipe: `third_party/g2o/conanfile.py` (packages as `g2o/20241228.1@voyah/stable`)
  - Build once before project: `conan create third_party/g2o --build=missing` (both Debug and Release)
  - No external dependency; self-contained

**System packages (optional, development/visualization only):**
- OpenCV 5 - Compiled from source into `/usr/local`
  - Used by: tests, visualization tool (private to tests/tools, not in library headers)
  - Not a Conan dependency (intentional; system build for dev flexibility)
  - Optional: Can omit if `VOYAHSLAM_BUILD_EXAMPLES=OFF`

- Pangolin 0.9.3 - OpenGL visualization, compiled from source into `/usr/local`
  - Used by: visualization tool only (development tool)
  - Not a Conan dependency (intentional; system build)
  - Optional: Can omit if `VOYAHSLAM_BUILD_EXAMPLES=OFF`

## Testing & Validation

**Test data sources:**
- EuRoC dataset (public: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualodometry2014)
- Synthetic data generated in tests via `voyah_test_support` helpers
- Dataset reader: `examples/dataset_reader/src/euroc_dataset_reader.cpp`

**Test framework:** Google Test (GTest 1.15.0)
- Unit tests: ~739 tests (Debug build)
- Integration tests: ~25 tests
- All tests runnable locally with `cmake --build . --target check_all`

---

*Integration audit: 2026-10-12*
