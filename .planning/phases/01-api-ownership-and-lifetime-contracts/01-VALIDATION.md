---
phase: "1"
slug: "api-ownership-and-lifetime-contracts"
# status lifecycle: draft (seeded by plan-phase) → validated (set by validate-phase §6)
# audit-milestone §5.5 distinguishes NOT-VALIDATED (draft) from PARTIAL (validated + nyquist_compliant: false) (#2117)
status: validated
# false: six behaviours are covered only by manual / release-gate checks (G1..G6 below);
# gaps accepted as manual-only by the user (2026-10-15), no new tests added.
nyquist_compliant: false
wave_0_complete: true
created: "2026-10-12"
---

# Phase 1 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | GoogleTest 1.15 via `gtest_discover_tests` + `ctest`; labels `unit` / `integration` / `vocabulary` |
| **Config file** | `cmake/VoyahTests.cmake`; per-library `libs/<lib>/tests/CMakeLists.txt` |
| **Quick run command** | `cmake --build cmake-build-debug/build/Debug --target check` (812 unit tests) |
| **Full suite command** | `cmake --build cmake-build-debug/build/Debug --target check_all` (852 tests; 1 skipped: `VocabularyEquivalenceTest` without the ORBvoc file) |
| **Latest gate** | 852/852 pass (R6 gate) |
| **Sanitizer gate** | TSan tree `cmake-build-tsan`; every TSan binary runs under `setarch -R` (else `FATAL: ThreadSanitizer: unexpected memory mapping`); `-*DeathTest*` excludes `SubscriptionDeathTest` and `SlamDeathTest` from TSan runs |
| **Release gate** | `conan create . --build=missing -pr profiles/linux-gcc13` (static), same with `-o shared=True`, and `-pr:h profiles/android-arm64 -pr:b default` |
| **Estimated runtime** | ~10 s (`check`), ~30 s (`check_all`) plus build |

---

## Sampling Rate

- **After every task commit:** the touched test binary with a `--gtest_filter`, then `--target check`
- **After every plan wave:** `--target check_all`
- **Before each release merge:** `check_all` green, the manual / release gates G1..G6 below, the user's explicit "merge and push"
- **Max feedback latency:** 60 seconds

Known flaky tests (`docs/TODO.md`, about lines 815-848), not the sole coverage of any Phase 1 behaviour:
`SlamTest.DuplicatesFusedInLongSequence`, `SlamTest.StereoMapGrowsWhileLeavingInitialView`,
`SlamTest.MonoRelocalizesAfterBlackout`, `SlamTest.StereoRelocalizesAfterBlackout`.

---

## Per-Task Verification Map

Requirement → test map, all green at the R6 gate (852/852):

| Requirement | Behavior | Test Type | Automated Command / Tests | File Exists | Status |
|-------------|----------|-----------|---------------------------|-------------|--------|
| OWN-01 | Received event immutable by type; `Notify` rejects an lvalue | compile-time | `event_lifetime_test` `static_assert`s (`CanNotify`, `CanAssignPriority`) in the `voyah_events_tests` build | ✅ `libs/events/tests/events/event_lifetime_test.cpp` | ✅ green |
| OWN-01 | Event held by a consumer outlives producer, subscription, observer and listener; freed once, on the last drop's thread | unit + ASan/TSan | `EventLifetimeTest.{OutlivesProducerSubscriptionObserverAndListener, LastDropFreesOnTheConsumerThread, NotifyWithoutObserversFreesAtOnce, ObserverDestroyedWithQueuedEventsReleasesThem}` | ✅ (same file) | ✅ green |
| OWN-01 | Slam destroyed while an observer still holds events: events valid, nothing dropped | integration | `SlamTest.{EventsOutliveSlam, StuckObserverHoldsEveryTrackerEvent}` | ✅ `libs/slam/tests/integration/slam_test.cpp` | ✅ green |
| OWN-01 | No pool / `Recyclable` / `StopAll` / overflow-counter symbols left | release gate | see G1 (manual-only) | n/a | ⬜ manual (G1) |
| OWN-02 | All 24 destruction orders of {producer, subscription, observer, listener} safe; every event freed once | unit + ASan/TSan | `AllOrders/DestructionOrderTest.AnyOrderIsSafe/*` (24) | ✅ `libs/events/tests/events/subscription_test.cpp` | ✅ green |
| OWN-02 | Observer destroyed first never notified; connect / `reset()` / move-assign while running throws; usage errors are `logic_error` | unit | `SubscriptionTest.{ObserverDestroyedWhileProducerRunsIsSafe, DestroyedSubscriptionDisconnects, DisconnectKeepsOtherProducersQueuedEvents, AddObserverWhileRunningThrows, AddSyncObserverWhileRunningThrows, ResetWhileRunningThrows, MoveAssignWhileRunningThrows, UsageErrorIsLogicError}`; `AsyncObserverTest.DestructorReleasesQueuedEvents` | ✅ (same file) | ✅ green |
| OWN-02 | Subscription destroyed while its producer runs terminates | death | `SubscriptionDeathTest.DestroyedWhileProducerRunsTerminates` (not under TSan) | ✅ (same file) | ✅ green |
| OWN-02 | Slam level: observer destroyed while running is safe; unwinding past the holder does not terminate | integration | `SlamTest.{ObserverDestroyedWhileSlamRunsIsSafe, UnwindPastSubscriptionHolderDoesNotTerminate, AddObserverWhileRunningThrows}` | ✅ `slam_test.cpp` | ✅ green |
| OWN-03 | Restricted accessors are not public (only through `*TestAccess`) | compile | `ThreadAffinityTest.RestrictedAccessorsAreNotPublic` | ✅ `libs/slam/tests/thread_affinity_test.cpp` | ✅ green |
| OWN-03 | `MapFollower` mirrors the Mapper map | unit + integration + TSan | `MapFollowerTest.*` (8); `SlamTest.MapFollowerMirrorsMapper` | ✅ `libs/slam/tests/mapping/map_follower_test.cpp` | ✅ green |
| OWN-03 | `MapFollower` adds no library template instantiations (R2 fix dac50cf / 202fe43) | build check | `BuildCheck.MapFollowerAddsNoLibraryInstantiations` | ✅ `libs/slam/tests/build_checks/` | ✅ green |
| OWN-04 | Invalid transitions throw; concurrent `Stop` joins; `Stop` idempotent | unit + TSan | `LifecycleTest.*` (12: StartTwiceThrows, StartAfterStopThrows, StopBeforeStartIsTerminal, FailedStartLeavesStopped, StartMovesCreatedToRunning, AddObserverAllowedWhenCreatedAndStopped, ConcurrentStartsOneWins, ConcurrentStopsReturnOnlyAfterJoin, StopIsIdempotent, DestructorStopsARunningNode, StartAndStopFromDifferentThreads, StopFromWorkerThreadThrows) | ✅ `libs/events/tests/events/lifecycle_test.cpp` | ✅ green |
| OWN-04 | Tracker queues before start, throws after stop | unit | `TrackerTest.{TrackBeforeStartQueues, TrackAfterStopThrows}` | ✅ | ✅ green |
| OWN-04 | Slam lifecycle, including option (a): `Stop` from a component thread changes nothing, from a foreign thread still stops (d4df304 / f46763d / 3c88365) | integration | `SlamTest.Lifecycle*` (incl. `LifecycleStopFromComponentThreadChangesNothing`, `LifecycleStopFromForeignThreadStillStops`); `SlamDeathTest.DestroyedOnAComponentThreadTerminates` (not under TSan) | ✅ `slam_test.cpp` | ✅ green |
| OWN-05 | Concurrent `DetectAndExtract` equals single-thread result | unit + TSan | `OrbExtractorWorkspaceTest.*` (5) | ✅ `libs/algs/tests/features/orb_extractor_workspace_test.cpp` | ✅ green |
| OWN-05 | Extractor output unchanged | golden (dataset) | `OrbExtractorGoldenTest.Corridor1OutputUnchanged` (`voyah_algs_dataset_tests`); must not be skipped, see G4 | ✅ | ✅ green |
| OWN-06 | Loggers created before `Initialize` re-targeted; concurrent create / log / `Initialize` | unit + TSan | `LoggerTest.*` (6) | ✅ `libs/common/tests/logging/logger_test.cpp` | ✅ green |
| OWN-07 | Default `Image` empty; view ops honour `offset_` / `stride_`; overflow guarded | unit + ASan | `ImageTest.*` (14) | ✅ `libs/common/tests/images/image_test.cpp` | ✅ green |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

Removed from the earlier plan: `SetWakeSignal` (removed in R1); the note that the `voyah_slam` test binaries
do not link until plan 01-05 (obsolete, every binary builds); the OWN-01 `grep` row as an automated item
(now the manual gate G1).

---

## Wave 0 Requirements

- [x] `libs/events/tests/events/subscription_test.cpp` — OWN-02 (`DestructionOrderTest`, `SubscriptionTest`, `SubscriptionDeathTest`)
- [x] `libs/events/tests/events/event_lifetime_test.cpp` — OWN-01
- [x] Test-only live-counting event (`Probe`) in a shared test header — OWN-01/02
- [x] `libs/events/tests/events/lifecycle_test.cpp` — OWN-04
- [x] `libs/common/tests/images/image_test.cpp` — OWN-07
- [x] `libs/common/tests/logging/logger_test.cpp` — OWN-06
- [x] `libs/slam/tests/mapping/map_follower_test.cpp`, `libs/slam/tests/thread_affinity_test.cpp` + shared `*TestAccess` header — OWN-03
- [x] `libs/slam/tests/build_checks/` (`BuildCheck.MapFollowerAddsNoLibraryInstantiations`) — OWN-03
- [x] `libs/algs/tests/features/orb_extractor_workspace_test.cpp` + `OrbExtractorGoldenTest` (`voyah_algs_dataset_tests`) — OWN-05
- [x] `libs/slam/tests/integration/slam_test.cpp` Phase 1 additions — OWN-01/02/03/04
- [x] Death-test convention (`threadsafe` style, `*DeathTest` names, excluded from TSan)

All Wave 0 files exist and are green.

---

## Manual-Only Verifications

User decision (2026-10-15): add no new tests; every gap below is recorded as manual-only / release-gate.

| ID | Behavior | Requirement | Why Manual | Test Instructions |
|----|----------|-------------|------------|-------------------|
| G1 | Removed ownership symbols stay removed | OWN-01 | absence of symbols is a repository property, not a test | Release gate: `git grep -E 'ObjectPool\|Recyclable\|StopAll\|event_pool_size\|transaction_pool_size\|max_events_in_flight\|num_dropped_loops\|num_results_dropped\|SetWakeSignal' -- libs examples vizualization test_package` prints nothing |
| G2 | Concurrency tests prove race-freedom only under TSan | OWN-02, OWN-04, OWN-05, OWN-06 | needs the separate TSan tree and `setarch -R`; death tests excluded | Release gate in `cmake-build-tsan`, each under `setarch -R`: `voyah_events_tests`; `voyah_slam_tests --gtest_filter='Tracker*:Mapper*:Slam*:GlobalBundle*:LoopCloser*:MapFollower*:-*DeathTest*'`; `voyah_integration_tests --gtest_filter='-*DeathTest*'`; `voyah_common_tests --gtest_filter='LoggerTest.*'`; `voyah_algs_tests --gtest_filter='OrbExtractorWorkspaceTest.*'`; expect 0 `WARNING: ThreadSanitizer`. Plus ASan on `voyah_events_tests` and `ImageTest.*` |
| G3 | App tracked-frames bar | OWN-02, OWN-03 | needs the corridor1 dataset and the example apps; caught the R2 `MapFollower` `-O0` regression that every test missed | `binocular_full_slam` on corridor1 `--rate=0 --max-frames=40`: >= 36 `[Mapper] [debug] FrameTracked` lines and "map: >= 3 Frames" (baseline 39 / 3 Frames / 777 points); `monocular_full_slam` exits 0 (WaitingForMap at 40 frames is the known state); neither prints "terminate" |
| G4 | Golden extractor test must not be skipped | OWN-05 | the test self-skips without the dataset | In the release gate confirm `OrbExtractorGoldenTest.Corridor1OutputUnchanged` ran (not "Skipped") |
| G5 | Packaging, docs and lock-freedom review | OWN-01, OWN-02 (all) | needs `conan create` toolchains and a reviewer's judgement | `conan create` `voyahslam/<version>` for linux-gcc13 static, shared and android-arm64 (test_package prints `voyahslam X.Y: unproject error ...`); DESIGN / TODO / version greps; reviewer reads that `inbox_core.h`, `subscription.h` and `observable.h` hold no locks |
| G6 | Image views alias the parent buffer | OWN-07 | a documented contract, not behaviour a test can pin | Documented in `image.h`; reviewer read |
| G7 | "merge and push" per release | all | the user's explicit approval | wait for the user's words |
| G8 | voyah_jni session told the new version | all | cross-repo communication | message the voyah_jni session with the version and API changes |

---

## Validation Sign-Off

- [x] All tasks have `<automated>` verify or Wave 0 dependencies
- [x] Sampling continuity: no 3 consecutive tasks without automated verify
- [x] Wave 0 covers all MISSING references
- [x] No watch-mode flags
- [x] Feedback latency < 60s
- [ ] `nyquist_compliant: true` set in frontmatter — left `false`: gaps G1..G6 accepted as manual-only by the user (2026-10-15)

**Approval:** validated 2026-10-15 (partial: gaps accepted as manual-only by the user)

## Validation Audit 2026-10-15

| Metric | Count |
|---|---|
| Gaps found | 6 |
| Resolved | 0 |
| Escalated | 6 |
