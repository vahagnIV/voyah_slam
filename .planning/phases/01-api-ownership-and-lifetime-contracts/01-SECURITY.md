---
phase: "01"
slug: api-ownership-and-lifetime-contracts
status: verified
# threats_open = count of OPEN threats at or above workflow.security_block_on severity (the blocking gate)
threats_open: 0
asvs_level: 1
block_on: high
register_authored_at_plan_time: true
created: "2026-10-15"
---

# Phase 01 — Security

> Per-phase security contract: threat register, accepted risks, and audit trail.

Scope: voyahslam (C++20 SLAM library, consumed by the voyah_jni Java/JNI binding), releases R1..R6 (0.4.0 to 0.9.0). The register was authored at plan time (the 20 PLAN.md `<threat_model>` blocks) and verified against the code, tests and git history at ASVS level 1 (a mitigation is present in the cited place). Sanitizer results (ASan/TSan) are taken from the plan SUMMARY records, not re-run by the audit.

---

## Trust Boundaries

| Boundary | Description | Data Crossing |
|----------|-------------|---------------|
| library API to consumer (JNI binding, C++ app) | The consumer's observers, listeners, subscriptions and held events die in any order and on any thread, including a GC thread (plans 01-01, 01-02, 01-05) | Shared event objects (`shared_ptr<const Event>`), subscription handles |
| producer thread to consumer threads | Events cross threads through `InboxCore` queues (01-01) | Immutable events |
| app/JNI thread to map data and component worker state | The app reads the map while the Mapper writes its own copy; a consumer could read a worker's replica while it mutates it (01-08, 01-09, 01-10) | Map replicas, snapshots, stats accessors |
| app/JNI threads (incl. a GC cleaner) to Start/Stop | Several threads may start, stop and destroy a Slam concurrently (01-12) | Component lifecycle state |
| app/JNI threads to a shared extractor | Several threads may extract features with one extractor object (01-15) | Image pyramid scratch |
| binding/app setup code to the process-wide logger | Initialize and SetLevel may run while component threads log (01-17) | Sink list, log level |
| JNI caller-supplied width/height to `Image(width, height)` | The binding builds working images from Java-provided sizes and copies width*height bytes into them (01-19) | Image dimensions (untrusted integers) |
| this repository to the voyah_jni consumer | The Conan package is the published API; every audit item is a break for the binding (01-07, 01-11, 01-14, 01-16, 01-18, 01-20) | Package version, API surface |
| library docs to the voyah_jni maintainer | The binding is written from DESIGN and the decisions log (01-06, 01-13) | Documented lifecycle and teardown rules |
| test code to library API | Tests are the first consumers of the new wiring rules (01-03) | Test assertions |
| viewer (display consumer) to producers | The viewer inbox is fed by two producers and may stall (01-04) | Held events |

---

## Threat Register

35 unique threats (T-01-01..T-01-34 and T-01-SC, which repeats in all 20 plans and is deduplicated). 30 mitigate, 5 accept. Mitigation evidence is `file:line` or test name.

| Threat ID | Category | Component | Severity | Disposition | Mitigation | Status |
|-----------|----------|-----------|----------|-------------|------------|--------|
| T-01-01 | Tampering / DoS | Event lifetime (event_types.h, observable.h) | high | mitigate | `event_types.h:27-47` (`EventPtr = shared_ptr<const Event>`, protected non-virtual dtor); `observable.h:71` `Notify(shared_ptr<T>&&)`; `EventLifetimeTest.OutlivesProducerSubscriptionObserverAndListener`, `DestructionOrderTest.AnyOrderIsSafe`; ASan clean per 01-01 SUMMARY (68/68) | closed |
| T-01-02 | Tampering / DoS | Observer destroyed before producer (async_observer.h, inbox_core.h) | high | mitigate | `inbox_core.h:48-52` (Push on a closed core drops), `:79-85` `Close()`; `~AsyncObserver` calls `core_->Close()`; `subscription_test.cpp:23-91` runs all 24 permutations (`next_permutation` over 4 parts); Debug, TSan and ASan clean per 01-01 SUMMARY | closed |
| T-01-03 | Tampering | Producer mutating a published event | medium | mitigate | `event_lifetime_test.cpp:25-40`: `static_assert(!CanAssignPriority/CanAssignMapId<EventPtr>)`, `!CanNotify` for lvalue, const-lvalue and raw pointer, `!is_destructible_v<Event>` | closed |
| T-01-04 | DoS | Unbounded inbox growth from a stalled consumer | medium | accept | See Accepted Risks Log (AR-01). Visibility tested by `AsyncObserverTest.BacklogWarningOncePerDoubling` and `ADrainedInboxNeverWarns` | closed |
| T-01-05 | Tampering / DoS | Slam teardown and the examples' unwinding path | high | mitigate | `slam.h:97` `subscriptions_` is the last member; `slam.cpp ~Slam` calls `Stop()` first; no `StopAll` anywhere (grep); `binocular_full_slam.cpp:66-68` and `monocular_full_slam.cpp:62-64` declare `subscriptions` before `slam`; `SlamTest.UnwindPastSubscriptionHolderDoesNotTerminate` (slam_test.cpp:1950) | closed |
| T-01-06 | DoS | Unbounded LoopDetected / GlobalBAFinished | medium | accept | See Accepted Risks Log (AR-02) | closed |
| T-01-07 | Tampering | Producer touching an event after Notify | medium | mitigate | All 12 production `Notify(std::move(...))` sites: mapper.cpp x3, mapper_loop, mapper_global_ba, mapper_merge, mapper_quality, global_bundle_adjuster, loop_closer, tracker x3; an lvalue does not compile (`Notify` takes `&&`) | closed |
| T-01-08 | Repudiation | Test assertions silently weakened during the mechanical migration | medium | mitigate | EXPECT_/ASSERT_ multiset check re-run against `afe4b70^` at `038f71b`: identical for the 5 migrated tracking/mapping files; only exception is the deleted pool-only `MapperTest.PooledMapTransactionIsReset` (01-03 SUMMARY deviation 1) | closed |
| T-01-09 | DoS | Held LoopDetected / GlobalBAFinished events accumulating (viewer) | low | accept | See Accepted Risks Log (AR-03). Delivery tests exist: `LoopCloserTest.DeliversLoopWhileEarlierLoopIsHeld` (loop_closer_test.cpp:777), `GlobalBundleAdjusterTest.DeliversResultWhileEarlierResultIsHeld` (global_bundle_adjuster_test.cpp:525) | closed |
| T-01-10 | Tampering / DoS | Events held past ~Slam, observer destroyed mid-run | high | mitigate | `SlamTest.EventsOutliveSlam` (slam_test.cpp:1844), `SlamTest.ObserverDestroyedWhileSlamRunsIsSafe` (:1904); TSan clean per 01-05 SUMMARY | closed |
| T-01-11 | DoS | Silent disconnect from a discarded Subscription | medium | mitigate | `subscription.h:71` `class [[nodiscard]] Subscription`; `-Werror=unused-result` in `cmake/VoyahTests.cmake:36` (every test binary) and `examples/full_slam/CMakeLists.txt:3,8,17` (both apps) | closed |
| T-01-12 | Repudiation / Tampering | Stale DESIGN text describing pools | medium | mitigate | DESIGN section 6.1 rewritten (`docs/DESIGN.md` lines 35, 507, 598, 750, 770); remaining "pool" hits are historical decisions-log rows and resolved open questions; the 0.4.0 row lists every API break (`docs/DESIGN.md:3475`) | closed |
| T-01-13 | Tampering | Unreviewed change reaching master (R1, 0.4.0) | high | mitigate | Merge `33a270f`; user's literal reply "merge and push" recorded in 01-07 SUMMARY:63; version 0.4.0; voyah_jni message at 01-07 SUMMARY:87 | closed |
| T-01-14 | DoS | Binding crashing on the changed API without notice | medium | mitigate | `conanfile.py` and `project()` at 0.9.0 (minor bumps 0.4.0..0.9.0); 0.4.0 row lists the breaks (`docs/DESIGN.md:3475`); voyah_jni message in 01-07 SUMMARY:87+ | closed |
| T-01-15 | Tampering | Cross-thread reads of map data (data race) | high | mitigate | `map_follower.h:14-32` (own replica, `Update()` on the caller's thread, immutable `Snapshot()`); 8 `MapFollowerTest` cases incl. `SnapshotReadOnAnotherThread`, `SlamTest.MapFollowerMirrorsMapper`; TSan with 0 warnings per 01-08 SUMMARY | closed |
| T-01-16 | DoS | A follower never updated grows its inbox | low | accept | See Accepted Risks Log (AR-04) | closed |
| T-01-17 | Tampering | Data race through Tracker/LoopCloser/GlobalBA replica and stats accessors | high | mitigate | `tracker.h:215-227`, `loop_closer.h:120-143`, `global_bundle_adjuster.h:101-109`: accessors private with friend `*TestAccess`; `global_bundle_adjuster.h:133-134` atomic counters; `thread_affinity_test.cpp:76-93` `static_assert(!Has...)` concept checks; grep finds no stats calls outside `test_access.h` and the concepts | closed |
| T-01-18 | Tampering | Data race through Mapper-thread-only accessors | high | mitigate | `mapper.h:260-290` twelve private accessors behind `MapperTestAccess`; `thread_affinity_test.cpp:95-106` twelve `static_assert(!Has...)` checks; both example apps use `MapFollower` | closed |
| T-01-19 | Tampering | Unreviewed change / unannounced API break (R2, 0.5.0) | high | mitigate | Merge `f1ecb28`; replies "debug it" then "merge and push" (01-11 SUMMARY:95); version 0.5.0; jni message at :130. R2 `-O0` regression fixed: `MapFollower` defined in `libs/slam/src/mapping/map_follower.cpp`, guarded by `BuildCheck.MapFollowerAddsNoLibraryInstantiations` (`libs/slam/tests/CMakeLists.txt:70`) | closed |
| T-01-20 | DoS | Concurrent Stop()/destructor returning before the worker is joined | high | mitigate | `event_node.h StopWith` waits on Starting/Stopping and retries; `slam.cpp Stop()` does the same; `LifecycleTest.ConcurrentStopsReturnOnlyAfterJoin` (lifecycle_test.cpp:168), `SlamTest.LifecycleStopIsIdempotentFromManyThreads` (slam_test.cpp:2006); TSan clean per 01-12 and 01-14 SUMMARY | closed |
| T-01-21 | DoS | Self-join deadlock from Stop() on the component's own thread | medium | mitigate | `worker_thread.cpp Stop` throws `UsageError` when `OnWorkerThread()`; `LifecycleTest.StopFromWorkerThreadThrows` (:202). R3 option (a): `Slam::Stop()` checks `OnComponentThread()` before changing anything; `SlamTest.LifecycleStopFromComponentThreadChangesNothing` (:2079). Reopened by the 2026-10-15 verification (deadlock when another thread is stopping); closed again in 0.9.2: `event_node.h StopWith` checks `on_worker_thread()` before reading the state, every component passes `worker_.OnWorkerThread()`; `LifecycleTest.StopFromWorkerThreadWhileAnotherThreadStopsThrows`, `StopFromWorkerThreadWhileStartingThrows`, `SlamTest.LifecycleComponentStopOnItsThreadWhileSlamStopsThrows`; TSan clean (01-21 SUMMARY) | closed |
| T-01-22 | Tampering | Silent restart of a stopped component reusing stale state | medium | mitigate | `EventNode::StartWith` throws `UsageError` unless Created ("Start() after Stop()"); `slam.cpp Start()` likewise; `LifecycleTest.StartAfterStopThrows` (:116), `SlamTest.LifecycleStartAfterStopThrows` (:1989) | closed |
| T-01-23 | DoS | Intermittent lifecycle race surviving a single test run | medium | mitigate | 01-13 SUMMARY:75-80: 50 repetitions each of `LifecycleTest.*` and `SlamTest.Lifecycle*` under TSan, 0 warnings; 01-14 re-ran the full TSan gate after the option (a) change | closed |
| T-01-24 | Tampering | Unreviewed change / unannounced API break (R3, 0.6.0) | high | mitigate | Merge `124a668`; "merge and push" at 01-14 SUMMARY:105; version 0.6.0; jni message at :138; the user-directed option (a) round was repeated in full | closed |
| T-01-25 | Tampering | Data race on the extractor's pyramid under concurrent DetectAndExtract | high | mitigate | `orb_feature_extractor.cpp:300-308` `DetectAndExtract(..., Workspace&) const`; pyramid lives in the caller's `Workspace` (`orb_feature_extractor.h:22-32`); `OrbExtractorWorkspaceTest.ConcurrentCallsEqualSingleThreadResult` (orb_extractor_workspace_test.cpp:58); TSan clean per 01-15 SUMMARY:157 | closed |
| T-01-26 | Tampering | Workspace built for one image size used on another | medium | mitigate | `orb_feature_extractor.cpp:303-304` `Fits` then `Initialize`; `image_pyramid.cpp:27-37` `Fits` compares size, level count and every scale factor; `WorkspaceReinitialisesOnNewImageSize` (:107), `WorkspaceReinitialisesForAnotherLevelCount` (:118) | closed |
| T-01-27 | Tampering | Unreviewed change / unannounced API break (R4, 0.7.0) | high | mitigate | Merge `823470d`; "merge and push" at 01-16 SUMMARY:65; version 0.7.0; jni message at :91 | closed |
| T-01-28 | Tampering / DoS | Data race on the logger sink list or level; racing register_logger | medium | mitigate | `logger.cpp`: `SharedSink()` is a `dist_sink_mt`, `CurrentLevel()` is atomic, `CreateLogger` holds `creation_mutex` for get-or-create; `LoggerTest.ConcurrentLogAndInitialize` (logger_test.cpp:113), `ConcurrentCreateOfOneNameReturnsOneLogger` (:85); TSan 6/6 clean per 01-17 SUMMARY:77 | closed |
| T-01-29 | Repudiation | Diagnostics lost because early loggers kept stdout after Initialize | low | mitigate | `Logger::Initialize` re-targets the shared sink via `set_sinks`; `LoggerTest.ComponentCreatedBeforeInitializeUsesNewSinks` (logger_test.cpp:35) | closed |
| T-01-30 | Tampering | Unreviewed change / unannounced API break (R5, 0.8.0) | high | mitigate | Merge `93c00ce`; "merge and push" at 01-18 SUMMARY:64; version 0.8.0; jni message at :90 | closed |
| T-01-31 | Tampering / Elevation | Heap overflow from a wrapped allocation size followed by a full-size copy | high | mitigate | `image.h:191-202` `AllocateElements`: size_t overflow checks throwing `std::length_error`; a byte count the allocator cannot serve is reported by `new[]` (class comment `image.h:30-35`); `ImageTest.SizeOverflowThrows` (image_test.cpp:164); ASan clean per 01-19 SUMMARY | closed |
| T-01-32 | Tampering | Out-of-view writes by operator*= on a view; partial copies of float views | medium | mitigate | `image.h:93-135` row-wise operations over `width*C`; `image.h:171-181` `CopyTo` uses `sizeof(T)`; `ImageTest.MultiplyAssignOnViewChangesOnlyTheView`, `CopyToFloatViewCopiesWholeRows`, `ConvertOfViewReadsOnlyTheView` | closed |
| T-01-33 | Tampering | Reads of uninitialised members of a default image | low | mitigate | `image.h:206-209` in-class zero initialisers; `ImageTest.DefaultIsEmpty` (image_test.cpp:30) default-initialises over a 0xAB-filled buffer | closed |
| T-01-34 | Tampering | Unreviewed change / unannounced API break (R6, 0.9.0) | high | mitigate | Merge `9e57bfa`; "merge and push" at 01-20 SUMMARY:72; version 0.9.0; jni message at :124 | closed |
| T-01-SC | Tampering | npm/pip/cargo/conan installs (deduplicated from all 20 plans) | low | accept | See Accepted Risks Log (AR-05). `git diff afe4b70^..HEAD -- conanfile.py` is the single `version` line; no `FetchContent`, `find_package`, `ExternalProject` or `requires` additions in any CMake file | closed |

*Status: open · closed · open — below high threshold (non-blocking)*
*Severity: critical > high > medium > low — only open threats at or above workflow.security_block_on (high) count toward threats_open*
*Disposition: mitigate (implementation required) · accept (documented risk) · transfer (third-party)*

### Release-process mitigations

All six merge commits (`33a270f`, `f1ecb28`, `124a668`, `823470d`, `93c00ce`, `9e57bfa`) exist as merges in `git log --merges`. Each release SUMMARY records the user's literal "merge and push" reply (01-07, 01-11, 01-14, 01-16, 01-18, 01-20). `docs/DESIGN.md` carries a "Package version" row for each of 0.4.0 through 0.9.0 (lines 3475-3487).

### Unregistered flags

None. All 20 SUMMARY `## Threat Flags` sections state "None".

### Informational notes (non-blocking, no threat opened)

- **R1 mono check:** `monocular_full_slam` ended in Initializing after 40 frames and was not compared with a 0.3.0 run. The user accepted this at the checkpoint (01-07 SUMMARY, Deviations).
- **R2 regression:** a binocular tracking regression (header-only `MapFollower` built at `-O0`) that the plan gates missed was found, fixed and guarded by `BuildCheck.MapFollowerAddsNoLibraryInstantiations`.
- **Image size check:** for single-channel types on 64-bit targets the `length_error` check cannot fire; `image.h:30-35` documents that `new[]` rejects the allocation instead, and `SizeOverflowThrows` covers both paths.
- **Sanitizer evidence:** ASan/TSan results are as recorded in the plan SUMMARY files; this audit did not re-run them.

---

## Accepted Risks Log

| Risk ID | Threat Ref | Rationale | Accepted By | Date |
|---------|------------|-----------|-------------|------|
| AR-01 | T-01-04 | Unbounded inbox growth from a stalled consumer: visibility only in this phase (D-24 backlog warning, implemented in `inbox_core.h WarnIfBehind`, tested). Bounded or lossy delivery is EVT-01, deferred to Phase 5 by user decision. Documented: `docs/DESIGN.md` "Backlog warning (D-24)" lines 750-757; `docs/TODO.md:247` (EVT-01) | Plan 01-01 (user decision, deferral of EVT-01) | 2026-10-13 |
| AR-02 | T-01-06 | `LoopDetected` / `GlobalBAFinished` published unbounded after the drop limits were removed; both producers are paced by Mapper transactions and `max_query_backlog`; the D-24 warning gives visibility. Documented: `docs/DESIGN.md` "Unbounded producers (D-25)" lines 759-762 and the 2026-10-13 decisions row (line 3473, item 4) | Plan 01-02 (user decision D-25) | 2026-10-13 |
| AR-03 | T-01-09 | Held `LoopDetected` / `GlobalBAFinished` events accumulating in a stalled viewer: paced by Mapper transactions; a stalled viewer holds only the events it has not drained and logs the backlog warning; real back-pressure is EVT-01 (Phase 5). Documented: `docs/DESIGN.md:906-909`, D-25 (lines 759-762), `docs/TODO.md:247` | Plan 01-04 | 2026-10-13 |
| AR-04 | T-01-16 | A `MapFollower` that is never updated lets its inbox grow; visible through the "consumer is behind" warning, where the inbox is named "MapFollower" (`map_follower.cpp:13`). Documented in the class comment `map_follower.h:30-32` | Plan 01-08 | 2026-10-13 |
| AR-05 | T-01-SC | Package installs: no package is added or upgraded in this phase; `conan create` builds only pinned, already-cached dependencies. Verified by the `conanfile.py` diff (version line only) and the absence of new dependency declarations in CMake. Documented in every plan's `<threat_model>` and RESEARCH "Package Legitimacy Audit" | Plans 01-01..01-20 | 2026-10-13 |

*Accepted risks do not resurface in future audit runs.*

---

## Security Audit Trail

| Audit Date | Threats Total | Closed | Open | Run By |
|------------|---------------|--------|------|--------|
| 2026-10-15 | 35 | 35 | 0 | gsd-security-auditor (ASVS L1, block_on high) |

---

## Sign-Off

- [x] All threats have a disposition (mitigate / accept / transfer)
- [x] Accepted risks documented in Accepted Risks Log
- [x] `threats_open: 0` confirmed
- [x] `status: verified` set in frontmatter

**Approval:** verified 2026-10-15

## Security Audit 2026-10-15

| Metric | Count |
|---|---|
| Threats found | 35 |
| Closed | 35 |
| Open | 0 |
