---
phase: "1"
slug: "api-ownership-and-lifetime-contracts"
# status lifecycle: draft (seeded by plan-phase) → validated (set by validate-phase §6)
# audit-milestone §5.5 distinguishes NOT-VALIDATED (draft) from PARTIAL (validated + nyquist_compliant: false) (#2117)
status: draft
nyquist_compliant: false
wave_0_complete: false
created: "2026-10-12"
---

# Phase 1 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | GoogleTest 1.15.0 via `gtest_discover_tests` (labels `unit` / `integration`) |
| **Config file** | `cmake/VoyahTests.cmake`; per-library `libs/<lib>/tests/CMakeLists.txt` |
| **Quick run command** | `cmake --build cmake-build-debug/build/Debug --target check` |
| **Full suite command** | `cmake --build cmake-build-debug/build/Debug --target check_all` |
| **Sanitizer gate** | TSan tree `cmake-build-tsan`; every TSan binary runs under `setarch -R` (else `FATAL: ThreadSanitizer: unexpected memory mapping`); `*DeathTest` excluded from TSan runs |
| **Release gate** | `conan create . --build=missing -pr profiles/linux-gcc13` (static), same with `-o shared=True`, and `-pr:h profiles/android-arm64 -pr:b default` |
| **Estimated runtime** | ~10 s (`check`), ~30 s (`check_all`) plus build |

---

## Sampling Rate

- **After every task commit:** the touched test binary with a `--gtest_filter`, then `--target check`
- **After every plan wave:** `--target check_all`
- **Before each release merge:** `check_all` green, TSan clean on `voyah_events_tests`, `voyah_slam_tests` (Tracker/Mapper/Slam/GlobalBundle/LoopCloser/MapFollower filters) and `voyah_integration_tests`, three `conan create` runs, the user's explicit "merge and push"
- **Max feedback latency:** 60 seconds

---

## Per-Task Verification Map

Filled by the planner per task; requirement → test map (from RESEARCH.md "Validation Architecture"):

| Requirement | Behavior | Test Type | Automated Command | File Exists |
|-------------|----------|-----------|-------------------|-------------|
| OWN-01 | Received event immutable by type; `Notify` rejects an lvalue | compile-time | build of `voyah_events_tests` (`static_assert`) | ❌ W0 `libs/events/tests/events/event_lifetime_test.cpp` |
| OWN-01 | Event held by a consumer outlives producer and observer; freed exactly once, on the last drop's thread | unit + ASan/TSan | `voyah_events_tests --gtest_filter='EventLifetimeTest.*'` | ❌ W0 (same file) |
| OWN-01 | Slam destroyed while an observer still holds events: events valid, nothing dropped | integration | `voyah_integration_tests --gtest_filter='SlamTest.EventsOutliveSlam*'` | extend `slam_test.cpp` |
| OWN-01 | No `ObjectPool`, `Recyclable`, `StopAll`, pool-size options or overflow counters left | review | `grep -rn` over `libs/ examples/ vizualization/` returns nothing | n/a |
| OWN-02 | All 24 destruction orders of {producer, subscription, observer, listener} safe; every event freed once | unit + ASan/TSan | `voyah_events_tests --gtest_filter='DestructionOrderTest/*'` | ❌ W0 `libs/events/tests/events/subscription_test.cpp` |
| OWN-02 | Observer destroyed first never notified; `~AsyncObserver` releases queued events | unit | `--gtest_filter='SubscriptionTest.*:AsyncObserverTest.*'` | ❌ W0 (same file) |
| OWN-02 | Connect/`reset()` while running throws; Subscription destroyed while running terminates | unit + death | `--gtest_filter='SubscriptionDeathTest.*'` (not under TSan) | ❌ W0 (same file) |
| OWN-03 | Restricted accessors only through `*TestAccess`; `MapFollower` mirrors the Mapper map | compile + unit + TSan | `voyah_slam_tests --gtest_filter='MapFollowerTest.*'` | ❌ W0 `libs/slam/tests/mapping/map_follower_test.cpp` |
| OWN-04 | Invalid transitions throw/assert; concurrent `Stop` joins | unit + TSan | `--gtest_filter='LifecycleTest.*:SlamTest.*Lifecycle*'` | ❌ W0 `libs/events/tests/events/lifecycle_test.cpp` |
| OWN-05 | Concurrent `DetectAndExtract` equals single-thread result; goldens unchanged | unit + TSan | `voyah_algs_tests --gtest_filter='*OrbExtractor*:*Workspace*'` | ❌ W0 |
| OWN-06 | Loggers created before `Initialize` re-targeted; concurrent create/log/Initialize | unit | `voyah_common_tests --gtest_filter='LoggerTest.*'` | ❌ W0 `libs/common/tests/logging/logger_test.cpp` |
| OWN-07 | Default `Image` empty; view ops honour `offset_`/`stride_`; overflow guarded | unit | `voyah_common_tests --gtest_filter='ImageTest.*'` | ❌ W0 `libs/common/tests/images/image_test.cpp` |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

### Per-task map (filled by the planner, 2026-10-13)

Every task's `<verify>` carries an `<automated>` command with a `<fails_when>`; this is the index.

| Plan / Task | Requirement | Automated command (abridged) | Status |
|-------------|-------------|------------------------------|--------|
| 01-01 T1 (tracer) | OWN-01, OWN-02 | `--target voyah_events_tests` then the whole binary (EventLifetimeTest + ported tests) | ⬜ |
| 01-01 T2 | OWN-02 | `voyah_events_tests --gtest_filter='DestructionOrderTest/*:SubscriptionTest.*:SubscriptionDeathTest.*:AsyncObserverTest.*'`; TSan (`setarch -R`, `-*DeathTest*`); ASan | ⬜ |
| 01-02 T1..T3 | OWN-01, OWN-02 | per-object `make -C .../libs/slam src/...cpp.o`; `--target voyah_slam`; `--target voyahviz monocular_full_slam binocular_full_slam`; both apps run 40 corridor1 frames (`--rate=0 --max-frames=40`, "tracker state:" line, no terminate) | ⬜ |
| 01-03 T1, T2 | OWN-01, OWN-02 | per-object `make -C .../libs/slam/tests tracking/... mapping/mapper_test.cpp.o`; assertion-preservation check (EXPECT_/ASSERT_ multiset per file equals `git merge-base HEAD master`) | ⬜ |
| 01-04 T1 | OWN-01 (D-25) | per-object build of loop/GBA/replica/matcher tests; assertion-preservation check (the two replaced drop tests set aside) | ⬜ |
| 01-04 T2 | OWN-02 | `voyahviz_tests` (whole binary); assertion-preservation check | ⬜ |
| 01-05 T1 | OWN-01, OWN-02 | `voyah_integration_tests --gtest_filter='SlamTest.EventsOutliveSlam:SlamTest.StuckObserver*:SlamTest.ObserverDestroyed*:SlamTest.Unwind*:SlamTest.AddObserverWhileRunningThrows'` + TSan; assertion-preservation check on slam_test.cpp (seven named tests set aside) | ⬜ |
| 01-05 T2 | OWN-01 | `--target check_all`; negated `git grep` for pool symbols | ⬜ |
| 01-06 T1, T2 | OWN-01, OWN-02 | DESIGN/version greps; `--target check` | ⬜ |
| 01-07 T1 | OWN-01, OWN-02 | check_all, apps 40-frame smoke, TSan x3, ASan, removed-symbol grep, three `conan create` | ⬜ |
| 01-08 T1 (tracer), T2 | OWN-03 | `voyah_slam_tests --gtest_filter='MapFollowerTest.*'`, `voyah_integration_tests --gtest_filter='SlamTest.MapFollowerMirrorsMapper'`; TSan | ⬜ |
| 01-09 T1, T2 | OWN-03 | `ThreadAffinityTest.*` (concept checks incl. the LoopCloser stats accessors), `check_all`; assertion-preservation check; negated `git grep` for LoopCloser stats calls outside its TestAccess | ⬜ |
| 01-10 T1, T2 | OWN-03 | `check_all`; assertion-preservation check; DESIGN/version greps | ⬜ |
| 01-11 T1 | OWN-03 | release gates + `ThreadAffinityTest.*:MapFollowerTest.*` | ⬜ |
| 01-12 T1 (tracer), T2 | OWN-04 | `LifecycleTest.*`, `TrackerTest.Track*`, `check_all`, TSan `SlamTest.Lifecycle*` | ⬜ |
| 01-13 T1, T2 | OWN-04 | DESIGN/version greps and the D-12 wording grep in `.planning/ROADMAP.md` / `REQUIREMENTS.md`; TSan `--gtest_repeat=50` on LifecycleTest and SlamTest.Lifecycle* | ⬜ |
| 01-14 T1 | OWN-04 | release gates + `LifecycleTest.*` | ⬜ |
| 01-15 T1 (tracer) | OWN-05 | `voyah_algs_tests --gtest_filter='OrbExtractorWorkspaceTest.*'`, `voyah_algs_dataset_tests --gtest_filter='OrbExtractorGoldenTest.*'` (not SKIPPED), TSan | ⬜ |
| 01-16 T1 | OWN-05 | release gates + golden digests | ⬜ |
| 01-17 T1 (tracer), T2 | OWN-06 | `voyah_common_tests --gtest_filter='LoggerTest.*'` + TSan | ⬜ |
| 01-18 T1 | OWN-06 | release gates + LoggerTest | ⬜ |
| 01-19 T1 (tracer), T2 | OWN-07 | `voyah_common_tests --gtest_filter='ImageTest.*'` + ASan + golden digests | ⬜ |
| 01-20 T1 | OWN-07 | release gates + ImageTest under ASan | ⬜ |

Notes: until plan 01-05 the `voyah_slam` test binaries do not link (expected), so R1 plans 01-02..01-04 verify per object file; `check` / `check_all` build every test binary and are used from 01-05 on. Plans run sequentially in the main working tree (one in-tree build directory and one release branch at a time).

---

## Wave 0 Requirements

- [ ] `libs/events/tests/events/subscription_test.cpp` — OWN-02 (`DestructionOrderTest`, `SubscriptionTest`, `SubscriptionDeathTest`)
- [ ] `libs/events/tests/events/event_lifetime_test.cpp` — OWN-01
- [ ] Test-only live-counting event (`Probe`) in a shared test header — OWN-01/02
- [ ] `libs/events/tests/events/lifecycle_test.cpp` — OWN-04
- [ ] `libs/common/tests/images/image_test.cpp` — OWN-07
- [ ] `libs/common/tests/logging/logger_test.cpp` — OWN-06
- [ ] `libs/slam/tests/mapping/map_follower_test.cpp` + shared `*TestAccess` header — OWN-03
- [ ] Workspace/concurrency test in `libs/algs/tests/features/` — OWN-05
- [ ] Death-test convention (`threadsafe` style, `*DeathTest` names, excluded from TSan)

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| "merge and push" per release | all | the user's explicit approval | wait for the user's words |
| voyah_jni session told the new version | all | cross-repo communication | message the voyah_jni session with the version and API changes |

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 60s
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
