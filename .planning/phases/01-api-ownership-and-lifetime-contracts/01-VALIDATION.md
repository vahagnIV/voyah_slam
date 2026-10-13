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
