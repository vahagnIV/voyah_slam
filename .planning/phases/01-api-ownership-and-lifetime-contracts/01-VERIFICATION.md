---
phase: 01-api-ownership-and-lifetime-contracts
verified: 2026-10-15T12:00:00Z
status: gaps_found
score: 6/7 must-haves verified
covered_files:
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-01-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-01-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-02-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-02-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-03-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-03-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-04-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-04-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-05-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-05-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-06-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-06-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-07-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-07-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-08-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-08-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-09-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-09-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-10-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-10-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-11-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-11-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-12-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-12-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-13-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-13-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-14-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-14-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-15-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-15-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-16-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-16-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-17-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-17-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-18-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-18-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-19-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-19-SUMMARY.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-20-PLAN.md
  - .planning/phases/01-api-ownership-and-lifetime-contracts/01-20-SUMMARY.md
  - examples/full_slam/binocular_full_slam.cpp
  - examples/full_slam/monocular_full_slam.cpp
  - libs/algs/src/features/orb_feature_extractor.cpp
  - libs/common/include/voyahslam/features/orb_feature_extractor.h
  - libs/common/include/voyahslam/images/image.h
  - libs/common/src/logging/logger.cpp
  - libs/events/include/voyahslam/events/event_node.h
  - libs/events/include/voyahslam/events/inbox_core.h
  - libs/events/include/voyahslam/events/observable.h
  - libs/events/include/voyahslam/events/subscription.h
  - libs/events/src/events/worker_thread.cpp
  - libs/slam/include/voyahslam/mapping/map_follower.h
  - libs/slam/include/voyahslam/mapping/mapper.h
  - libs/slam/include/voyahslam/slam/slam.h
  - libs/slam/include/voyahslam/tracking/tracker.h
  - libs/slam/src/mapping/map_follower.cpp
  - libs/slam/src/slam/slam.cpp
covered_digest: "v3:sha256:cb774d955e4e4f4a1cde79b6e250c405d74a1741aec52a8e584c164e16a22955"
behavior_unverified: 0
overrides_applied: 0
gaps:
  - truth: "Stop() called on a component's own worker thread throws UsageError instead of self-joining (01-12 must-have; threat T-01-21 'self-join deadlock', recorded closed in 01-SECURITY.md; DESIGN exception policy; ROADMAP SC4 'invalid lifecycle transitions fail loudly')"
    status: failed
    severity: major
    reason: "The own-thread refusal lives only in WorkerThread::Stop(), which EventNode::StopWith reaches only in its Running case. If another thread has already moved the node Running -> Stopping and is joining the worker, a Stop() issued from the worker thread (a sync handler or callback) reads Stopping, calls WaitWhileState(Stopping), and waits for the thread that is joining it. Neither returns. Reproduced with a standalone program built against the Debug libvoyah_events.a (scratchpad/v1/wr01.cpp): 'DEADLOCK: Stop() did not return in 3 s'. Slam::Stop() is protected (it checks OnComponentThread() first), but Tracker/Mapper/LoopCloser/GlobalBA::Stop() are public through slam.tracker() etc. LifecycleTest.StopFromWorkerThreadThrows covers only the non-concurrent case."
    artifacts:
      - path: "libs/events/include/voyahslam/events/event_node.h"
        issue: "StopWith() (lines ~102-125): no own-thread check in the Starting/Stopping branch."
      - path: "libs/events/tests/events/lifecycle_test.cpp"
        issue: "No test for 'worker calls Stop() while another thread is inside Stop()'."
    missing:
      - "Give StopWith an own-thread predicate (for example StopWith(stop_worker, on_worker_thread)) and throw UsageError before the switch, whatever the state; each component passes [this]{ return worker_.OnWorkerThread(); }. Slam::Stop() already does the equivalent."
      - "Add LifecycleTest.StopFromWorkerThreadWhileAnotherThreadStopsThrows: the worker blocks in a handler until run_state()==Stopping, then calls Stop(); expect UsageError and that the main Stop() returns (use a watchdog timeout)."
      - "Update 01-SECURITY.md T-01-21 evidence after the fix."
  - truth: "Thread-affine drivers cannot be reached from an app thread while the component runs (ROADMAP SC3 / audit #5: 'never while running' entry points enforced)"
    status: partial
    severity: minor
    reason: "The named accessors (atlas(), map_replica(), last_matches(), the twelve Mapper accessors, the LoopCloser/GlobalBA ones) are private behind *TestAccess and guarded by static_assert concepts (verified). Tracker/Mapper/LoopCloser/GlobalBA::ProcessPending() remain public with only a comment 'never concurrently with a running worker'; nothing enforces it in any build. A consumer calling it on a running component races with the worker on the replica, last_frame_ and batch_. (Review item IN-07.)"
    artifacts:
      - path: "libs/slam/include/voyahslam/tracking/tracker.h"
        issue: "ProcessPending() public (line ~201)."
      - path: "libs/slam/include/voyahslam/mapping/mapper.h"
        issue: "ProcessPending() public (line ~203)."
    missing:
      - "Make ProcessPending() private behind the existing *TestAccess friends (the worker lambda is a member and can call it), or throw UsageError when run_state() is Running and !OnWorkerThread()."
deferred: []
advisory:
  - finding: "CR-01: OrbFeatureExtractor::ComputePointsPerLevel divides by zero (SIGFPE) in release builds for images narrower or shorter than about 233 px (n_cols/n_rows == 0 is guarded only by assert); levels == 0 is also unguarded."
    category: other
    reason: "Verified by reading (orb_feature_extractor.cpp:561-567, unchanged since the first commit 0fefefa, 2026-09-30). It is input validation, not declaration order, destruction order, lifetime or wrong-thread use, so it is outside the Phase 1 goal and OWN-05 (which is concurrency). It is a real crash for a JNI caller with a 224x224 image. Not in docs/TODO.md. Recommend: throw std::invalid_argument in DetectAndExtract when the last pyramid level is below PATCH_SIZE + 35 = 66 px, add a small-image test, and record in TODO if not fixed now. Severity high, scope out-of-phase."
    evidence_status: "read, not executed"
  - finding: "CR-02: images::Image view constructor, operator[] and CopyTo are unchecked in release builds (no start+size <= parent bounds check; CopyTo size match is assert-only, heap overflow on mismatch)."
    category: other
    reason: "Verified by reading (image.h:50-55, 70-78, 168-184). OWN-07's acceptance (tests on a sub-image view; default init; view-correct Convert/operator*/operator*=; non-const Set; aliasing documented) is met; bounds validation was not part of OWN-07 or the goal. Severity high for the JNI caller, scope out-of-phase. Recommend: throw std::out_of_range in the view constructor and std::invalid_argument in CopyTo, with tests, or record in TODO."
    evidence_status: "read, not executed"
  - finding: "WR-02: no try/catch around the worker loop; an exception from ProcessPending or a sync handler is std::terminate with no log, and a throwing sync handler skips async delivery in Observable::Notify."
    category: other
    reason: "Pre-existing since 2026-10-07 (worker_thread.cpp), not a Phase 1 regression; DESIGN exception policy does not claim otherwise. Recommend log-critical-and-stop in the worker body and a DESIGN section 7 line."
    evidence_status: "read"
  - finding: "WR-03: MapFollower thread ownership is assert-only (debug builds) and owner_ is a non-atomic std::thread::id."
    category: other
    reason: "OWN-03 and SC3 explicitly accept 'test access or a debug thread assert', and map_follower.h documents it. Release-build misuse (Snapshot() from a second thread while Update() runs) is a data race. Recommend atomic owner_ with compare_exchange and UsageError in every build (one atomic load per call)."
    evidence_status: "read"
  - finding: "WR-04: AddObserver/Subscription::reset() check run state and then mutate the wiring vectors non-atomically; a Start() on another thread can win the Created -> Starting CAS in between and the worker then reads vectors being edited."
    category: other
    reason: "Needs two caller threads racing connect against Start, which the documented rule (connect before Start) forbids but does not enforce. Not reproduced. Recommend an 'Editing' claim state, or document that connection edits must be ordered before Start() by the caller."
    evidence_status: "read, not reproduced"
  - finding: "WR-05: Slam::Start() rollback and Slam::Stop() partial-failure paths can leave state_ Starting (rollback Stop() throws) or revert to Running after the Tracker is already Stopped."
    category: other
    reason: "Needs a throwing Stop() (failed join), not reachable in normal operation. Recommend per-component try/catch in the rollback and always ending in Stopped."
    evidence_status: "read, not reproduced"
  - finding: "WR-06: both example apps subscribe a MapFollower to the Mapper but call follower.Update() only once, after slam.Stop(); the follower inbox then holds every MapTransaction for the whole run (pools are gone, so nothing bounds it) and logs 'consumer is behind' warnings."
    category: other
    reason: "Verified by reading (binocular_full_slam.cpp: follower.inbox() subscribed, single Update() after Stop). The examples are what users copy. Does not break any success criterion. Recommend calling follower.Update() next to trajectory->Poll() in the frame loop."
    evidence_status: "read; no long-run measurement"
---

# Phase 1: API Ownership and Lifetime Contracts Verification Report

**Phase Goal:** A library consumer whose objects die in any order (the Java/JNI binding in `../voyah_jni`, or any C++ app) cannot crash the library through declaration order, destruction order or calls from the wrong thread: every type owns or shares what it points to after a call returns, and every required ordering is removed or enforced by an assert or an error.
**Verified:** 2026-10-15 (master, HEAD 9b80b9c; no source change since the R6 gate head 18f2984, the commits after it are docs only)
**Status:** gaps_found
**Re-verification:** No, initial verification

## Goal Achievement

Five of the six roadmap success criteria are met with code and test evidence. One must-have is falsified: the documented "Stop() on a component's own worker thread throws instead of self-joining" guarantee does not hold when a second thread is already stopping the component. That is a wrong-thread call that deadlocks instead of failing loudly, which is the exact failure class the goal names. I reproduced it. The fix is small and local. Everything else raised by the code review is either pre-existing input validation outside this phase's goal (CR-01, CR-02) or a narrow, advisory robustness issue.

### Observable Truths

| #   | Truth | Status | Evidence |
| --- | ----- | ------ | -------- |
| SC1 | An event, `Listener` or `AsyncObserver` that outlives its producer stays valid and is freed once; events are `shared_ptr<const T>`; `ObjectPool`/`Recyclable` gone; `global_ba_` invariant and `StopAll` gone | VERIFIED | `git grep` for `ObjectPool\|Recyclable\|StopAll\|event_pool_size\|transaction_pool_size\|max_events_in_flight\|num_dropped_loops\|num_results_dropped\|SetWakeSignal` over libs, examples, vizualization, test_package prints nothing (G1). `slam.h` carries no `global_ba_` ordering comment (members are plain, `subscriptions_` is last). `Observable::Notify(std::shared_ptr<T>&&)` with `EventPtr = shared_ptr<const Event>`. Ran `voyah_events_tests` (81/81 pass) and `SlamTest.EventsOutliveSlam`, `StuckObserverHoldsEveryTrackerEvent` (pass). ASan on events tests passes in the R6 gate log. |
| SC2 | Observer destroyed before its producer never notified; connect/disconnect while running throws; examples do not depend on viewer/TrajectoryWriter declaration order | VERIFIED (with accepted residual) | `AllOrders/DestructionOrderTest.AnyOrderIsSafe/*` (24 orders), `SubscriptionTest.*`, `SlamTest.ObserverDestroyedWhileSlamRunsIsSafe`, `AddObserverWhileRunningThrows` pass. Observers (viewer, `TrajectoryWriter`, follower) can be destroyed in any order (shared `InboxCore`). Residual by user decision D-07/D-23: a `Subscription` destroyed while its producer runs logs and terminates; the examples therefore declare the `subscriptions` vector before `slam`. That is an enforced, loud, documented ordering, matching the goal's "enforced by an assert or an error", but it is not "any order" for `Subscription` objects. Recorded here for transparency, not as a gap. |
| SC3 | "Mapper thread only"/"never while running" accessors cannot be reached from an app thread while running; apps have a supported way to read the map | VERIFIED (minor gap on `ProcessPending`) | Accessors private behind `*TestAccess`; `thread_affinity_test.cpp` `static_assert(!Has...<T>)` concepts for `atlas`, `map_replica`, `last_matches` and the Mapper/LoopCloser/GlobalBA set (ran `ThreadAffinityTest.*`, `MapFollowerTest.*`: 11/11 pass). `mapping::MapFollower` + `MapSnapshot` is the supported read path; `SlamTest.MapFollowerMirrorsMapper` passes. Gap: public `ProcessPending()` is unenforced (see gaps, minor). |
| SC4 | Invalid lifecycle transitions (`AddObserver` while running, `Start` twice, `Start` after `Stop`, `Track()` after `Stop()`) throw `UsageError`; `Track()` before `Start()` queues; Stopped terminal and documented | VERIFIED | `event_node.h` CAS state machine; `LifecycleTest.*` (12) and `SlamTest.Lifecycle*` pass; `TrackerTest.TrackBeforeStartQueues`/`TrackAfterStopThrows` pass; DESIGN section 4 "Lifecycle" and the decisions log record "Stop() then Start() is not supported". |
| SC4b | (01-12 must-have, T-01-21) `Stop()` on a component's own worker thread throws `UsageError` instead of self-joining | FAILED | Deadlocks when another thread is already in `Stop()`. Reproduced: standalone binary linked to `libvoyah_events.a`, worker waits for `run_state()==Stopping` then calls `Stop()`, main calls `Stop()`: watchdog fires after 3 s, exit 42. See gaps. |
| SC5 | Two threads may call `DetectAndExtract` concurrently; `Logger::Initialize` re-targets earlier loggers; default `Image` empty; view ops honour `offset_`/`stride_` | VERIFIED | `DetectAndExtract(image, n, Workspace&) const` with no mutable member (header lines 26-41); ran `OrbExtractorWorkspaceTest.*` 5/5 pass, R6 gate shows TSan-clean and the golden test `OrbExtractorGoldenTest.Corridor1OutputUnchanged` PASSED (ran, not skipped). Logger: all loggers write through one `dist_sink_mt` (`logger.cpp` `SharedSink()`/`set_sinks`); `LoggerTest.ComponentCreatedBeforeInitializeUsesNewSinks` etc. pass. Image: members default-initialised, `Convert`/`operator*`/`operator*=`/`CopyTo` row-wise with `offset_`, `stride_`; `ImageTest.*` 14/14 pass; ASan clean. |
| SC6 | Each item has a DESIGN decisions-log entry; `check_all` green; TSan clean on event and Slam tests; Conan packages recreated with a version the voyah_jni session is told about | VERIFIED | DESIGN.md has decisions-log rows for audit #3+#2, #5, #6, #7, #8, #9 and package versions 0.4.0 to 0.9.0 (lines ~3472-3487). R6 gate logs: `check_all` 852/852 (26 s), TSan events 80, slam 196, integration 36 passed, ASan events 81 and image 14 passed, Conan static, shared and android-arm64 `voyahslam/0.9.0` built. `conanfile.py` version 0.9.0. 01-20-SUMMARY contains "Message for the voyah_jni session"; merge and push recorded (9e57bfa). Apps: R6 gate binocular run ends at "map: 3 Frames, 777 points" with no terminate; monocular exits cleanly in the known WaitingForMap state. I did not re-run the TSan/ASan/Conan gates; source is unchanged since the gate head. |

**Score:** 6/7 truths verified (5 of 6 roadmap criteria plus the 01-12 own-thread must-have; 0 present-but-behavior-unverified)

### Requirements Coverage

| Requirement | Source Plan(s) | Description | Status | Evidence |
| ----------- | -------------- | ----------- | ------ | -------- |
| OWN-01 | 01-01..01-07 | shared_ptr events, pools, `global_ba_` invariant and `StopAll` removed | SATISFIED | SC1 evidence; REQUIREMENTS.md marks Complete |
| OWN-02 | 01-01..01-07 | RAII `Subscription`, loud failure on connect/disconnect while running | SATISFIED | SC2 evidence (24-order matrix, death test, throws tests) |
| OWN-03 | 01-08..01-11 | Thread-affine accessors enforced; supported map read path | SATISFIED, minor residual | SC3; `ProcessPending()` still public and unenforced |
| OWN-04 | 01-12..01-14 | Lifecycle state machine, errors on invalid transitions, restart decided | PARTIAL | SC4 verified for the listed transitions; the own-thread `Stop()` refusal fails under concurrent Stop (gap 1) |
| OWN-05 | 01-15 | `DetectAndExtract` concurrency-safe via caller workspace | SATISFIED | SC5; concurrent-call test equals single-thread result |
| OWN-06 | 01-16..01-17 | Logger re-targets earlier loggers, sinks protected | SATISFIED | SC5; `dist_sink_mt` plus function-local atomics/mutex |
| OWN-07 | 01-18..01-20 | `Image` default init, view-correct ops, non-const `Set`, aliasing documented | SATISFIED | SC5; bounds validation of views is outside the requirement (see advisory CR-02) |

All seven IDs appear in the plan frontmatter and in REQUIREMENTS.md (all marked Complete). No orphaned requirements: the traceability table maps exactly OWN-01..OWN-07 to Phase 1.

### Behavioral Spot-Checks

| Behavior | Command | Result | Status |
| -------- | ------- | ------ | ------ |
| Events layer, 24 destruction orders, lifecycle | `voyah_events_tests` (Debug, `-O2 -g`, asserts on) | 81 tests passed | PASS |
| Logger and Image | `voyah_common_tests --gtest_filter='LoggerTest.*:ImageTest.*'` | 20 passed | PASS |
| Concurrent extraction | `voyah_algs_tests --gtest_filter='OrbExtractorWorkspaceTest.*'` | 5 passed | PASS |
| Thread affinity, follower, tracker queueing | `voyah_slam_tests` filter `ThreadAffinityTest.*:MapFollowerTest.*:TrackerTest.TrackBeforeStartQueues:TrackAfterStopThrows` | 11 passed | PASS |
| Slam-level lifetime and lifecycle | `voyah_integration_tests` filter `SlamTest.EventsOutliveSlam:Lifecycle*:ObserverDestroyedWhileSlamRunsIsSafe:UnwindPast...:AddObserverWhileRunningThrows:StuckObserver...:MapFollowerMirrorsMapper` | 14 passed | PASS |
| Own-thread `Stop()` under a concurrent `Stop()` | scratchpad `wr01.cpp` against `libvoyah_events.a` | deadlock, watchdog exit 42 | FAIL |
| Removed symbols stay removed (G1) | `git grep -E 'ObjectPool\|Recyclable\|StopAll\|...' -- libs examples vizualization test_package` | empty | PASS |

The test binaries predate the HEAD docs commits; `git diff 18f2984 HEAD -- libs examples vizualization cmake conanfile.py CMakeLists.txt test_package` is empty, so they correspond to the current sources.

### Anti-Patterns Found

| File | Line | Pattern | Severity | Impact |
| ---- | ---- | ------- | -------- | ------ |
| `libs/events/include/voyahslam/events/event_node.h` | ~102-125 | own-thread check missing in the Starting/Stopping branch | Major | Deadlock instead of `UsageError` (gap 1) |
| `libs/algs/src/features/orb_feature_extractor.cpp` | 561-567 | assert-only guard before integer division | High, out of phase scope | SIGFPE on small images in release (advisory CR-01) |
| `libs/common/include/voyahslam/images/image.h` | 50-55, 168-184 | assert-only or absent bounds checks on views and `CopyTo` | High, out of phase scope | OOB read/write for caller-supplied sizes (advisory CR-02) |
| `libs/slam/include/voyahslam/tracking/tracker.h`, `mapper.h` | ~201, ~203 | public driver with comment-only restriction | Minor | Gap 2 |
| `examples/full_slam/*_full_slam.cpp` | 66-70, 128-133 | follower inbox never drained during the run | Minor | Unbounded inbox in the example (advisory WR-06) |

A scan of the 81 library, example and viewer source files changed since Phase 1 began found no TBD, FIXME or XXX markers. No stub or placeholder implementations were found in the Phase 1 artifacts read (event layer, Slam, follower, image, logger, workspace).

### Human Verification Required

None new. The validation strategy's manual-only items G1..G8 were accepted by the user on 2026-10-15; G1, G3 (binocular tracked-frames bar), G4 (golden test ran) and G8 (voyah_jni message) are covered by the evidence above. The packaging and lock-freedom review (G5) and the "views alias the parent buffer" documentation (G6) remain reviewer-judgement items already accepted as manual-only.

### Gaps Summary

**Gap 1 (major, closes the goal): own-thread `Stop()` deadlocks under a concurrent `Stop()`.** Phase 1's goal includes "calls from the wrong thread" and "every required ordering ... enforced by an error". The phase promised, in code comments, DESIGN's exception policy, plan 01-12 and threat T-01-21, that a component's own worker calling `Stop()` gets `UsageError`. That holds only if the node is still Running. In the Stopping state the worker waits on the thread that is joining it. A JNI callback running on the Tracker or Mapper thread that calls `slam.tracker().Stop()` while Java's `close()` is stopping the Slam hits it. Fix: pass an own-thread predicate into `EventNode::StopWith` and throw before the state switch (about 10 lines in `event_node.h` plus a one-line change in each of Tracker, Mapper, LoopCloser, GlobalBA `Stop()`), add the concurrent test, and update the T-01-21 evidence in 01-SECURITY.md. A gap-closure plan of one task suffices.

**Gap 2 (minor): public `ProcessPending()`** on the four components is the one thread-affine entry point left open. Move it behind the `*TestAccess` friends or throw while Running.

**Decision requested for CR-01 and CR-02.** I classified both as outside the Phase 1 goal: they are pre-existing input-validation holes (CR-01 dates from the first commit; CR-02 is in `Image` view bounds, which OWN-07 never required), not ordering, lifetime or wrong-thread defects, and the user already scoped OWN-07 to the acceptance in REQUIREMENTS.md. They are still real crashes for a JNI caller (a 224x224 image kills the process). The cheapest honest options: (a) add both fixes to the gap-closure plan (about 15 lines each plus tests), or (b) add them to `docs/TODO.md` and Phase 4 (EXTR-01 already touches the 19-px border handling) and leave Phase 1 as is. They are listed in the `advisory` block, not as gaps, because deferring them does not contradict the stated goal. If the user reads the goal as "the library cannot be crashed by a JNI caller", promote them to gaps.

The code-review warnings WR-02 to WR-06 are advisory: each is either pre-existing, accepted by an explicit requirement wording (WR-03: "debug thread assert"), or needs a failure that normal operation does not produce. WR-06 is the one worth doing alongside the closure plan because it is a one-line change in the example that users copy.

---

_Verified: 2026-10-15_
_Verifier: Claude (gsd-verifier)_
