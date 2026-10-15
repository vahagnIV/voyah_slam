---
phase: 01-api-ownership-and-lifetime-contracts
reviewed: 2026-10-15T00:00:00Z
depth: standard
files_reviewed: 87
files_reviewed_list:
  - CMakeLists.txt
  - cmake/VoyahTests.cmake
  - conanfile.py
  - examples/full_slam/CMakeLists.txt
  - examples/full_slam/binocular_full_slam.cpp
  - examples/full_slam/full_slam_common.h
  - examples/full_slam/monocular_full_slam.cpp
  - libs/algs/src/features/orb/image_pyramid.cpp
  - libs/algs/src/features/orb_feature_extractor.cpp
  - libs/algs/tests/CMakeLists.txt
  - libs/algs/tests/features/orb_extractor_workspace_test.cpp
  - libs/common/CMakeLists.txt
  - libs/common/include/voyahslam/errors.h
  - libs/common/include/voyahslam/features/orb/image_pyramid.h
  - libs/common/include/voyahslam/features/orb_feature_extractor.h
  - libs/common/include/voyahslam/images/image.h
  - libs/common/include/voyahslam/logging/logger.h
  - libs/common/src/logging/logger.cpp
  - libs/common/tests/CMakeLists.txt
  - libs/common/tests/images/image_test.cpp
  - libs/common/tests/logging/logger_test.cpp
  - libs/events/include/voyahslam/events/async_observer.h
  - libs/events/include/voyahslam/events/event_node.h
  - libs/events/include/voyahslam/events/event_types.h
  - libs/events/include/voyahslam/events/frame_tracked.h
  - libs/events/include/voyahslam/events/global_ba_finished.h
  - libs/events/include/voyahslam/events/inbox_core.h
  - libs/events/include/voyahslam/events/initialization_measurement.h
  - libs/events/include/voyahslam/events/listener.h
  - libs/events/include/voyahslam/events/loop_detected.h
  - libs/events/include/voyahslam/events/map_transaction.h
  - libs/events/include/voyahslam/events/observable.h
  - libs/events/include/voyahslam/events/subscription.h
  - libs/events/include/voyahslam/events/tracking_lost.h
  - libs/events/include/voyahslam/events/worker_thread.h
  - libs/events/src/events/worker_thread.cpp
  - libs/events/tests/CMakeLists.txt
  - libs/events/tests/events/event_lifetime_test.cpp
  - libs/events/tests/events/event_system_test.cpp
  - libs/events/tests/events/lifecycle_test.cpp
  - libs/events/tests/events/probe_event.h
  - libs/events/tests/events/subscription_test.cpp
  - libs/events/tests/events/wake_signal_test.cpp
  - libs/events/tests/events/worker_thread_test.cpp
  - libs/slam/CMakeLists.txt
  - libs/slam/include/voyahslam/global_ba/global_ba.h
  - libs/slam/include/voyahslam/global_ba/global_bundle_adjuster.h
  - libs/slam/include/voyahslam/loop_closing/common_region.h
  - libs/slam/include/voyahslam/loop_closing/loop_closer.h
  - libs/slam/include/voyahslam/mapping/map_follower.h
  - libs/slam/include/voyahslam/mapping/map_snapshot.h
  - libs/slam/include/voyahslam/mapping/mapper.h
  - libs/slam/include/voyahslam/slam/slam.h
  - libs/slam/include/voyahslam/tracking/tracker.h
  - libs/slam/src/global_ba/global_bundle_adjuster.cpp
  - libs/slam/src/loop_closing/loop_closer.cpp
  - libs/slam/src/mapping/map_follower.cpp
  - libs/slam/src/mapping/mapper.cpp
  - libs/slam/src/mapping/mapper_global_ba.cpp
  - libs/slam/src/mapping/mapper_loop.cpp
  - libs/slam/src/mapping/mapper_merge.cpp
  - libs/slam/src/mapping/mapper_quality.cpp
  - libs/slam/src/slam/slam.cpp
  - libs/slam/src/tracking/tracker.cpp
  - libs/slam/tests/CMakeLists.txt
  - libs/slam/tests/build_checks/check_consumer_instantiations.cmake
  - libs/slam/tests/build_checks/follower_consumer.cpp
  - libs/slam/tests/features/projection_matcher_test.cpp
  - libs/slam/tests/global_ba/global_ba_test.cpp
  - libs/slam/tests/global_ba/global_bundle_adjuster_test.cpp
  - libs/slam/tests/integration/slam_test.cpp
  - libs/slam/tests/loop_closing/loop_closer_test.cpp
  - libs/slam/tests/mapping/map_follower_test.cpp
  - libs/slam/tests/mapping/mapper_test.cpp
  - libs/slam/tests/test_access.h
  - libs/slam/tests/thread_affinity_test.cpp
  - libs/slam/tests/tracking/atlas_replica_test.cpp
  - libs/slam/tests/tracking/local_map_test.cpp
  - libs/slam/tests/tracking/map_replica_test.cpp
  - libs/slam/tests/tracking/relocalization_test.cpp
  - libs/slam/tests/tracking/tracker_test.cpp
  - libs/slam/tests/tracking/tracker_test_fixture.h
  - vizualization/include/voyahviz/map_viewer.h
  - vizualization/src/map_viewer.cpp
  - vizualization/src/pangolin_renderer.cpp
  - vizualization/tests/map_viewer_test.cpp
  - vizualization/tests/scene_state_test.cpp
findings:
  critical: 2
  warning: 6
  info: 7
  total: 15
status: issues_found
---

# Phase 1: Code Review Report

**Reviewed:** 2026-10-15
**Depth:** standard
**Files Reviewed:** 87

## Summary

The production files were read in full for the lock-free and lifecycle core (`inbox_core.h`, `subscription.h`, `observable.h`, `event_node.h`, `worker_thread.{h,cpp}`, `slam.{h,cpp}`, `tracker.{h,cpp}`, `map_follower.{h,cpp}`, `global_bundle_adjuster.cpp`, `loop_closer.cpp`, `logger.{h,cpp}`, `image.h`, `image_pyramid.{h,cpp}`, ORB workspace change, `map_viewer.{h,cpp}`, the examples). The large test files (`mapper_test.cpp`, `loop_closer_test.cpp`, `tracker_test.cpp`, etc.) were sampled, not read line by line, for flaky timing and vacuous asserts; the lifecycle, subscription, worker, wake-signal, follower, affinity, image, logger and ORB-workspace tests were read in full. `slam_test.cpp` was read around its lifecycle and observer tests.

What holds up: I traced the `Subscription`/`Wiring`/`InboxCore` ownership graph (each holder keeps the core alive; `Observable`'s destructor stores `Stopped` so late `Subscription`s edit only the shared block), the `Starting`/`Stopping` CAS state machine in `EventNode`, the `WakeSignal` snapshot-then-check protocol in `WorkerThread`, and the `Slam` member destruction order. I found no ordering or memory-order error in the happy-path lock-free code.

The defects are at the edges: two memory-safety or crash defects reachable by a JNI caller that passes plain sizes (`Image` views, ORB extractor on small images), one provable deadlock that contradicts a documented contract (`EventNode::StopWith`), and several places where the stated "thrown in every build, never only asserted" policy of `errors.h` is not followed.

Verified = traced to a concrete code path in the source. Suspicion = needs a specific interleaving or failure I could not run.

## Critical Issues

### CR-01: ORB extractor divides by zero (SIGFPE) on images narrower or shorter than about 233 px

**File:** `libs/algs/src/features/orb_feature_extractor.cpp:562-569` (reached through `DetectAndExtract`, `:296-307`)
**Status:** Verified. The code predates this phase, but it is in a reviewed file and the new `Workspace` overloads are the public entry point.
**Issue:** `ComputePointsPerLevel` computes

```cpp
const int n_cols = (image_cols - PATCH_SIZE) / W;   // W = 35, PATCH_SIZE = 31
assert(n_cols > 0);
const int cell_width = std::ceil((image_cols - PATCH_SIZE) / n_cols);
```

For any pyramid level with `image_cols <= 65` (or rows), `n_cols` is 0. The only guard is `assert`, so a release build executes an integer division by zero and the process dies with SIGFPE. With the default 8 levels at scale 1.2 the last level is 1/3.58 of the input, so any input whose width or height is below about 233 px triggers it. A 224x224 image is a common size. The existing tests use 256 px and up and never hit it. `DetectAndExtract` is the call a JNI binding makes with caller-supplied images, and a crash here kills the JVM.
**Fix:** Validate before building the pyramid and throw instead of asserting:
```cpp
// in DetectAndExtract(image, n, workspace), before pyramid.Initialize/Build
const double min_scale = scale_factors_.empty() ? 1.0 : scale_factors_.back();
const double last_w = image.Width() / min_scale, last_h = image.Height() / min_scale;
if (last_w < 66 || last_h < 66)   // (PATCH_SIZE + W) so that n_cols, n_rows >= 1
    throw std::invalid_argument("OrbFeatureExtractor: image too small for the pyramid depth");
```
Also guard against `scale_factors_.empty()` (`levels == 0` makes `image_pyramid_[0]` in `ImagePyramid::Build` and `Initialize` undefined). Add a test with a small image.

### CR-02: `Image` views and `CopyTo` are unchecked in release builds (out-of-bounds read/write)

**File:** `libs/common/include/voyahslam/images/image.h:50-55` (view constructor), `:70-78` (`operator[]`), `:168-184` (`CopyTo`)
**Status:** Verified by reading. Exploitability depends on the caller passing unchecked numbers, which the class comment (lines 30-33) says the JNI binding does.
**Issue:**
- `Image(parent, start_x, start_y, width, height)` computes an offset and stores `width`/`height` without checking `start_x + width <= parent.Width()` or `start_y + height <= parent.Height()`. Any later `At`/`Set`/`Convert`/`operator*=`/`CopyTo` on that view reads or writes outside the parent's buffer. A view of a default (empty, null-buffer) image with non-zero size also dereferences null.
- `CopyTo` checks `other.Width() == Width()` and `other.Height() == Height()` only with `assert`. Under `NDEBUG` a smaller `other` receives a `memcpy` of this view's full size, a heap overflow. The fast path (`width_ == stride_ && other.stride_ == stride_`) also copies `width_*height_*C` elements assuming `other` is equally contiguous.
- `operator[]` has no bound on `i`.

`errors.h` states the library's policy: usage errors are "thrown in every build, never only asserted", precisely because the caller may be the JNI binding. The views and `CopyTo` are the only new public surface in this phase that breaks that policy while handling caller-controlled sizes.
**Fix:**
```cpp
Image(Image& other, unsigned sx, unsigned sy, unsigned w, unsigned h)
    : Image(other, /*offset*/..., w, h) {
  if (static_cast<size_t>(sx) + w > other.width_ || static_cast<size_t>(sy) + h > other.height_)
    throw std::out_of_range("Image view exceeds its parent");
}
void CopyTo(Image& other) const {
  if (other.width_ != width_ || other.height_ != height_)
    throw std::invalid_argument("Image::CopyTo: size mismatch");
  ...
}
```
Add tests for the out-of-range view and the size-mismatched `CopyTo` (the current `image_test.cpp` covers only in-range views).

## Warnings

### WR-01: `EventNode::StopWith` deadlocks when a component's own worker thread calls `Stop()` while another thread is already stopping it

**File:** `libs/events/include/voyahslam/events/event_node.h:102-105` (documented contract at `:89-94`, `tracker.h:182-183`, `slam.h:34-40`)
**Status:** Verified by reading; needs two threads (one external `Stop()`, one `Stop()` from a sync handler or callback on the worker).
**Issue:** The "refuse on own thread" check lives only inside `stop_worker` (`WorkerThread::Stop`), which runs only in the `Running` case. If thread B has moved the node `Running -> Stopping` and is inside `worker_.Stop()` joining the worker, and the worker thread A (inside a sync handler) calls `Stop()`, A reads `Stopping`, takes the `Starting/Stopping` branch and calls `WaitWhileState(Stopping)`. A waits for B to finish, B waits to join A, and neither returns. The documented behavior ("throws UsageError") only holds for `Slam::Stop()`, which checks `OnComponentThread()` before touching state (`slam.cpp:97`). `slam.tracker().Stop()` and the other component accessors are public.
**Fix:** Let the component tell `StopWith` how to detect its own thread, and check before waiting:
```cpp
template <typename TStopWorker, typename TOnWorker>
void StopWith(TStopWorker&& stop_worker, TOnWorker&& on_worker_thread) {
    if (on_worker_thread()) throw UsageError("Stop() from the component's own worker thread");
    ...
}
// Tracker::Stop(): StopWith([this]{ worker_.Stop(); }, [this]{ return worker_.OnWorkerThread(); });
```
Add a test: sync handler calls `Stop()` on the node while the main thread is also in `Stop()`.

### WR-02: An exception leaving `process_pending` or a sync handler terminates the process silently, and a throwing sync handler also skips async delivery

**File:** `libs/events/src/events/worker_thread.cpp:18-28`, `libs/events/include/voyahslam/events/observable.h:74-79`
**Status:** Verified by reading.
**Issue:** The worker thread body has no `try/catch`. Any exception from `ProcessPending()` (`std::bad_alloc` while building a transaction, an Eigen/std exception on a malformed measurement passed to `Track()`, a `UsageError`, a throwing sync handler such as one that calls into Java) unwinds out of the `std::thread` function and calls `std::terminate` with no log line. `Observable::Notify` also runs sync handlers before the async pushes, so a throwing sync handler aborts the loop and the async observers never get the event even if the process survived. For a library consumed from the JVM, a diagnosable failure (log critical, mark the node stopped) is much better than an anonymous abort.
**Fix:** Wrap the loop body:
```cpp
try { ... } catch (const std::exception& e) {
    Logger::CreateLogger("WorkerThread")->critical("worker died: {}", e.what());
    running_.store(false, std::memory_order_release);
    worker_id_.store(std::thread::id{}, std::memory_order_release);
    return;   // or std::terminate() after logging, if fail-fast is wanted
}
```
and in `Notify` deliver the async pushes first or guard each sync handler with `try/catch`. Document the chosen policy in DESIGN section 7 (exception policy).

### WR-03: `MapFollower` thread ownership is `assert`-only, and the owner id is a plain non-atomic

**File:** `libs/slam/src/mapping/map_follower.cpp:79-90`, `libs/slam/include/voyahslam/mapping/map_follower.h:26-28, 80`
**Status:** Verified by reading.
**Issue:** The class says `Update()`, `atlas()` and `Snapshot()` belong to one thread and that this is "checked by assert in debug builds of the library". In a release build nothing is checked, and `Snapshot()` from a second thread (for example a JVM UI thread, since JNI calls arrive on arbitrary Java threads) iterates the `AtlasReplica` while `Update()` mutates it, a data race on non-atomic containers. `owner_` itself is a plain `std::thread::id` written by `ClaimThread()`, so two first calls from different threads race even in debug. This contradicts `errors.h` ("thrown in every build, never only asserted") for exactly the caller that the follower exists to serve.
**Fix:** Make `owner_` a `std::atomic<std::thread::id>`, claim it with `compare_exchange_strong` from the default id, and throw `UsageError` on mismatch in every build (the cost is one atomic load per call).

### WR-04: Connect/disconnect versus `Start()` has a check-then-act race on the wiring vectors

**File:** `libs/events/include/voyahslam/events/observable.h:37-58, 106-110`, `libs/events/include/voyahslam/events/subscription.h:90-115`
**Status:** Suspicion from reading; needs `Start()` on one thread concurrent with `AddObserver`/`Subscription` destruction on another.
**Issue:** `ConnectAsync` does `ThrowIfRunning()` (an atomic load) and then mutates `wiring_->async`/`next_id`; `Subscription::~Subscription`/`reset()` do the same around `Remove`. `StartWith` can win the `Created -> Starting` CAS between the check and the mutation, so the worker may begin reading `async`/`sync` while another thread is in `push_back`/`erase_if`. The header comments promise `Start()`/`Stop()` are "safe from any thread", but the wiring edit is only safe when the caller externally orders it before `Start()`. That is a usage rule, not something the type enforces, and the JNI binding is the kind of caller that gets it wrong.
**Fix:** Either state explicitly in `event_node.h` and `slam.h` that connections must be ordered before `Start()` by the caller, or add a `Wiring` "editing" claim (CAS `Created/Stopped -> Editing`, Start fails with `UsageError` while editing) so the two cannot overlap.

### WR-05: `Slam::Start` rollback and `Slam::Stop` partial-failure paths can strand the state machine

**File:** `libs/slam/src/slam/slam.cpp:79-88, 123-130`
**Status:** Suspicion (needs a failing `std::thread::join` or a throw out of a component `Stop()`).
**Issue:** The `catch (...)` in `Start()` calls four `Stop()`s with no protection. If one throws (for example a `std::system_error` from `join`), the later `Stop()`s and the `state_.store(Stopped)` / `notify_all` are skipped, and `state_` stays `Starting` forever, so every later `Stop()` (including the destructor, which is `noexcept`, so this becomes `std::terminate`) blocks on `state_.wait(Starting)`. In `Stop()`, if `loop_closer_.Stop()` throws after `tracker_.Stop()` succeeded, `state_` is restored to `Running` while the Tracker is already `Stopped`: `run_state()` reports `Running` and `Track()` throws, an inconsistent view.
**Fix:** In the rollback, stop each component in its own `try/catch`, always finish with `state_.store(Stopped); notify_all()`, and rethrow the original exception. In `Stop()`, if a component failed after others stopped, still end in `Stopped` (terminal) rather than reverting to `Running`.

### WR-06: Example apps keep a `MapFollower` inbox that is never drained during the run

**File:** `examples/full_slam/monocular_full_slam.cpp:66-67, 128`, `examples/full_slam/binocular_full_slam.cpp` (same lines)
**Status:** Verified by reading.
**Issue:** `subscriptions.push_back(slam.mapper().AddObserver(follower.inbox()))` subscribes the follower to every Mapper transaction, but `follower.Update()` is only called after `slam.Stop()`. The follower's own header says "A follower that is never updated lets its inbox grow". On a long dataset every `MapTransaction` (new points, descriptors, bags of words, culled frames' data) is retained until the end, and the log fills with the "consumer is behind" warning at 1024, 2048, 4096... queued events. Since pools were removed in this phase the inbox is no longer bounded. The examples are what users copy.
**Fix:** Call `follower.Update()` inside the frame loop (it is cheap when idle), for example every N frames next to `trajectory->Poll()`.

## Info

### IN-01: `InboxCore::Push` leaves `pending_priority_` inflated if `enqueue` throws

**File:** `libs/events/include/voyahslam/events/inbox_core.h:49-52`
**Issue:** The count is incremented before `queue_.enqueue(...)` (deliberately, so a pop never sees the count behind the queue). `enqueue` allocates and can throw `std::bad_alloc`; the increment is then never undone, `pending_priority()` stays above 0, and `mapper_quality.cpp:422` (`should_stop = pending_priority() > 0`) would cut every later local BA short.
**Fix:** `try { queue_.enqueue(std::move(event)); } catch (...) { if (priority) pending_priority_.fetch_sub(1); throw; }`.

### IN-02: Logger edge cases

**File:** `libs/common/src/logging/logger.cpp:36-60`
**Issue:** (a) `SetLevel` does not take `creation_mutex`, so a logger created between `CurrentLevel().load()` and `register_logger` can keep the old level. (b) `CreateLogger` returns whatever `spdlog::get(name)` finds, so if the host application registered its own spdlog logger named "Tracker" or "Mapper", the library logs through the app's sinks and not through `SharedSink()`. (c) `Initialize` accepts null `sink_ptr` entries, which crash on the next log call. (d) `CreateLogger` takes a global mutex and is called from `InboxCore::WarnIfBehind` and the `Subscription` terminate path, which is acceptable (rare) but contradicts the "never on the event path" comment.
**Fix:** Prefix library logger names (`"voyahslam.Tracker"`), reject null sinks, and take the mutex in `SetLevel`.

### IN-03: `MapViewer::Start()` strands its state if the thread cannot be created

**File:** `vizualization/src/map_viewer.cpp:42-44`
**Issue:** `TryEnterMode` and `loop_running_.store(true)` run before `thread_ = std::thread(...)`. If the constructor throws `std::system_error`, `state_` stays `kOwnThread` and `loop_running_` stays true, so later `Start()` calls are ignored ("already running") and `WaitForWindowClose()` would block.
**Fix:** Wrap the thread creation in `try/catch` that restores `state_` to `kIdle` and `loop_running_` to false before rethrowing.

### IN-04: Test hygiene: shared logger sinks not restored on assertion failure

**File:** `libs/events/tests/events/subscription_test.cpp:309-335, 337-355`
**Issue:** `logger->sinks().push_back(sink)` on the process-wide "AsyncObserver" logger is undone by `pop_back()` at the end of the test body. A failing `ASSERT_EQ`/`ASSERT_TRUE` returns early and leaves the sink attached for the rest of the binary. `ASSERT_EQ(ProbeEvent::alive.load(), 0)` at test start also couples tests through a global counter. `ConcurrentLogAndInitialize` (`logger_test.cpp:104-129`) asserts nothing about the logged data and is only meaningful under TSAN.
**Fix:** Use a small RAII guard that pops the sink in its destructor.

### IN-05: Process-terminating paths a JNI caller can reach

**File:** `libs/events/include/voyahslam/events/subscription.h:90-101`, `libs/events/src/events/worker_thread.cpp:11`, `libs/slam/src/slam/slam.cpp:56-61`
**Issue:** By design (D-07) destroying a `Subscription` while its producer runs, destroying a `Slam` on a component thread, or destroying a `WorkerThread` on its own thread (its destructor calls `Stop()`, which throws, and destructors are `noexcept`) all terminate the process. If the binding ties `Subscription` lifetime to Java GC or `close()`, a late or early finalizer aborts the JVM. This is documented, but nothing in the API lets the binding ask "is it safe to drop this now?" short of reading `RunState`.
**Fix:** Expose `Subscription::producer_running()` (or a `try_reset()` returning bool) so the binding can defer instead of aborting.

### IN-06: Small correctness and robustness items in `image.h` / ORB code

**File:** `libs/common/include/voyahslam/images/image.h:140-143, 164, 221-229`, `libs/algs/src/features/orb_feature_extractor.cpp:572-573`, `vizualization/src/opencv_wrapping.h:15`
**Issue:** `operator=(float)` and `operator T()` dereference `data_[offset_]` on an empty image (null). `operator<<` prints `uint8_t` pixels as characters. `std::ceil((image_cols - PATCH_SIZE) / n_cols)` applies `ceil` to an integer division (a no-op), so the FAST grid truncates and leaves up to `n_cols-1` right-hand columns uncovered (ORB-SLAM3 uses floating point). `WrapAsCvMat` takes `&image.At(0,0,0)` on a possibly empty image.
**Fix:** Guard empty images in the accessors, cast to `int` when streaming, and use `std::ceil(static_cast<double>(...) / n_cols)` if full coverage is intended.

### IN-07: `Tracker::ProcessPending()` / `Mapper::ProcessPending()` are public and unsynchronized with the worker

**File:** `libs/slam/include/voyahslam/tracking/tracker.h:197-201`, `libs/slam/include/voyahslam/mapping/mapper.h:195-197`
**Issue:** They are public "so tests can drive it synchronously (never concurrently with a running worker)". A binding or app calling them on a running component races with the worker on all thread-affine state (the replica, `last_frame_`, `batch_`). The phase made the read accessors private for exactly this reason; these are the remaining public entry points into the same state. Not enforced in any build.
**Fix:** Make them private behind the existing `*TestAccess` friend structs, or throw `UsageError` when `run_state() == Running`.

---

_Reviewed: 2026-10-15_
_Reviewer: Claude (gsd-code-reviewer)_
_Depth: standard_
