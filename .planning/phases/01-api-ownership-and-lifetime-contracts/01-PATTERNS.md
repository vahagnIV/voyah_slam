# Phase 1: API Ownership and Lifetime Contracts - Pattern Map

**Mapped:** 2026-10-13
**Files analyzed:** ~40 (grouped)
**Analogs found:** strong analogs for most; no analog for death tests, exception types, Subscription, MapFollower, Workspace.

All analog paths are git-tracked under `libs/`, `examples/`, `vizualization/`. Layout is `libs/<lib>/{include/voyahslam/...,src,tests}` (dirs: `common`, `events`, `algs`, `slam`, `test_support`), not `libs/voyah_*`.

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match |
|---|---|---|---|---|
| `libs/events/include/voyahslam/events/event_types.h` (drop `recycle_`/`set_ref_count_`, protected ctor takes only type) | model | event-driven | itself (L11-26 of Event struct) | modify |
| `events/{map_transaction,frame_tracked,loop_detected,global_ba_finished,tracking_lost,initialization_measurement}.h` (drop `Recyclable`, `Reset()`, lambda ctor args) | model | event-driven | `global_ba_finished.h` / `loop_detected.h` | modify |
| `events/observable.h` (`Notify(shared_ptr<const Event>)`, wiring block) | service | pub-sub | itself | modify |
| `events/async_observer.h` + new inbox core (queue, `WakeSignal`, priority counter) | service | pub-sub queue | itself (moodycamel queue, `pending_priority_`) | modify |
| `events/listener.h` (holds `shared_ptr<InboxCore>`, `current_` is `shared_ptr<const Event>`) | utility | streaming | itself | modify |
| new `events/subscription.h` (move-only RAII, `[[nodiscard]]`) | utility | event-driven | `WorkerThread` (move-deleted RAII, `worker_thread.h`) and `Listener` move ops | partial |
| `events/event_node.h` (`AddObserver` returns `Subscription`, remove `SetWakeSignal`) | middleware | pub-sub | itself | modify |
| `events/worker_thread.h/.cpp` (unchanged API; state CAS lives in components) | utility | event-driven | itself | none |
| new `UsageError` (likely `libs/common/include/voyahslam/error/usage_error.h`) | utility | n/a | `libs/common/src/io/read_image.cpp` (only throw site) | no analog |
| `libs/slam/include/voyahslam/{tracking/tracker,mapping/mapper,loop_closing/loop_closer,global_ba/global_bundle_adjuster}.h`, `src/loop_closing/loop_closer.cpp`, `src/global_ba/global_bundle_adjuster.cpp`, `src/slam/slam.cpp`, `slam/slam.h` | service | event-driven | `mapper.h` (`MapperTestAccess` friend, pools to remove) | modify |
| `libs/common/include/voyahslam/memory/{object_pool,recyclable,sync_policy}.h`, `src/memory/object_pool.cpp`, `tests/memory/object_pool_test.cpp` | delete | n/a | n/a | delete |
| new `TrackerTestAccess`, `LoopCloserTestAccess`, `GlobalBATestAccess` | test helper | request-response | `MapperTestAccess` in `libs/slam/tests/mapping/mapper_test.cpp:39` | exact |
| new `MapFollower` + `MapSnapshot` (`libs/slam/include/voyahslam/tracking/map_follower.h`) | service | event-driven + CRUD | `atlas_replica.h` (wrapped), `ViewerSceneState` consumer in `vizualization/src/map_viewer.cpp:157-160` | role-match |
| `libs/common/include/voyahslam/features/orb_feature_extractor.h`, `libs/algs/src/features/orb_feature_extractor.cpp` (Workspace) | service | transform | itself (`mutable orb::ImagePyramid pyramid_` L47) | modify |
| `libs/common/include/voyahslam/logging/logger.h`, `src/logging/logger.cpp` | utility | n/a | itself | modify |
| `libs/common/include/voyahslam/images/image.h` + new `libs/common/tests/images/image_test.cpp` | model + test | transform | `libs/common/tests/map/frame_test.cpp` for test style | modify |
| `vizualization/include/voyahviz/map_viewer.h`, `src/map_viewer.cpp` | component | pub-sub consumer | itself | modify |
| `examples/full_slam/{full_slam_common.h,monocular_full_slam.cpp,binocular_full_slam.cpp}` | app | event-driven | `TrajectoryWriter` (full_slam_common.h:187-223) | modify |
| tests: `libs/events/tests/events/event_system_test.cpp`, `wake_signal_test.cpp`, slam tests (`tracker_test_fixture.h`, `tracker_test.cpp`, `mapper_test.cpp`, `loop_closer_test.cpp`, `global_ba_*_test.cpp`, `local_map_test.cpp`, `map_replica_test.cpp`, `atlas_replica_test.cpp`, `projection_matcher_test.cpp`, `integration/slam_test.cpp`), algs extractor tests, `vizualization/tests/{map_viewer_test,scene_state_test}.cpp` | test | n/a | same files | modify |

## Pattern Assignments

### Event types (`events/*.h`) - strip pool machinery
**Current pattern to remove** (`global_ba_finished.h:20-44`): struct derives `Event` and `memory::Recyclable<T, ThreadSafePolicy>`, has `Reset()` (clears vectors, `priority=false`), and a ctor passing two captureless lambdas (`recycle_`, `set_ref_count_`) to the protected `Event(type, r, s)` ctor. Target shape: `struct GlobalBAFinished : Event { fields...; GlobalBAFinished() : Event(EventType::GlobalBAFinished) {} };`. Drop `#include <voyahslam/memory/recyclable.h>` (L12; `loop_detected.h` L59). Also update the comments in `event_types.h:337` and `:339-340` ("pooled events are not reset"; priority "not reset on reuse"): after D-01 the producer sets `priority`/`map_id` before publishing as `shared_ptr<const T>`, and they are immutable afterwards. `EventType` enum and its "keep last" comment stay. Event destructors must not depend on the destroying thread (D-03). `Event` needs a virtual-free or protected non-virtual dtor: since `make_shared<T>` destroys with the exact type, a protected non-virtual dtor in `Event` is fine, but publishing as `shared_ptr<const Event>` also works because the control block stores the deleter.

### `observable.h` (pub-sub)
**Analog:** itself, L12-59. Replace `std::vector<AsyncObserver*> async_observers_` with entries that hold shared inbox cores (D-22), and `Notify(Event*)` with `Notify(std::shared_ptr<const Event>)`. Keep the sync-then-async ordering:
```cpp
void Notify(Event* event) {
    for (auto& handler : sync_handlers_) handler.fn(handler.ctx, event);   // sync first, inline
    if (async_observers_.empty()) {...return;}
    for (auto* observer : async_observers_) observer->Push(event);          // fan-out
}
```
After: `for (auto& o : async) o.core->Push(event)` copies the `shared_ptr` per observer (no `set_ref_count_`). `ConnectSync(void(*fn)(void*, Event*), void* ctx)` becomes `(void*, const Event&)` or shared_ptr; its handler "must not retain the event" comment (event_node.h:94-95) stays. `async_count()` stays (event_node.h `async_observer_count`).

### `async_observer.h` + inbox core + `listener.h` (queue holding events)
**Analog:** itself. Keep these pieces, moving them into a shared core (`struct InboxCore { moodycamel::ConcurrentQueue<std::shared_ptr<const Event>> queue; std::atomic<uint32_t> pending_priority{0}; WakeSignal wake_signal; }`):
- Counted-before-enqueue priority (`async_observer.h:49-59`):
```cpp
void Push(Event* event) {
    if (event->priority) pending_priority_.fetch_add(1, std::memory_order_acq_rel);
    queue_.enqueue(event);
    if (wake_signal_) wake_signal_->Notify();
}
```
- Pop-side decrement, duplicated in `AsyncObserver::TryPop` (L36-47) and `Listener::GetNextEvent` (listener.h:63-72): `if (queue_->try_dequeue(current_) && current_->priority && pending_priority_) pending_priority_->fetch_sub(1, acq_rel);`. Refactor into one core method so both paths share it.
- `Listener` move ctor/assignment (listener.h:43-60): the `other.current_ = nullptr` idiom; with `shared_ptr<const Event> current_` the defaults suffice and the explicit `recycle_` calls vanish. `Listener(queue&, atomic*)` ctor becomes `Listener(std::shared_ptr<InboxCore>)`. Existing tests construct `Listener listener(queue)` (event_system_test.cpp:41-ff) - these must be rewritten.
- Backlog warning (D-24): add a `size_hint`/counter in the core and log with `CLASS_LOGGER` (see Logger shared pattern); warn once each time queue size doubles above a floor, naming the observer.
- `AsyncObserver` destructor releases queued events (D-08): drain via `try_dequeue` loop into a discarded `shared_ptr`; no producer drain.
- Wake signal ownership moves into the core; `wake_signal()` accessor on the observer replaces `SetWakeSignal` (async_observer.h:23, event_node.h:88 `inbox_.SetWakeSignal(&wake_signal_)`). `EventNode` (event_node.h:22-48) protected `wake_signal()` accessor should return `inbox_.wake_signal()`; `WorkerThread(WakeSignal&)` (worker_thread.h:20) binds to it, so the core must outlive the `WorkerThread` (shared core held by the node satisfies that).

### New `Subscription` (`events/subscription.h`)
**No direct analog**; combine two local idioms.
- Move-only RAII with `other.x = nullptr` in move: `Listener` (listener.h:43-60).
- Joining destructor that stops first: `WorkerThread` (worker_thread.h:15-37, `~WorkerThread()` calls `Stop()` idempotently).
Signature direction: `[[nodiscard]] Subscription AddObserver(AsyncObserver&)` / `AddSyncObserver(...)`; holds `std::weak_ptr<WiringBlock>` + entry id and a `running` probe from the wiring block; `reset()` throws `UsageError` when the producer runs; destructor on a running producer logs and `std::terminate()` (D-07); after the producer is gone the weak_ptr is expired -> no-op. Connect only while stopped: check producer state (`WorkerThread::running()` exists at worker_thread.h:29; the new Created/Running/Stopped atomic state is the better source).

### Components (Tracker, Mapper, LoopCloser, GlobalBA): remove pools, add lifecycle
**Analog:** `mapper.h`, `tracker.h`.
- Remove: `#include <voyahslam/memory/object_pool.h>` (tracker.h:30, mapper.h:40); `TrackerOptions::event_pool_size` (tracker.h:93-96), `MapperOptions::transaction_pool_size` (mapper.h:86-89); `event_pool_overflow()` (tracker.h:198-203), `transaction_pool_overflow()` (mapper.h:224-227); pool members (tracker.h:342-348, mapper.h:774-775); `loop_detected_pool_.TryCreate()` (loop_closer.cpp:247) and the GlobalBA equivalents plus `max_events_in_flight`, `num_dropped_loops`, `num_results_dropped` (D-25). `Mapper::CreateTransaction()` (mapper.h:~420, "A pooled transaction") becomes `std::make_shared<events::MapTransaction<...>>()` returning a non-const `shared_ptr` the Mapper fills, then publishes as `shared_ptr<const T>`.
- Publish pattern replaced: `observable_.Notify(event)` (loop_closer.cpp:285; `MapperTestAccess::PublishLoopEdge` mapper_test.cpp:46-51 uses `CreateTransaction()` + `Notify`).
- Lifecycle: `Start()/Stop()` currently documented "must be called from one controlling thread" (tracker.h:180-182, mapper.h:193-195) - rewrite to an atomic state CAS (Created->Running->Stopped, D-10/D-11). Destructors call `Stop()` (comment "Stops the worker before any member it uses is destroyed", tracker.h:177 / mapper.h:190) - keep idempotent. `Track()` after Stop throws `UsageError`, before Start still queues (D-12). Add `slam.h` update: remove the `global_ba_` ordering invariant (slam.h L62-69 comment; `std::optional<TGlobalBundleAdjuster> global_ba_` L69).
- `Slam` wiring (`libs/slam/src/slam/slam.cpp:28-37`) calls `AddObserver` for six edges; these now return `[[nodiscard]]` `Subscription`s, so `Slam` must hold them as members (declare after the components so they are destroyed first, or before and rely on the no-op rule - pick per D-22/D-23) and `slam.h:19-21` comment ("Not movable: the components hold pointers to each other's inboxes") stays valid.

### `*TestAccess` structs (D-14, D-26)
**Analog (exact):** declaration `mapper.h:105-106` + `friend struct MapperTestAccess;` at mapper.h:292 (inside `private:`), definition in the test file `mapper_test.cpp:33-ff`, namespace `voyahslam::mapping`, template static functions taking `Mapper<TFeatureExtractor>&`:
```cpp
// Test access to the Mapper's private graph edits (defined by the tests).
struct MapperTestAccess;
...
private:
    friend struct MapperTestAccess;
...
struct MapperTestAccess {
    template <typename TFeatureExtractor>
    static void CorrectWindow(Mapper<TFeatureExtractor>& out_mapper, ...) { out_mapper.CorrectWindow(...); }
};
```
Apply to `TrackerTestAccess` (`atlas()`, `map_replica()`, `last_matches()`; tracker.h:217-223), `LoopCloserTestAccess`, `GlobalBATestAccess`, and extend `MapperTestAccess` with the moved accessors (`covisibility()`, `Reference()`, `NumVisible()`, `NumFound()`, `last_local_ba()`, `BowVectorOf()`, `Parent()`, `LoopEdges()`, `KeyFrame()` at mapper.h:~280, `map_ids()`, `active_map()`, `num_keyframes(MapId)` at mapper.h:~288-290). Existing call sites use `MapperTestAccess::NumVisible(mapper_, map::MapId{1})` (mapper_test.cpp:2582-2637); new accessors follow the `(mapper, args)` static form. Note `mapper_test.cpp:39` defines the struct in the anonymous/test namespace file; since tests in several files (tracker_test_fixture.h is a header shared by tests) need it, define each new Access struct once in a shared test header (e.g. `libs/slam/tests/tracking/tracker_test_fixture.h`) to avoid ODR duplicates - the struct must be in the same namespace as the class (`voyahslam::tracking`, `loop_closing`, `global_ba`). Plain `size_t` counters become `std::atomic<size_t>` (D-14); keep the any-thread `num_keyframes()` public.

### `MapFollower` / `MapSnapshot`
**Analogs:** `libs/slam/include/voyahslam/tracking/atlas_replica.h` and `map_replica.h` (wrapped; tests `libs/slam/tests/tracking/atlas_replica_test.cpp` for how a replica applies a `MapTransaction`); consumer-loop style from `vizualization/src/map_viewer.cpp:157-160`:
```cpp
voyahslam::events::Listener listener = inbox_.GetListener();
while (voyahslam::events::Event* event = listener.GetNextEvent()) { ... }
```
and the `TrajectoryWriter` pattern in `examples/full_slam/full_slam_common.h:187-223` (owns `AsyncObserver inbox_` plus `Listener listener_ = inbox_.GetListener();` with the comment "After its inbox: destroyed first"; `Poll()` drains; type-check `event->type != EventType::FrameTracked` then `static_cast<const events::FrameTracked<T>*>`). `MapFollower::Update()` has the same shape as `Poll()` but applies `MapTransaction`s to its `AtlasReplica`. Under D-01 the listener returns `shared_ptr<const Event>`, so use `std::static_pointer_cast`. `MapSnapshot` is immutable shared data: follow `shared_ptr<const T>` precedent (vocabulary, `TSharedMeasurement`, `TDistortionModelPtr`). Place in the `voyah_slam` lib (depends on replica types); add the header to the slam test list in `libs/slam/tests/CMakeLists.txt` (`tracking/map_follower_test.cpp`).

### Extractor `Workspace` (D-15..D-17)
**Analog:** `orb_feature_extractor.h:22,47` (`DetectAndExtract(...) const` plus `mutable orb::ImagePyramid pyramid_`) and `libs/algs/src/features/orb_feature_extractor.cpp`; include `voyahslam/features/orb/image_pyramid.h` (L5). Move `pyramid_` into a nested/struct `Workspace`, new signature `DetectAndExtract(const images::TWorkingImage&, size_t, Workspace& out_workspace) const` (`out_` prefix for non-const ref params per CLAUDE.md), keep the 2-arg overload building a temporary workspace. Callers to update (found by grep): `examples/full_slam/*`, `libs/algs/tests/features/{orb_extractor_golden_test,feature_extractor_levels_test}.cpp`, `libs/slam/tests/tracking/tracker_test_fixture.h`, `libs/slam/tests/features/projection_matcher_test.cpp`, `libs/slam/tests/integration/slam_test.cpp`, benchmarks (grep `benchmarks/` - not scanned, verify), `libs/common/tests/memory/...` not relevant.

### Logger (D-18)
**Analog:** itself. `logger.cpp:6-39`: static `sinks_` vector and `level_`; `CreateLogger` copies `sinks_` at creation (the ordering bug) and `SetLevel` only calls `spdlog::set_level`. Target: a single `std::shared_ptr<spdlog::sinks::dist_sink_mt>` (include `spdlog/sinks/dist_sink.h`) created lazily (function-local static to avoid static-init order), passed to every `spdlog::logger`; `Initialize` calls `dist->set_sinks(sinks)` (default stdout color sink when empty, as at L10-12); `SetLevel` loops `spdlog::apply_all(...)` as well as setting `level_`. Keep `CLASS_LOGGER(className)` macro (logger.h L36-37) and the `spdlog::get(name)` reuse (logger.cpp L22-26). No Logger tests exist today; add `libs/common/tests/logging/logger_test.cpp` and register it.

### `images::Image` (D-20)
**Analog:** itself, `image.h:14-70`: defaults `Image() = default` leaves members uninitialized (add `= 0` / `{}` initializers on `offset_, width_, height_, stride_`); `Set(...) const` (L47-50) -> non-const; `Convert()` (L52-60) and `operator*=` (L62-67) iterate flat `width_*height_(*C)` from `GetData()`/`data_[i]` ignoring `offset_`/`stride_` - rewrite row-wise using `At`-style offset `offset_ + y*stride_*C`. Signed/unsigned loop `int i < unsigned` also to fix. Tests: new `libs/common/tests/images/image_test.cpp` (no image tests exist; `libs/common/tests/` subdirs: algebra, features, geometry, map, memory, support).

### `MapViewer` and apps
**Analog:** `vizualization/include/voyahviz/map_viewer.h:46,91,120` (`inbox()` accessor, `AsyncObserver inbox_` member) and `map_viewer.cpp:157-160` drain loop; `examples/full_slam/{monocular,binocular}_full_slam.cpp:60-80`. Replace the `StopAll` guard (struct at monocular L67-76 / binocular L71-80) with `std::vector<events::Subscription> subscriptions;` declared before `slam` (D-23) and `subscriptions.push_back(slam.tracker().AddObserver(viewer.inbox()));` for the three wiring lines (monocular L78-80). Replace `full_slam::PrintAtlas(slam.mapper(), ...)` (monocular L138, binocular L146; definition `full_slam_common.h:174`) with a `MapFollower` subscribed before `Start()` and `Update()`d after `Stop()`. `TrajectoryWriter` keeps its inbox + listener members; fix the comment "releasing the event it holds" and adopt shared_ptr.

## Shared Patterns

### Test registration
`voyah_add_test_binary(<name> LABEL <unit|integration|vocabulary> SOURCES ... LIBS ...)`, defined in `cmake/VoyahTests.cmake:26`. Per-lib list in `libs/<lib>/tests/CMakeLists.txt`; e.g. `libs/events/tests/CMakeLists.txt`:
```cmake
voyah_add_test_binary(voyah_events_tests
    LABEL unit
    SOURCES
        events/event_system_test.cpp
        events/wake_signal_test.cpp
        events/worker_thread_test.cpp
    LIBS
        voyah_events
)
```
Add new test `.cpp` files to `SOURCES` (alphabetical-ish, paths relative to the tests dir). `libs/slam/tests/CMakeLists.txt` also adds `target_include_directories(voyah_slam_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} PRIVATE .../../src)` so test headers like `tracking/tracker_test_fixture.h` resolve. Test names: `TEST(EventSystemTest, ListenerAutoRecycle)` style in namespace `voyahslam::events { namespace { ... } }`; gtest only (`<gtest/gtest.h>`). `libs/slam/tests` `voyah_slam_tests` links `voyah_slam voyah_test_support`. Delete `libs/common/tests/memory/object_pool_test.cpp` and its entry in `libs/common/tests/CMakeLists.txt`.

### Death-test convention
**No existing death tests** (grep for `DEATH` in `libs/**/*.cpp` finds none). New convention to establish for D-07 (terminate on `Subscription` destroyed while running): use gtest `EXPECT_DEATH`/`ASSERT_DEATH` with suite name suffix `DeathTest` (e.g. `TEST(SubscriptionDeathTest, ...)`, so gtest runs it first) and `GTEST_FLAG_SET(death_test_style, "threadsafe")` because the tests start threads; match the log message regex. Under TSan, death tests with threads need "threadsafe". Put these in the `unit` label binary. Document the convention in DESIGN.

### Exceptions (D-09)
Only throw site today is `libs/common/src/io/read_image.cpp` (grep `throw` to confirm) and `examples/full_slam/full_slam_common.h:192` (`throw std::runtime_error("cannot write " + path.string())`). New `UsageError : std::logic_error` in `voyah_common` (lowest layer needed: `voyah_events` and `voyah_slam` both depend on common). Tests: `EXPECT_THROW(x, UsageError)`. Record the policy in `docs/DESIGN.md` decisions log and conventions.

### Logging
`CLASS_LOGGER(ClassName);` member macro (`logger.h:36`) gives `log_`; use for the backlog warning (D-24) and the pre-terminate message (D-07).

### Naming / style
Google style per CLAUDE.md: `PascalCase` types/functions, `snake_case` vars, `snake_case_` members, `out_` prefix on non-const ref params, `[[nodiscard]]` on accessors, include guards `VOYAHSLAM_<LIB>_<FILE>_H`, headers include as `<voyahslam/<area>/<file>.h>`. Lower layers cannot include upper ones (events cannot include slam).

## No Analog Found

| File | Role | Data Flow | Reason |
|---|---|---|---|
| `Subscription` | utility | event-driven | no RAII connection handle exists (compose from `Listener`/`WorkerThread` idioms above) |
| `UsageError` | utility | n/a | no exception type in the library |
| Death tests | test | n/a | none exist; convention above |
| `MapSnapshot` | model | transform | no snapshot type; follow `shared_ptr<const T>` precedent |
| Inbox core / wiring block (shared_ptr control blocks across threads) | utility | pub-sub | nothing like it; derive from `AsyncObserver` internals |
| Image / Logger tests | test | n/a | none exist for these classes |

## Metadata

**Analog search scope:** `libs/{events,common,slam,algs}`, `examples/full_slam`, `vizualization`, `cmake/`
**Files scanned:** ~45 (heads/greps; large `mapper.h`/`mapper_test.cpp` sampled only)
**Not scanned:** `benchmarks/`, `tools/`, `test_package/` (grep these for `DetectAndExtract`, `AddObserver`, `ObjectPool` before planning the sweep)
**Pattern extraction date:** 2026-10-13
