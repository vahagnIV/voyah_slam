# Phase 1: API Ownership and Lifetime Contracts - Context

**Gathered:** 2026-10-12
**Status:** Ready for planning (revised 2026-10-13: event ownership, then the planning answers D-22..D-26)

<domain>
## Phase Boundary

Finish the voyah_jni ownership audit (`../voyah_jni/docs/voyah_slam_ownership_audit.md`) so that
no library consumer (the Java/JNI binding, whose objects die in any order, or a C++ app) can
crash the library through declaration order, destruction order or calls from the wrong
thread. Target principle: a type owns, or shares ownership of, everything it points to after a
call returns; every required ordering is removed or enforced (an error), not just documented.

In scope: audit items #3 (pool lifetime), #2 (observer subscriptions), #5 (thread-affinity
accessors and a supported map read path), #6 (lifecycle state machine), #7 (extractor thread
safety), #8 (logger initialization order), #9 (`images::Image` correctness). Requirements
OWN-01..07. Items #1 and #4 are already merged (0.2.0, 0.3.0).

Not in scope: lossy observer delivery (EVT-01, Phase 5), the Android logcat sink (PLAT-01,
Phase 5), a `voyahslam::Session` facade, moving voyahviz onto the new map follower.

</domain>

<decisions>
## Implementation Decisions

### Event ownership (#3, OWN-01) — revised 2026-10-13
The original D-01..D-04 (a shared, intrusively counted pool core) were **superseded on
2026-10-13** by the user: events are now coarse (one `MapTransaction` per Mapper action, one
`FrameTracked` per frame; well under 100 events/s in total), and the large payload (the
measurement) is already a per-frame `shared_ptr`, so pooling saves only the event shell and some
vector capacity — microseconds against a 10–30 ms frame. The pools are what made events able to
outlive their memory in the first place.
- **D-01:** Events are **heap-allocated and shared**: a producer creates each event with
  `std::make_shared` and publishes it as **`std::shared_ptr<const T>`** (one allocation per
  event; fan-out copies the pointer; a consumer acknowledges by dropping its copy). An event owns
  its memory, so it is valid however long a consumer holds it and whichever object dies first.
  `const` makes events immutable by type; if a consumer turns out to write to a received event,
  research/planning resolves it (copy, or a documented non-const exception). — **Reversibility:**
  one-way — every event type, `Observable`/`Listener`/`AsyncObserver` and the binding change.
- **D-02:** **`ObjectPool` and `Recyclable` are deleted** from `voyah_common` (events are their
  only users), with their tests, the pool-size options (`TrackerOptions::event_pool_size`,
  `MapperOptions::transaction_pool_size`), the overflow counters/accessors and every event's
  `Reset()`. The audit's "`~ObjectPool` asserts every slot came back" becomes moot. The
  drop-limits of `LoopDetected` (16) and `GlobalBAFinished` (`max_events_in_flight`, 2) are
  **dropped** too (D-25).
- **D-03:** The general-purpose allocator is acceptable on the event path: it is not a lock on
  map data (DESIGN §2), and per-frame allocations already happen (measurement, replicas). An
  event may be freed on any thread (viewer, JNI or GC thread): event destructors must not depend
  on the destroying thread (DESIGN rule).
- **D-04:** Afterwards remove the `global_ba_` ordering invariant (`slam.h`, DESIGN) and the
  apps' `StopAll` teardown guard (`examples/full_slam/*_full_slam.cpp`) **entirely** (D-23). This closes OQ 25 and makes OQ 14 (pool sizing) obsolete.

### Subscription API (#2, OWN-02)
- **D-05:** Connecting and disconnecting are allowed **only while the producer is stopped**
  (Created or Stopped). Disconnecting while it runs, without locks, was offered and declined.
- **D-06:** `AddObserver` and `AddSyncObserver` return a `[[nodiscard]]`, move-only RAII
  **`Subscription`**; its destructor or `reset()` disconnects. This covers `ConnectAsync`, the
  `ConnectSync` `void* ctx`, and the raw `WakeSignal*` of `SetWakeSignal` where it crosses
  object boundaries. Observers held by `shared_ptr` were declined. — **Reversibility:** one-way
  — the public wiring API of every component and of the voyah_jni binding changes.
- **D-07:** Misuse while running: an explicit `reset()`/disconnect, or an `AddObserver`, while
  the producer runs **throws** (the usage error of D-09). A `Subscription` **destroyed** while
  its producer runs **logs and calls `std::terminate`** in every build (destructors cannot
  throw; this gives a defined, loud failure instead of a later use-after-free).
- **D-08 (amended 2026-10-13):** When an `AsyncObserver` is destroyed, the events still queued
  are **released** (this closes TODO "AsyncObserver destruction doesn't acknowledge events");
  safe even after the producer is gone, because events own their memory (D-01). **Nothing is
  drained on disconnect**: an inbox can be fed by several producers (the viewer gets Tracker and
  Mapper events) and queued events carry no producer tag; events left queued stay valid.

### Misuse errors and lifecycle (#6, OWN-04)
- **D-09:** Invalid lifecycle calls **throw a `std::logic_error` subclass** (e.g.
  `voyahslam::UsageError`) in every build; the binding maps it to `IllegalStateException`.
  This is the library's first exception policy, so record it in DESIGN (decisions log and
  conventions). It is introduced with the #3+#2 release, because D-07 needs it.
- **D-10:** The state machine is Created → Running → Stopped, and **Stopped is terminal**:
  `Start` after `Stop` throws, `Start` twice throws, and `Stop` is idempotent (destructors call it).
  A new session means a new `Slam`. Applies to `Slam` and its components (Tracker, Mapper,
  LoopCloser, GlobalBA, `WorkerThread` users). Not `WorkerThread` itself, and not `MapViewer`
  (D-26).
- **D-11:** `Start`/`Stop` are safe **from any thread**, serialized by an atomic state CAS (a
  losing `Start` throws, `Stop` is idempotent). This replaces "must be called from one
  controlling thread", because a Java `close()` or GC cleaner may stop or destroy from another thread.
- **D-12:** `Track()` **before `Start()` keeps queuing** (today's behaviour); `Track()` **after
  `Stop()` throws**.

### Map read path and thread-affinity accessors (#5, OWN-03)
- **D-13:** A library **`MapFollower`** owns an inbox and an `AtlasReplica`. The app subscribes
  it to the Mapper's `MapTransaction`s, calls `Update()` on its own thread (which applies pending
  transactions) and reads the replica on that thread. It can also produce an immutable
  **`std::shared_ptr<const MapSnapshot>`** (points, keyframe poses, map ids; exact contents set
  at planning) for other threads or for Java. Snapshot cost is O(map) per call, paid only
  on request.
- **D-14:** The "Mapper thread only" accessors (`Mapper::covisibility()`, `Reference()`,
  `NumVisible()`, `NumFound()`, `last_local_ba()`, `BowVectorOf()`, `Parent()`, `LoopEdges()`,
  `KeyFrame()`, `map_ids()`, `active_map()`, `num_keyframes()`) and the "never while running"
  ones (`Tracker::atlas()`, `map_replica()`, `last_matches()`) become **private behind friend
  `*TestAccess` structs** (the `MapperTestAccess` pattern), so misuse fails at compile time.
  Tests and tools move to the access structs. **Extended 2026-10-13 (D-26):** the same applies
  to `LoopCloser::atlas()`/`map_replica()` and `GlobalBundleAdjuster::atlas()`/`map_replica()`;
  their plain `size_t` counters become `std::atomic`; only the `num_keyframes(MapId)` overload
  is restricted, and the no-argument `num_keyframes()` (an any-thread atomic) stays public.

### Extractor thread safety (#7, OWN-05)
- **D-15:** The **caller provides the workspace**:
  `DetectAndExtract(image, n, Workspace&) const`. The extractor becomes immutable and
  shareable, and the pyramid moves into the workspace (no per-frame allocation when the
  workspace is reused). Chosen over making the call non-const or using thread-local storage.
  — **Reversibility:** costly — every caller (apps, tests, benchmarks, voyah_jni) changes.
- **D-16:** A convenience **allocating overload `DetectAndExtract(image, n) const` stays**. It
  builds a temporary workspace per call, which is fine for tests and one-offs; document that a
  loop should reuse a workspace.
- **D-17:** The caller owns the workspaces, **one per extraction thread** (stereo: two if left
  and right are extracted in parallel). The workspace type is documented as not thread-safe.

### Logger (#8, OWN-06)
- **D-18:** Every logger gets **one shared spdlog `dist_sink_mt`**. `Logger::Initialize()` swaps
  its child sinks, so creation order stops mattering by construction, and `Initialize` may be
  called again at any time. `SetLevel` applies to every registered logger, not only to later
  ones. The cost is one extra indirection per log call.
- **D-19:** The Android logcat default sink stays in Phase 5 (PLAT-01). The binding can already
  pass spdlog's `android_sink` to `Initialize()`.

### Image (#9, OWN-07)
- **D-20:** **Fix in place**, with no separate `ImageView` type:
  - default-initialize every member (an empty image);
  - make `Convert()`, `operator*=` and `operator*` row-wise, honouring `offset_` / `stride_`;
  - make `Set()` non-const;
  - document that views alias the parent's buffer.

### Planning answers (2026-10-13, user: "all recommended")
- **D-22 (shared inbox core, OWN-02):** an `AsyncObserver`'s queue, its `WakeSignal` and its
  priority counter live in a shared **inbox core** held by the observer, its `Listener`s and the
  producer's connection entry; a component's connection list lives in a shared **wiring block**
  held by the component and its `Subscription`s. So every destruction order of {producer,
  subscription, observer, listener} is safe by construction: an observer destroyed first just
  stops being read (later events are released when the core goes), a `Subscription` destroyed
  after its producer is a no-op. `SetWakeSignal(WakeSignal*)` is removed (the core owns the
  signal; consumers wait on `observer.wake_signal()`). The earlier "count subscriptions per
  observer and terminate" idea is not adopted. D-05 and D-07 are unchanged (connect/disconnect
  only while stopped). — **Reversibility:** one-way (public wiring API, with D-06).
- **D-23 (no `StopAll`):** apps and the binding hold their subscriptions in a
  `std::vector<events::Subscription>` (or members) **declared before `slam`**, so `~Slam` (which
  stops) runs first and the subscriptions are destroyed after their producers (a no-op, D-22);
  an exception unwinding past them never hits D-07's terminate. The examples carry no guard.
- **D-24 (backlog warning):** without pools a stalled observer grows memory silently, so an
  `AsyncObserver` logs a "consumer is behind" warning once each time its queue size doubles
  (above a small floor), naming the observer.
- **D-25 (no drop-limits):** `LoopDetected` and `GlobalBAFinished` are published unbounded
  (`make_shared`), like every other event; `max_events_in_flight`, `num_dropped_loops` and
  `num_results_dropped` are removed. Both producers are paced by the Mapper's transactions and
  the LoopCloser keeps `max_query_backlog`. This fixes TODO "A dropped result is not retried"
  and makes the GBA-01 / Phase 3 "a dropped `GlobalBAFinished` is retried" criterion moot.
- **D-26 (scope answers):** D-14 extended to LoopCloser/GlobalBA (see D-14); `PrintAtlas` in the
  examples is replaced by a `MapFollower` subscribed before `Start()` and `Update()`d after
  `Stop()`; `WorkerThread` stays a restartable low-level primitive (`RestartAfterStop` stays) and
  the terminal Stopped rule lives in the components' lifecycle; `MapViewer` keeps "restart after
  `Stop()` allowed" (D-10 does not apply to it), only its wiring changes (D-06, D-22).

### Releases
- **D-21:** **#3 and #2 ship together** as one branch and one minor version bump (they are designed
  together and both reshape the event API). Then #5, #6, #7, #8 and #9 each get **their own
  branch, minor bump and release**, as #1 and #4 did. Each release:
  - a review, then a merge only on the user's explicit "merge and push";
  - a DESIGN decisions-log entry and `check_all`;
  - ThreadSanitizer on the event and Slam tests (#3+#2 and #6 at least);
  - `conan create` for linux-gcc13 (static and shared) and android-arm64;
  - the voyah_jni session is told the new version (every item is an API break for it).

### Claude's Discretion
- How the event base / `Listener` / `AsyncObserver` carry `shared_ptr<const T>` (type erasure,
  queue element type), and how the priority flag and counter survive, as long as D-01..D-03 hold.
- The exact `Subscription` type (one type for sync and async, or two) and how components track
  the "stopped" state used by D-05 (`WorkerThread::running()` exists).
- The `UsageError` name and header location (lowest layer that needs it, likely `voyah_common`).
- `MapSnapshot` contents and the `MapFollower` API details beyond D-13.
- The name and contents of the extractor `Workspace`.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Audit (source of this phase)
- `../voyah_jni/docs/voyah_slam_ownership_audit.md` (absolute path `/home/vahagn/git/voyah_jni/docs/voyah_slam_ownership_audit.md`) — the item list, failure cases, proposed fixes and coordination with voyah_jni; the decisions above override its proposals where they differ (e.g. D-01/D-02 delete the pools instead of adding the `~ObjectPool` assert).
- `../voyah_jni/native/src/slam_session.h` / `slam_session.cpp` — the binding's current workarounds (declaration order, stop before drain), which this phase should make unnecessary.

### Project rules and design
- `CLAUDE.md` — build, test targets, naming conventions, the "record decisions in DESIGN" rule.
- `docs/DESIGN.md` §2 "Core Principles" (no locks on map data), §6 "Events" and §6.1 "Event Lifecycle: Pooled, Reference-Counted, Acknowledged" (pointer rule, `Notify()` ownership), §7 "Coding Conventions", §8 "Decisions Log" (2026-10-12 entries for #1 and #4), §9 OQ 25 "Event pool lifetime".
- `docs/TODO.md` "Ownership audit (voyah_jni, 2026-10-12)", "Event system" (subscriber lifetime, pool lifetime, AsyncObserver drain) and "Map & events" (lossy delivery, deferred).
- `docs/ROADMAP.md` "API hardening for library consumers".
- `.planning/REQUIREMENTS.md` OWN-01..07; `.planning/ROADMAP.md` Phase 1 success criteria.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `libs/common/include/voyahslam/memory/object_pool.h`, `recyclable.h`: the event pool and intrusive refcount, deleted by D-02.
- `libs/events/include/voyahslam/events/worker_thread.h`: `running()`, usable for the stopped check (D-05, D-10).
- `libs/slam/include/voyahslam/tracking/atlas_replica.h`, `map_replica.h`: the replica that `MapFollower` wraps (D-13).
- `MapperTestAccess` (`libs/slam/include/voyahslam/mapping/mapper.h`): the pattern for D-14.

### Established Patterns
- Events are pooled and intrusively refcounted today, and `Notify()` always takes ownership (DESIGN §6.1); D-01 replaces this with `shared_ptr<const T>`.
- Components own their pools as members: Tracker (3), Mapper (`MapTransaction`), LoopCloser and GlobalBA use `TryCreate` (they drop on purpose). All production pools use `ThreadSafePolicy`.
- Shared immutable data via `shared_ptr<const T>` (vocabulary, `TSharedMeasurement`, `TDistortionModelPtr`): the model for `MapSnapshot`.
- The library throws almost nowhere today (only `io/read_image.cpp`); D-09 introduces a policy.

### Integration Points
- `libs/events/include/voyahslam/events/observable.h` (`ConnectAsync`, `ConnectSync`, `Notify`), `event_node.h` (`AddObserver`, `AddSyncObserver`, `inbox_.SetWakeSignal`), `async_observer.h`, `listener.h` (`current_` is recycled in its destructor).
- `libs/slam/include/voyahslam/slam/slam.h` (wiring, the `global_ba_` invariant), and `tracker.h`, `mapper.h`, `loop_closer.h`, `global_bundle_adjuster.h` (pools, accessors, Start/Stop).
- `libs/common/include/voyahslam/features/orb_feature_extractor.h` and `libs/algs/src/features/orb_feature_extractor.cpp` (D-15); the callers in `examples/full_slam/*`, tests and benchmarks.
- `libs/common/include/voyahslam/logging/logger.h` and `libs/common/src/logging/logger.cpp` (D-18).
- `libs/common/include/voyahslam/images/image.h` (D-20).
- `vizualization/` (voyahviz `MapViewer`): it uses `AddObserver(viewer.inbox())` and must adopt `Subscription`s; its threading comments change with D-11.
- `examples/full_slam/{monocular,binocular}_full_slam.cpp` and `full_slam_common.h`: the `StopAll` guard to remove, and the wiring to update.
- `test_package/` and `conanfile.py`: the version bump and package recreation per release.

</code_context>

<specifics>
## Specific Ideas

- The user first required allocation-free event publishing (the reason for the pools), then on 2026-10-13 dropped it: since events became coarse transactions, one allocation per event is negligible (D-01..D-03).
- The binding's main case is a GC or `close()` on another thread. D-11 makes Start/Stop safe there. Disconnecting while running stays a loud failure (D-07) by the user's choice, so the binding must still stop `Slam` before closing listeners. That is now an enforced rule rather than a silent crash.

</specifics>

<deferred>
## Deferred Ideas

- **Disconnect while running (lock-free):** a shared queue core plus a "closed" flag, with the producer skipping closed observers. Declined for this phase in favour of "only while stopped". Revisit if the binding finds the stop-first rule too restrictive.
- **`voyahslam::Session` facade** (`Slam` plus `MapFollower` in one object for JNI): design it after the binding uses `MapFollower`, in the library or in voyah_jni.
- **voyahviz `SceneState` on `MapFollower`:** dogfood the follower in the viewer later.
- **Android logcat default sink:** stays in PLAT-01 (Phase 5).
- **Lossy delivery for display-only observers:** stays in EVT-01 (Phase 5); it builds on D-01 and D-06.

</deferred>

---

*Phase: 01-api-ownership-and-lifetime-contracts*
*Context gathered: 2026-10-12*
