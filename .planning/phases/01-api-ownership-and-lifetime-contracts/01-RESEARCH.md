# Phase 1: API Ownership and Lifetime Contracts - Research

**Researched:** 2026-10-12; R1 (OWN-01, OWN-02) refreshed 2026-10-13
**Domain:** C++20 library API hardening (event lifetime, RAII subscriptions, lifecycle state machine, thread-affinity, logging, image views), for a Java/JNI consumer whose objects die in any order
**Confidence:** HIGH on code facts (every file below was read this session); MEDIUM on the design recommendations marked `[ASSUMED]` (they sit inside "Claude's Discretion" or expose a gap in a locked decision)

> **Revision 2026-10-13 (R1 only: audit items #3 + #2, OWN-01 and OWN-02).** The user superseded
> CONTEXT D-01..D-04 (shared intrusively-counted pool core): events are now heap-allocated
> `std::shared_ptr<const T>` and `ObjectPool` / `Recyclable` are deleted (DESIGN decisions log,
> 2026-10-13 row, commit 5dd4ada). Everything that described the pool core is replaced:
> Summary, Responsibility Map rows, Standard Stack supporting rows, System Architecture Diagram,
> Patterns 1 and 2, Anti-Patterns, Don't Hand-Roll, Pitfalls 1-5 and 10, the R1 Code Examples,
> State of the Art, Assumptions A3/A4 (and new A11-A16), Open Questions 1-3 (now R1-Q1..R1-Q5),
> and the OWN-01/OWN-02 rows of the Validation Architecture, Wave 0 and Security tables.
> **Unchanged** (still valid, R2..R6): Patterns 3-9 (Pattern 8 lost one stale test reference),
> Pitfalls 6-9 and 11-18, Open Questions 4-7, Environment Availability, the OWN-03..07 test rows,
> and the CLAUDE.md constraints.
> New in this refresh: the R1 plan breakdown (section "R1 Plan Breakdown"), the change inventory
> with call-site counts (section "R1 Change Inventory"), and the bound analysis for `LoopDetected` /
> `GlobalBAFinished` (Pattern 1b).

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

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
  `Reset()`. The audit's "`~ObjectPool` asserts every slot came back" becomes moot. Publishers
  that drop on purpose today (`LoopDetected` pool of 16, `GlobalBAFinished`
  `max_events_in_flight`) keep their bound with a plain in-flight count (or the bound is
  dropped if research shows it is no longer needed) — planner's call, recorded in DESIGN.
- **D-03:** The general-purpose allocator is acceptable on the event path: it is not a lock on
  map data (DESIGN §2), and per-frame allocations already happen (measurement, replicas). An
  event may be freed on any thread (viewer, JNI or GC thread): event destructors must not depend
  on the destroying thread (DESIGN rule).
- **D-04:** Afterwards remove the `global_ba_` ordering invariant (`slam.h`, DESIGN) and the
  apps' `StopAll` teardown guard (`examples/full_slam/*_full_slam.cpp`), subject to the D-07
  question raised in research. This closes OQ 25 and makes OQ 14 (pool sizing) obsolete.

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
- **D-08:** On disconnect, or when an `AsyncObserver` is destroyed, the events still queued are
  **drained and released** (this closes TODO "AsyncObserver destruction doesn't acknowledge
  events"); safe even after the producer is gone, because events own their memory (D-01).

### Misuse errors and lifecycle (#6, OWN-04)
- **D-09:** Invalid lifecycle calls **throw a `std::logic_error` subclass** (e.g.
  `voyahslam::UsageError`) in every build; the binding maps it to `IllegalStateException`.
  This is the library's first exception policy, so record it in DESIGN (decisions log and
  conventions). It is introduced with the #3+#2 release, because D-07 needs it.
- **D-10:** The state machine is Created → Running → Stopped, and **Stopped is terminal**:
  `Start` after `Stop` throws, `Start` twice throws, and `Stop` is idempotent (destructors call it).
  A new session means a new `Slam`. Applies to `Slam` and its components (Tracker, Mapper,
  LoopCloser, GlobalBA, `WorkerThread` users).
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
  Tests and tools move to the access structs.

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

### Deferred Ideas (OUT OF SCOPE)

- **Disconnect while running (lock-free):** a shared queue core plus a "closed" flag, with the producer skipping closed observers. Declined for this phase in favour of "only while stopped". Revisit if the binding finds the stop-first rule too restrictive.
- **`voyahslam::Session` facade** (`Slam` plus `MapFollower` in one object for JNI): design it after the binding uses `MapFollower`, in the library or in voyah_jni.
- **voyahviz `SceneState` on `MapFollower`:** dogfood the follower in the viewer later.
- **Android logcat default sink:** stays in PLAT-01 (Phase 5).
- **Lossy delivery for display-only observers:** stays in EVT-01 (Phase 5); it builds on D-01 and D-06.
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| OWN-01 | Events are `std::shared_ptr<const T>`, `ObjectPool`/`Recyclable` deleted, `global_ba_` invariant and apps' `StopAll` guard removed; release-after-producer-destroyed test | Pattern 1 (self-owning events, delivery layer), Pattern 1b (bounds), Pitfalls 1-3, "R1 Change Inventory", Validation rows OWN-01 |
| OWN-02 | RAII `Subscription` for `ConnectAsync`/`ConnectSync`/`SetWakeSignal`; connect/disconnect while running fails loudly; observer destroyed first is never notified | Pattern 2 (inbox core + wiring block), R1-Q1..R1-Q3, Pitfalls 4-8 |
| OWN-03 | Thread-affinity accessors enforced; supported map read path | Pattern 5 (TestAccess), Pattern 6 (`MapFollower`), Open Question 4, Pitfalls 11-13 |
| OWN-04 | Lifecycle state machine; invalid transitions error; restart decided | Pattern 3 (atomic lifecycle), Pitfall 9, Open Question 6 |
| OWN-05 | `DetectAndExtract` concurrently safe or impossible to misuse | Pattern 7 (workspace), Pitfall 14 |
| OWN-06 | `Logger::Initialize` retargets earlier loggers; `sinks_` protected | Pattern 8 (`dist_sink_mt`), Pitfall 15 |
| OWN-07 | `Image` default-init, row-wise view ops, non-const `Set`, aliasing documented | Pattern 9, Pitfall 16 (extra findings: `CopyTo`, constructor overflow) |
</phase_requirements>

## Summary

The phase is six releases of API hardening inside one codebase: R1 (#3 events + #2 subscriptions, 0.4.0 if each release bumps the minor), then #5, #6, #7, #8, #9. R1 got much smaller in design and larger in churn after the 2026-10-13 decision. Design: an event is one `std::make_shared<T>()` object published as `std::shared_ptr<const T>`; the non-template delivery layer carries `std::shared_ptr<const Event>` (the `Event` base keeps only `type`, `map_id`, `priority`; the two function pointers go) and the consumer's existing `switch` keeps its `static_cast` on a `const Event&`. Every production consumer already takes `const events::Event&` and none writes to a received event (Q2 below), so `const` costs nothing. Churn: 12 production publish sites, 16 test files that create, hold or consume events (14 of them create events through pools), 69 wiring call sites, 17 pool tests and about 11 pool-specific event tests to delete or rewrite, and the DESIGN §6/§6.1 rewrite `[VERIFIED: grep and ctest -N this session, see "R1 Change Inventory"]`.

Three findings change earlier assumptions. (1) With pools gone, the remaining destruction-order hazard is the **wiring** (a `Subscription` must survive its producer, the JNI case; a producer must survive its observers). The shared `Wiring` block stays, and a shared **inbox core** (queue, priority counter, closed flag, owned `WakeSignal`) held by the observer, its `Listener`s and every producer entry makes "observer destroyed first" safe by construction, so the counter-and-terminate rule of the earlier draft (A3) is not needed. This is the one recommendation that goes beyond the locked decisions (R1-Q1). (2) The two bounded publishers (`LoopDetected` pool of 16, `GlobalBAFinished` pool of 2) can drop their bound: both producers are driven by the Mapper's own transactions, the LoopCloser already has its own back-pressure (`max_query_backlog = 8`), and dropped GBA results are a known defect (TODO line 573) that a bound-free publish removes (R1-Q4; it also makes Phase 3 success criterion 3 moot). (3) D-08's "drain on disconnect" must become "release at `~AsyncObserver`" because a viewer inbox is fed by two producers (R1-Q3); because events own their memory, a queued event is simply valid until someone drops it, and `moodycamel::ConcurrentQueue`'s destructor already destroys undequeued elements `[VERIFIED: probe run this session, 50100 of 50100 events destroyed, TSan clean]`.

Tooling is unchanged: a ThreadSanitizer tree (`cmake-build-tsan`, needs `setarch -R`) and an ASan tree already exist; Conan, NDK r30 and the 0.3.0 packages are in the local cache. Baseline `ctest -N` lists 773 tests `[VERIFIED: ctest -N this session]` (746 unit per the previous session's `check` run).

**Primary recommendation:** Build R1 in four waves: (1) the `voyah_events` layer as a tracer (`UsageError`, `Event`/`EventPtr`, inbox core, `Observable` + `Wiring` + `Subscription`, all six event structs, rewritten events tests, 24-permutation destruction-order test, TSan + ASan), (2) production migration of `voyah_slam`, `voyahviz` and the examples (bounds dropped, `StopAll` replaced by a subscription holder declared before `Slam`), (3) test migration in three disjoint file groups, (4) delete `memory/{object_pool,recyclable,sync_policy}` and their tests, rewrite DESIGN §6/§6.1, bump the version, run `check_all`, TSan/ASan, three `conan create` runs, and stop at a human checkpoint for "merge and push".

## Architectural Responsibility Map

The "tiers" here are the repo's libraries and consumers (the dependency rule: a lower layer cannot include an upper one, DESIGN section 7).

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| `UsageError` (exception policy) | `voyah_common` | - | Lowest layer that needs it: `voyah_events` throws it. `[ASSUMED]` header name `voyahslam/errors.h` |
| `Event` base, `EventPtr`, six event structs | `voyah_events` (`events/`) | - | Already there; `Event` needs only `map::MapId` from common. `ObjectPool`/`Recyclable`/`sync_policy.h` leave `voyah_common` (`memory/` keeps `column_buffer.h`) |
| Inbox core, `AsyncObserver`, `Listener`, `Observable`, `Wiring`, `Subscription`, `RunState` | `voyah_events` | - | `Observable`/`EventNode`/`AsyncObserver` live here and depend on common only |
| Publishing (`make_shared` + `Notify(std::move(p))`) | each producer component in `voyah_slam` | - | Tracker, Mapper, LoopCloser, GlobalBA keep owning "what is published"; no component owns an event after `Notify` |
| Start/Stop state machine of each component, `Track()` check | `voyah_slam` (components) over `voyah_events` `WorkerThread`/`EventNode` | `Slam` (composite state) | Components own the threads; `Slam` composes four of them |
| `MapFollower`, `MapSnapshot` | `voyah_slam` | - | Wraps `AtlasReplica`/`MapReplica` (`voyah_slam`); not in `voyahviz` (deferred) |
| `*TestAccess` friends | `voyah_slam` headers (forward declaration) + `libs/slam/tests` (definition) | - | `MapperTestAccess` pattern; definition lives with the tests |
| Extractor `Workspace` | `voyah_common` (declaration, next to `OrbFeatureExtractor` and `orb/image_pyramid.h`) | `voyah_algs` (definition) | The extractor class is declared in common and defined in algs (DESIGN section 7 "Placement") |
| `Logger` | `voyah_common` | - | `logging/logger.{h,cpp}` |
| `Image` | `voyah_common` (`images/image.h`) | tests currently in `voyah_algs` tests | Header-only template in common; no image test exists in `libs/common/tests` |
| Example wiring / teardown | `examples/full_slam`, `voyahviz` | - | Consumers: they hold `Subscription`s and decide the stop order |

## Standard Stack

This phase adds **no new external package**. Everything is existing code plus APIs of the pinned dependencies and the C++20 standard library (`std::shared_ptr`, `std::make_shared`, `std::atomic::wait`).

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `std::shared_ptr<const Event>` / `std::make_shared` | libstdc++ 13.3 (g++ 13.3.0) | The event handle: fan-out copies it, dropping it acknowledges | D-01; control block remembers the dynamic type, so `Event` needs no virtual destructor. `[VERIFIED: probe compiled and ran this session: shared_ptr<const Event> from make_shared<Derived>() with a protected non-virtual Event destructor, 50000 events fanned out to two moodycamel queues and dropped on consumer threads, TSan clean]` |
| concurrentqueue (moodycamel) | 1.0.4 `[VERIFIED: conanfile.py:67 "concurrentqueue/1.0.4"]` | Inbox queue, element type becomes `std::shared_ptr<const Event>` | Already the event transport. Its producer destructor destroys elements that were never dequeued `[VERIFIED: concurrentqueue.h:1790 (in ~ExplicitProducer) "// Destruct any elements not yet dequeued."; and the probe: 100 events left in a destroyed queue were destroyed]`, which makes "queue destroyed with events inside" leak-free by itself |
| spdlog | 1.17.0 `[VERIFIED: conanfile.py:66 "spdlog/1.17.0"]` | `dist_sink_mt` for the shared sink (D-18, R5) | Part of spdlog; `set_sinks` takes the sink mutex, so it is safe against concurrent `log` `[VERIFIED: dist_sink.h:42-45 "void set_sinks(std::vector<std::shared_ptr<sink>> sinks) {" / "std::lock_guard<Mutex> lock(base_sink<Mutex>::mutex_);" / "sinks_ = std::move(sinks);"]` |
| GoogleTest | 1.15.0 `[VERIFIED: conanfile.py:79 "gtest/1.15.0"]` | Tests, incl. death tests for the D-07 terminate | Existing framework |
| Eigen | 3.4.0 | `MapSnapshot` storage (SoA `TPoint3DF` columns) | Existing |

### Supporting
| Tool | Purpose | When to Use |
|------|---------|-------------|
| `cmake-build-tsan` (scratch tree, gitignored) | ThreadSanitizer runs | Every release; event + Slam + integration tests |
| `cmake-build-asan` (scratch tree) | AddressSanitizer + LeakSanitizer on the events tests and the destruction-order tests | R1: the "event outlives everything" tests are only meaningful under ASan (a use-after-free is silent otherwise); g2o-using tests abort under ASan (TODO PLAT-01), so filter to `voyah_events_tests` `[ASSUMED]` extra gate, not in the success criteria |
| Conan 2 + NDK r30 | `conan create` for linux-gcc13 static/shared and android-arm64 | End of each release |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| `shared_ptr<const Event>` through the non-template layer (D-01, locked) | A template layer per event type | Declined 2026-09-30 (template bloat; concrete types only known at the consumer's `switch`); unchanged |
| Shared inbox core held by observer, listeners and producer entries (recommended, R1-Q1) | Per-`AsyncObserver` live-subscription counter and `std::terminate` in `~AsyncObserver` (earlier draft, A3) | The counter adds a loud failure the user has not seen; the core makes the order harmless. Costs one allocation per `AsyncObserver` (not per event) |
| Shared `Wiring` block for subscriptions (recommended) | Subscription holds a raw `Observable*` | Dangles when the producer dies first (the JNI case); see Pattern 2 |
| `std::atomic::wait` for Start/Stop losers (recommended) | spin / `std::mutex` in `WorkerThread` | A mutex on the control plane is allowed by principle 1 (map/event path only), but D-11 says CAS; `atomic::wait` is already used by `WakeSignal` |

**Installation:** none. **Version verification:** versions above are the pins in `conanfile.py`; nothing is added.

## Package Legitimacy Audit

No external package is installed or added by this phase, so the legitimacy gate (`package-legitimacy check`) has nothing to check.

| Package | Registry | Age | Downloads | Source Repo | Verdict | Disposition |
|---------|----------|-----|-----------|-------------|---------|-------------|
| (none) | - | - | - | - | - | - |

**Packages removed due to [SLOP] verdict:** none
**Packages flagged as suspicious [SUS]:** none

## Architecture Patterns

### System Architecture Diagram

```
 Producer thread (Tracker / Mapper / LoopCloser / GlobalBA worker)        Consumer threads (component workers, viewer, JNI dispatcher, GC cleaner)
 ──────────────────────────────────────────────────────────────────        ───────────────────────────────────────────────────────────────────────
  p = std::make_shared<T>()            (one allocation; T is a plain struct, no pool, no Reset)
  fill payload; set map_id, priority   (p is a mutable shared_ptr<T> only up to here)
  observable.Notify(std::move(p))      (rvalue-only template: the producer's handle is gone;
     │                                  p converts to EventPtr = shared_ptr<const Event>)
     ├─ sync handlers, inline:  fn(ctx, const Event&)
     └─ for entry in Wiring.async:  entry.inbox->Push(EventPtr copy)         ─► InboxCore.queue ─► Listener::GetNextEvent() ─► EventPtr (by value)
            (closed inbox: the copy is dropped here)                                              consumer reads const Event&, drops EventPtr = acknowledgement
                                                                                                  the LAST drop frees the event on THAT thread (D-03)

 App / owner thread                                              Ownership of the wiring (no raw pointer crosses an object boundary)
 ──────────────────                                              ─────────────────────────────────────────────────────────────────────
  sub = producer.AddObserver(observer)   (throws UsageError       Wiring (shared_ptr; held by Observable AND by each Subscription)
     if the producer is Running, D-07)                             ├─ std::atomic<RunState> state   (set by the component's Start/Stop)
  producer.Start()   Created ─CAS─► Running                        ├─ async entries: {id, shared_ptr<InboxCore>}   (written only when !Running)
  producer.Stop()    Running ─────► Stopped (terminal)             └─ sync entries:  {id, fn, ctx}
  sub.reset()        only if !Running, else throws
  ~Subscription      Running → log + std::terminate (D-07)        InboxCore (shared_ptr; held by AsyncObserver, its Listeners, producer entries)
  ~AsyncObserver     InboxCore::Close(): closed flag, then        ├─ moodycamel::ConcurrentQueue<EventPtr>
                     drain (events released on this thread)       ├─ std::atomic<uint32_t> pending_priority
                                                                   ├─ std::atomic<bool> closed
                                                                   └─ WakeSignal (owned; replaces SetWakeSignal(WakeSignal*))

 Map read path (OWN-03, R2):  Mapper ─MapTransaction─► follower.inbox ──(app thread)──► MapFollower::Update() ──► AtlasReplica ──► Snapshot() → shared_ptr<const MapSnapshot>
```

### Recommended Project Structure
```
libs/common/include/voyahslam/
├── errors.h                      # UsageError : std::logic_error            (new, [ASSUMED] name/location)
├── memory/column_buffer.h        # stays (a grow-only buffer, no longer tied to a pool)
├── memory/{object_pool,recyclable,sync_policy}.h, src/memory/object_pool.cpp   # DELETED (D-02)
├── features/orb_feature_extractor.h   # + Workspace, workspace overload     (changed, R4)
├── images/image.h                # fixes                                    (changed, R6)
└── logging/logger.h              # (changed, R5)
libs/events/include/voyahslam/events/
├── event_types.h                 # Event without recycle_/set_ref_count_; EventPtr   (changed)
├── {frame_tracked,global_ba_finished,initialization_measurement,loop_detected,map_transaction,tracking_lost}.h   # plain structs (changed)
├── inbox_core.h                  # queue + priority counter + closed flag + WakeSignal   (new, [ASSUMED] name)
├── async_observer.h / listener.h / observable.h / event_node.h                         (changed)
├── subscription.h                # Wiring + Subscription + RunState                  (new)
└── worker_thread.h               # unchanged in R1
libs/slam/include/voyahslam/
├── mapping/map_follower.h, map_snapshot.h   # (new, R2)
└── {tracking,mapping,loop_closing,global_ba,slam}/…                          (changed)
libs/slam/tests/
├── test_access.h (or mapping/mapper_test_access.h)   # shared *TestAccess definitions (new, R2)
└── …
```

### Pattern 1: Self-owning events through the non-template delivery layer (D-01..D-03)
**What:** `Event` keeps only data; every event is created with `std::make_shared<T>()`, filled while it is a mutable `shared_ptr<T>`, and handed over with `Notify(std::move(p))`. The delivery layer carries `EventPtr = std::shared_ptr<const Event>`; consumers keep their `switch (event.type)` and `static_cast<const T&>(event)`.
**Current code to change** `[VERIFIED: event_types.h:18-26 "struct Event {" / "void (*recycle_)(Event*);" / "void (*set_ref_count_)(Event*, int);" / "map::MapId map_id{};" / "bool priority = false;"]`, `[VERIFIED: event_types.h:9-16 "enum class EventType : uint8_t {" ... "MapTransaction," "LoopDetected," "FrameTracked," "InitializationMeasurement," "GlobalBAFinished," "TrackingLost,  // keep last: consumers size per-type tables by it"]`, `[VERIFIED: async_observer.h:62 "moodycamel::ConcurrentQueue<Event*> queue_;"]`, `[VERIFIED: listener.h:46-55 "Event* GetNextEvent() {" ... "current_->recycle_(current_);"]`, `[VERIFIED: observable.h:39 "event->set_ref_count_(event, static_cast<int>(async_observers_.size()));"]`.
**Answers to the research questions (all `[ASSUMED]` design unless quoted):**
1. **Delivery layer (minimal design).**
   - `Event`: `EventType type; map::MapId map_id{}; bool priority = false;` plus a protected `explicit Event(EventType)` and a **protected non-virtual destructor**. Destruction goes through the `make_shared` control block, which remembers `T`; a protected destructor makes `std::shared_ptr<Event>(raw)` a compile error. `[VERIFIED: probe compiled and ran this session]`
   - `using EventPtr = std::shared_ptr<const Event>;` Queue element type: `moodycamel::ConcurrentQueue<EventPtr>` `[VERIFIED: probe]`. A typed pointer is needed only if a consumer retains an event (none does; `static_pointer_cast<const T>` is then available); otherwise `static_cast<const T&>(*event)` as today.
   - `Listener` shrinks to a handle on the inbox core: `explicit Listener(std::shared_ptr<InboxCore>)`, `EventPtr GetNextEvent()` (empty when none), freely copyable and movable. There is **no `current_`**, no destructor acknowledgement, no move-semantics test: the caller owns the returned pointer, and a loop `while (events::EventPtr event = listener.GetNextEvent()) { ApplyEvent(*event); }` releases each event at the end of its iteration, exactly when the old "next `GetNextEvent()` acknowledges" rule did.
   - `AsyncObserver::TryPop(EventPtr& out_event)` (project convention `out_` prefix) for the Mapper's batch; `queue()` becomes private (closes the TODO "make `AsyncObserver::queue()` private"; its single user is `slam_test.cpp:1717`, replace with `size_approx()`).
   - `Observable::Notify` is `template <typename T> void Notify(std::shared_ptr<T>&& event)` with `static_assert(std::is_base_of_v<Event, T>)`: an **lvalue is a compile error**, so a producer cannot keep a mutable alias to a published event `[VERIFIED: probe compiled: "cannot bind rvalue reference of type 'std::shared_ptr<A>&&' to lvalue"]`.
2. **Does any consumer write to a received event?** No. Production consumers take `const events::Event&` already: `[VERIFIED: tracker.h:254 "void ApplyEvent(const events::Event& event);"]`, `[VERIFIED: loop_closer.h:143 "void ApplyEvent(const events::Event& event);"]`, `[VERIFIED: global_bundle_adjuster.h:93 "bool ApplyEvent(const events::Event& event, std::optional<map::MapId> map_id = std::nullopt,"]`, `[VERIFIED: map_replica.h:77 "void Apply(const events::Event& event);"]`, `[VERIFIED: mapper.h:362 "void ApplyEvent(const events::Event& event);"]`, `[VERIFIED: scene_state.h:46 "void Apply(const voyahslam::events::Event& event);"]`. Sites that hold a non-const `Event*` only to read and must switch to `EventPtr`: `tracker.cpp:82`, `mapper.h:734 "std::vector<events::Event*> batch_;"` with `mapper.cpp:68-82`, `full_slam_common.h:200`, `map_viewer.cpp:160`, and the binding's `slam_session.cpp:180`. Tests cast with non-const `static_cast<T*>(event)` in 13 places (`event_system_test.cpp:289,292`; `slam_test.cpp:130,165`; `global_bundle_adjuster_test.cpp:209`; `tracker_test_fixture.h:144,163`; `loop_closer_test.cpp:398`; `mapper_test.cpp:226,233,762,787`; `tracker_test.cpp:72`), all read-only; the fix is `const`. A grep for `const_cast` over `libs/*/tests` found none on events. The compiler will list any write; none is expected, so **no non-const exception is needed**.
3. **Sync handlers (`ConnectSync` / `AddSyncObserver`).** They stay (D-06 puts them under `Subscription`) with the signature `void (*fn)(void* ctx, const Event&)`; `Notify` runs them first, inline, on the producer thread, then fans out. They are test-only today: no production code connects one `[VERIFIED: grep AddSyncObserver|ConnectSync over libs/slam/include libs/slam/src examples vizualization: no hits; tests only: event_system_test 4, tracker_test 2, mapper_test 3]`; the Mapper tests use them to inject an urgent event mid-step (`mapper_test.cpp:1804-1815`, `4083-4095`). What is **gone**: the 2026-09-30 rulings "sync-only notify skips ref counting" and "stack-allocated events possible for sync" (DESIGN decisions log, 2026-09-30 rows): there is no ref count to skip, and an event is always a `make_shared` object. Tests `ObservableSyncHandler` and `ObservableNoAsyncSkipsRefCount` (stack `TestEvent`) become `make_shared<TestEvent>()` tests; record the supersession in DESIGN.
4. **Priority.** `Event::priority` stays a plain `bool` in the base, set by the producer before `Notify`. `InboxCore::Push` increments `pending_priority` before the enqueue when `event->priority`, `TryPop` decrements when it is set (as `async_observer.h:43-45` and `:52-54` do today). The old invariant "the flag is stable from push to pop" `[VERIFIED: async_observer.h:40-42 "The flag is stable from push to pop: it is set before publishing and"]` was a convention backed by "the event is not reused until every observer released it"; it is now **enforced by the type** (`const`) and no reuse exists. `pending_priority()` stays on `AsyncObserver` (the Mapper's `should_stop` reads it: `[VERIFIED: mapper_quality.cpp:422 "ba_options.should_stop = [this] { return inbox_.pending_priority() > 0; };"]`). Delete `TryPopDecrementsAndReusedEventIsNotCounted` (reuse no longer exists); keep `PriorityCountTracksPushPop` on `make_shared` events; `pending_priority_counter()` has no user outside that test (drop it).
5. **`TSharedMeasurement` release timing.** Today a pooled event keeps its measurement pointer until its slot is reused `[VERIFIED: frame_tracked.h:49-50 "// Called by the pool on reuse; keeps the vectors' capacity and releases" / "// the measurement."; event_system_test.cpp:376-377 "A pooled event keeps its slot's object alive; on reuse it lets go of the shared" / "measurement it carried."]`; after the change it is released when the last `EventPtr` is dropped, which is earlier (the old behaviour pinned up to the pool size of measurements `[ASSUMED]`). No consumer relies on the old timing: the only checks are `RecycledEventsReleaseTheirMeasurement` (`event_system_test.cpp:378`, to be rewritten as "the last drop releases it") and `slam_test.cpp:841 "EXPECT_GE(mapper_frame->shared_measurement().use_count(), 3);"`, which counts the Mapper's Frame and the two replicas, not events.
**Rules that make it correct:**
- Event destructors must not depend on the destroying thread (D-03; DESIGN rule). All six event types hold only `std::vector`, Eigen, `shared_ptr<const ...>`, `ColumnBuffer` and plain data `[VERIFIED: frame_tracked.h, global_ba_finished.h, map_transaction.h, loop_detected.h, tracking_lost.h, initialization_measurement.h read this session]`.
- Remove from each event type: the `Recyclable` base, `Reset()`, and the two lambda arguments of the `Event` constructor. `map_id` and `priority` are still "set by the producer on every publish" (now trivially true: a fresh object).
- Mapper batching: `std::vector<events::EventPtr> batch_`; keep `batch_[i].reset()` after each `ApplyEvent` (`mapper.cpp:82` is the "acknowledge" line today) so memory is released per event, not per batch.
- Producers: `CreateTransaction()` returns `std::shared_ptr<events::MapTransaction<FE>>`; the seven Mapper sites keep filling through `*created` / `created->` and end with `observable_.Notify(std::move(created))` (`mapper.cpp:237,603,952`, `mapper_global_ba.cpp:135`, `mapper_quality.cpp:329`, `mapper_loop.cpp:101`, `mapper_merge.cpp:173` `[VERIFIED: grep "observable_.Notify"]`). No site touches the event after `Notify`.
- `ColumnBuffer::Resize` keeps working on a fresh buffer; its "reuse capacity" benefit is gone with the pool, a `reserve`-style hint is a later micro-optimisation, not needed (events are under 100 per second in total, DESIGN 2026-10-13 row).

### Pattern 1b: What replaces the `LoopDetected` / `GlobalBAFinished` bounds (D-02, "planner's call")
**Why they exist** `[VERIFIED: loop_closer.cpp:247-253 "auto* event = loop_detected_pool_.TryCreate();" / "++num_dropped_loops_;" / "LoopDetected events in flight"; global_bundle_adjuster.cpp:150 "auto* event = result_pool_.TryCreate();"; DESIGN §6 table: "Pool of 16, published with `TryCreate`" and "Pool of `GlobalBaOptions::max_events_in_flight` (2), published with `TryCreate` (exhausted: the result is dropped and counted)"]`: back-pressure when the Mapper falls behind (the pools were fixed-size so a slow consumer had to be answered with a drop).
**Recommendation: drop both bounds** (`LoopCloserOptions::max_events_in_flight`, `GlobalBaOptions::max_events_in_flight`, `num_dropped_loops()`, `num_results_dropped()`, the two "dropped" log lines and the `Stop()` log count). Reasons:
- Both producers are paced by the Mapper's own output. The LoopCloser turns each added Frame of an applied `MapTransaction` into a query `[VERIFIED: loop_closer.cpp:105-107 "for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {" ... "queries_.push_back(added.id);"]` and drops queries beyond `max_query_backlog = 8` `[VERIFIED: common_region.h:70 "size_t max_query_backlog = 8;"]`, so `LoopDetected` events are at most one per query: a stalled Mapper stops producing the transactions that feed new queries. GlobalBA runs only on a transaction that adds a loop edge (one per correction, the Mapper's own output), so at most a handful of results can be outstanding.
- A `GlobalBAFinished` is the one big event (about 1.2 MB on corridor1 `[CITED: docs/DESIGN.md §6 table row GlobalBAFinished]`), but a stale one is rejected by the Mapper by epoch, and the old drop was a **known defect**: TODO "A dropped result is not retried" (`docs/TODO.md:573`) and Phase 3 success criterion 3 / GBA-01 ask for a retry. Never dropping removes the defect.
- `MapTransaction` and `FrameTracked` are already never dropped (DESIGN 2026-10-12 row); this makes the rule uniform.
**Impact to carry into planning:** delete the drop tests (`loop_closer_test.cpp:749-763`, `global_bundle_adjuster_test.cpp:510-519`) and replace them with "a result / loop is delivered even while an earlier one is still held"; remove `global_ba_results_dropped` from the `slam_test.cpp` counters (`:543,693,699,1348`); close TODO line 573; and **tell the user** that `.planning/ROADMAP.md` Phase 3 criterion 3 and `REQUIREMENTS.md` GBA-01 ("a dropped `GlobalBAFinished` is retried") become moot (cross-phase edit, R1-Q4).
**If the user wants a bound kept** (simplest form, `[ASSUMED]`): an in-flight count carried by the event itself, so it works after the producer is gone:
```cpp
// Sketch only. The counter is a shared_ptr so a late drop (any thread, after the producer died) is safe.
auto in_flight = std::make_shared<std::atomic<size_t>>(0);     // member of the publisher
if (in_flight->load(std::memory_order_relaxed) >= max_in_flight) { ++num_dropped_; return; }
in_flight->fetch_add(1, std::memory_order_relaxed);
std::shared_ptr<events::LoopDetected> event(new events::LoopDetected, [in_flight](events::LoopDetected* e) {
    delete e;
    in_flight->fetch_sub(1, std::memory_order_relaxed);
});
```
This costs a second allocation per event (irrelevant at a few events per second) and keeps `num_dropped_*`; it is not recommended because it re-creates the defect.

### Pattern 2: Inbox core + shared wiring block + `Subscription` (D-05..D-08)
**What:** two shared blocks, both allocated once per component or observer (never per event).
- `Wiring` (per `Observable`, held as `shared_ptr` by the `Observable` and every `Subscription`): the `std::atomic<RunState>` of Pattern 3, the async entries `{id, shared_ptr<InboxCore>}` and the sync entries `{id, fn, ctx}`. Pattern 3's `std::atomic<RunState>` lives in this block (not as a plain `EventNode` member; `EventNode` forwards it), so a `Subscription` can read it after the producer is gone. `Notify` iterates the vectors through a cached raw pointer (no refcount traffic on the hot path). A `Subscription` holds `shared_ptr<Wiring>` + the entry id, so it is valid after the producer is gone; disconnecting then only edits the shared block.
- `InboxCore` (per `AsyncObserver`; held by it, by its `Listener`s and by the producers' async entries): the queue of `EventPtr`, `pending_priority`, an `atomic<bool> closed`, and the observer's **`WakeSignal`**. `AsyncObserver::~AsyncObserver` calls `Close()`: set `closed`, then drain the queue (D-08). `Push` on a closed core drops the event (the copy goes out of scope) and does not touch the signal's owner. Because the core owns the signal, the raw `WakeSignal*` of `SetWakeSignal` disappears; consumers wait on `observer.wake_signal()`.
**Why this is simpler than the earlier draft:** the earlier draft needed a per-observer subscription counter and a terminate in `~AsyncObserver` (A3) because a producer entry held a raw `AsyncObserver*`. With the core shared, "observer destroyed first" is **memory-safe by construction**: nothing dangles, a later `Start()` pushes into a closed core, and the events are dropped. Pools no longer enter the picture at all (the events own their memory), so the pool half of the old Pattern 2 is gone.
**Destruction-order matrix (what the 24-permutation test must show):**
| Dies first | Producer Created/Stopped | Producer Running |
|------------|--------------------------|------------------|
| `Subscription` | entry removed; fine | `std::terminate` after a log line (D-07, death test) |
| `AsyncObserver` | core closed and drained; entry stays, later pushes are dropped; the `Subscription` later removes the entry | same: pushes dropped, no UAF, no terminate |
| Producer (`Slam`, component) | `Wiring` survives inside the `Subscription`; `state` is Stopped; the `Subscription` disconnects harmlessly | impossible: the component's destructor stops it first (all four call `worker_.Stop()` in their destructors `[VERIFIED: tracker.cpp:53-54, mapper.cpp:50-51, loop_closer.cpp:19-20, global_bundle_adjuster.cpp:13-16]`) |
| `Listener` | nothing (it only shares the core) | nothing |
| Events held by anyone | valid, freed once on the thread of the last drop | same |
**Mutation rule (lock-free, TSan-clean):** entries are written only when `state != Running`; `Start()` publishes `Running` with a release store before the thread is created (thread creation is a happens-before edge); `Stop()`'s join gives the reverse edge for later disconnects. No mutex. A bare `Observable` (tests) has no component driving `state`, so it is never "running": D-05 is then not enforced there, hence the D-05/D-07 tests must use an `EventNode`.
**Current code** `[VERIFIED: observable.h:15 "async_observers_.push_back(&observer);"]`, `[VERIFIED: observable.h:19 "sync_handlers_.push_back({fn, ctx});"]`, `[VERIFIED: event_node.h:24 "void AddObserver(AsyncObserver& observer) { observable_.ConnectAsync(observer); }"]`, `[VERIFIED: event_node.h:19 "EventNode() { inbox_.SetWakeSignal(&wake_signal_); }"]` (members `observable_`, `inbox_`, then private `wake_signal_`: the signal is destroyed *first*, a latent hazard that the owned signal removes; `EventNode::wake_signal()` then returns `inbox_.wake_signal()` and `WorkerThread worker_{wake_signal()}` keeps working because every component's `worker_` is its last member `[VERIFIED: loop_closer.h:190, mapper.h:780, tracker.h:353, global_bundle_adjuster.h:127 "events::WorkerThread worker_{wake_signal()};"]`).
**Binding impact** `[VERIFIED: slam_session.cpp:83-84 "inbox_.SetWakeSignal(&wake_);" / "slam_->tracker().AddObserver(inbox_);"; slam_session.h:121 "// declared before slam_ and outlives it. events_ holds no event once the"]`: the session drops its own `WakeSignal`, waits on `inbox_.wake_signal()`, keeps the `Subscription` in a member, and loses the declaration-order comment. Tell the voyah_jni session: `Event*` becomes `EventPtr`, `SetWakeSignal` is gone, `AddObserver` returns a `Subscription`.
**`SetWakeSignal(nullptr)`** is tested today (`AsyncObserverTest.PushWithoutSignalStillEnqueues`, `wake_signal_test.cpp:59`); the equivalent becomes "push to an observer nobody waits on still enqueues".

### Pattern 3: One atomic lifecycle per component (D-09..D-12)
**What:** `enum class RunState : uint8_t { Created, Starting, Running, Stopping, Stopped }` in an `std::atomic<RunState>` owned by `EventNode` (base of Tracker, Mapper, LoopCloser, GlobalBA `[VERIFIED: tracker.h "class Tracker : public events::EventNode", mapper.h, loop_closer.h:78, global_bundle_adjuster.h:45]`). `Start`: CAS `Created→Starting`, create the thread, store `Running`; any other state throws `UsageError`. `Stop`: `Created→Stopped` directly; `Running→Stopping→Stopped` after join; if another thread is `Starting` or `Stopping`, **wait** (`state_.wait`) until it settles, so a concurrent destructor never returns while the worker still runs. `Track()` throws `UsageError` if `Stopped`.
**Put the state in R1, not R3:** D-05 needs "is the producer stopped" in R1. Introduce the full enum in R1 and use it only for wiring checks; R3 (#6) adds the throwing transitions. This avoids reworking `EventNode` twice.
**Current code** `[VERIFIED: worker_thread.cpp:13-14 "if (running()) return; if (thread_.joinable()) thread_.join();"]` (Start twice is a silent no-op today; the thread member `thread_` is assigned in `Start` with no synchronization against a concurrent `Stop`, which D-11 makes a real race).
**`Slam` composes four components:** it needs its own state (Created/Starting/Running/Stopping/Stopped) so `Slam::Start` on one thread and `Slam::Stop` on another do not interleave component starts and stops, and a failed component `Start` rolls back the ones already started.

### Pattern 4: Release-by-release ordering
Follow D-21: R1 (#3+#2) -> R2 (#5) -> R3 (#6) -> R4 (#7) -> R5 (#8) -> R6 (#9). Independent items (#7, #8, #9) do not depend on each other; #5 depends on R1 (the follower is a `Subscription`ed `AsyncObserver`); #6 depends on R1 (`Lifecycle`, `UsageError`). Versions `[ASSUMED]`: 0.4.0 ... 0.9.0 (a minor bump each).

### Pattern 5: Private accessors behind friend `*TestAccess` (D-14)
**Existing pattern** `[VERIFIED: mapper.h:292 "friend struct MapperTestAccess;"]`; the struct is defined only in `mapper_test.cpp` today. Seven other test files use the accessors, so the access structs move into one shared test header (`MapperTestAccess`, `TrackerTestAccess`, plus LoopCloser/GlobalBA if in scope) and tests call e.g. `TrackerTestAccess::MapReplica(tracker)`. One definition in a header included by every TU keeps the ODR clean.
**Call-site counts to budget** `[VERIFIED: grep -c over libs/slam/tests, examples, vizualization]`: `mapper_test.cpp` has ~25 `.covisibility()`, 17 `NumVisible(`, 17 `NumFound(`, 10 `last_local_ba()`, 57 `num_keyframes(`, 16 `map_ids()`, 17 `active_map()`; `tracker_test.cpp` has 28 `map_replica()`, 13 `.atlas()`, 15 `last_matches()`; other users: `global_bundle_adjuster_test.cpp`, `loop_closer_test.cpp`, `relocalization_test.cpp`, `slam_test.cpp`, `tracker_test_fixture.h`. The migration is mechanical (sed-able) but large.
**Only non-test user:** `PrintAtlas` in `examples/full_slam/full_slam_common.h:176-178` (`mapper.map_ids()`, `mapper.num_keyframes(id)`, `mapper.active_map()`), read after `Stop()`. Replace it with a `MapFollower` (dogfoods D-13).
**Do not make private:** the no-argument `Mapper::num_keyframes()` is a documented any-thread atomic `[VERIFIED: mapper.h:207 "[[nodiscard]] size_t num_keyframes() const {" returning num_keyframes_.load(std::memory_order_acquire)]` and is used by both examples; only `num_keyframes(map::MapId)` (line 289) is Mapper-thread-only. D-14 lists "`num_keyframes()`" ambiguously.

### Pattern 6: `MapFollower` and `MapSnapshot` (D-13)
**Shape `[ASSUMED]`:**
```cpp
// libs/slam/include/voyahslam/mapping/map_follower.h  (sketch; names are discretion)
template <typename TFeatureExtractor>
class MapFollower {
public:
    events::AsyncObserver& inbox();                 // app: Subscription s = mapper.AddObserver(follower.inbox());
    size_t Update();                                // applies pending MapTransactions on the CALLER's thread; returns count
    const tracking::AtlasReplica<TFeatureExtractor>& atlas() const;   // read on the Update() thread only
    std::shared_ptr<const MapSnapshot> Snapshot() const;               // O(map); immutable; any thread may hold it
};
```
`MapSnapshot` mirrors what `SceneState` already decodes (a lightweight scene, not the heavy replica) `[VERIFIED: scene_state.h: per-map point positions + alive flags + keyframe poses T_cw]`: per map {id, epoch, point ids + positions (3xN), frame ids + `T_cw`}, plus the live map ids and the active map. Keep it plain data (no pointers into replica objects), consistent with the Pointer rule.
**Consequences of D-05 for the follower:** it can only be connected while the Mapper is not running, so it always sees every `MapTransaction` from the start. That makes it immune to the "late joining" limitation (LATE-01 stays deferred), but it also means a follower cannot be added to a running `Slam`.
**Cost to state in DESIGN:** each follower is a full `AtlasReplica` (descriptors, BoW, covisibility) per consumer, heavier than `SceneState`; a stalled `Update()` grows the Mapper's `MapTransaction` pool (growth is logged, `MapTransaction` must never be dropped). Assert single-thread use in debug (capture the thread id at the first `Update()`) `[ASSUMED]`.

### Pattern 7: Extractor workspace (D-15..D-17)
**Current** `[VERIFIED: orb_feature_extractor.h:47 "mutable orb::ImagePyramid pyramid_;"]`, `[VERIFIED: orb_feature_extractor.h:22 "[[nodiscard]] TFeatures DetectAndExtract(const images::TWorkingImage &image, size_t number_of_features) const;"]`. The pyramid is initialised from the first image's size and never re-initialised `[VERIFIED: orb_feature_extractor.cpp:300-303 "if (!pyramid_.Initialized()) pyramid_.Initialize(image.Width(), image.Height(), scale_factors_); pyramid_.Build(image);"]`; `ImagePyramid::Initialize` appends to its vectors, so calling it twice would double them `[VERIFIED: image_pyramid.cpp:11-22]`.
**Do:** nested `OrbFeatureExtractor::Workspace` holding an `orb::ImagePyramid`; `DetectAndExtract(image, n, Workspace&) const` prepares it (re-initialising when the image size changes, which today is only an `assert` in `Build`); the private helpers that read `pyramid_` (`ComputeKeyPointsOctTree`, `ComputePointsPerLevel`, `ComputeOrientation` callers at orb_feature_extractor.cpp:450-452 and 543) take `const ImagePyramid&`; add a `const` `operator[]` to `ImagePyramid` (it is non-const today, image_pyramid.h:15-18). The convenience overload builds a temporary `Workspace`.
**Anchor for correctness:** the golden test `libs/algs/tests/features/orb_extractor_golden_test.cpp` (digests, `VOYAHSLAM_RECORD_ORB_GOLDEN`) must pass **unchanged**: the workspace overload is bit-identical by construction. No other extractor member is mutated (`gaussian_filter_*` are only read, orb_feature_extractor.cpp:635).

### Pattern 8: Logger with one shared `dist_sink_mt` (D-18)
**Current** `[VERIFIED: logger.cpp:10 "sinks_ = sinks;"]`, creation by name with get-then-register `[VERIFIED: logger.cpp:22 "auto log = spdlog::get(std::string(name));" and :34 "spdlog::register_logger(log);"]`.
**Do:** a function-local `static std::shared_ptr<spdlog::sinks::dist_sink_mt>` (not a namespace-scope static: `CLASS_LOGGER` runs in component constructors, possibly during static init); `CreateLogger` builds every logger with `{dist_sink}`; `Initialize(sinks)` calls `dist_sink->set_sinks(sinks)` (empty vector keeps today's default stdout sink). Tests that capture log lines add a sink to a logger directly (`logger->sinks().push_back(sink)` with a `spdlog::sinks::ringbuffer_sink_mt`, then `sink->last_raw(n)` and `logger->sinks().pop_back()`); this keeps working because the logger's own sink vector now holds the dist sink and the test appends a second one. **Note (R1 refresh):** the only existing example of this capture pattern is `ObjectPoolTest.OverflowWarningIsRateLimited` `[VERIFIED: object_pool_test.cpp:316-338]`, which R1 deletes with the pool; copy the pattern into the new `logger_test.cpp` (R5) and into any R1 test that checks a warning (R1-Q5).
**Already true, do not re-implement:** `Logger::SetLevel` already applies to existing loggers, because `spdlog::set_level` loops over the registry `[VERIFIED: spdlog registry-inl.h:149-155 "for (auto &l : loggers_) { l.second->set_level(log_level); }"]`. What is still racy is the plain static `level_` (read in `CreateLogger`, written in `SetLevel`): make it `std::atomic`.

### Pattern 9: `Image` fixes (D-20)
**Current** `[VERIFIED: image.h:15 "Image() = default;"]` with members `[VERIFIED: image.h:128 "size_t offset_;"]` followed by `width_`, `height_`, `stride_` (uninitialised), `[VERIFIED: image.h:50 "void Set(size_t y, size_t x, size_t c, const T &value) const {"]`, `[VERIFIED: image.h:59 "for (int i = 0; i < width_ * height_; ++i) {"]` in `Convert()` (no `* C`, no offset/stride).
**Do:** brace-initialise every member (`offset_{0}, width_{0}, height_{0}, stride_{0}`), also reset the moved-from image in the move constructor/assignment (they leave `width_`/`stride_` set on a null buffer); implement `Convert`, `operator*=`, `operator*` as `for y in rows, for x in width*C` starting at `offset_ + y*stride_*C`; `Set` non-const; document that a view `(other, x, y, w, h)` shares the parent's buffer (`shared_ptr<T[]>`).
**No callers exist** (`Convert<`, `.Convert`, `operator*` have zero uses outside `image.h` and no test), so every changed operation needs a new test on a sub-image view; there is no `libs/common/tests/images` directory, so add `images/image_test.cpp` to `libs/common/tests/CMakeLists.txt`.

### Anti-Patterns to Avoid
- **Raw pointer from a `Subscription` into the producer, or from a producer entry into an `AsyncObserver`:** dangles when the other side dies first (Pattern 2). Share the `Wiring` and the `InboxCore` instead.
- **A producer that keeps a mutable `shared_ptr<T>` after `Notify`:** a data race with every consumer and a hole in `const`. `Notify` takes an rvalue only.
- **Retaining a received event "to avoid copying":** legal now (the event owns its memory), but replicas must still copy what they keep: nothing in the Tracker, LoopCloser or GlobalBA replicas may alias event storage, or one slow replica pins every transaction it saw. The existing `NewFrame::measurement` sharing is the only intended sharing.
- **A `std::mutex` on the event path** (principle 1). A control-plane atomic with `wait` is fine; the allocator's internal locks are not locks on map data (DESIGN 2026-10-13 row).
- **Draining the whole inbox on a single disconnect** (R1-Q3): the viewer's inbox receives from two producers.
- **A `Listener` that drains or closes the inbox:** `~AsyncObserver` is the only place that closes and drains; listeners only share the core.
- **Re-creating a pool-like in-flight cap by default:** see Pattern 1b.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Event lifetime across producer, fan-out consumers and any thread | Intrusive refcount, pool core, `Recyclable` | `std::shared_ptr<const T>` from `std::make_shared` | D-01; the atomic count, destruction on the last drop and `const` come for free |
| Fan-out to swappable log sinks | Own sink list + lock | `spdlog::sinks::dist_sink_mt::set_sinks` | Already mutex-protected against concurrent `log`; D-18 |
| Sleeping a losing `Start`/`Stop` | Spin loop | `std::atomic::wait/notify` (C++20) | Already how `WakeSignal` works `[VERIFIED: wake_signal.h:21-27 "counter_.notify_one();" / "counter_.wait(seen, std::memory_order_acquire);"]` |
| Cross-thread immutable map data | Copy-on-write scheme | `std::shared_ptr<const MapSnapshot>` | The project's pattern for shared immutable data (vocabulary, `TSharedMeasurement`, `TDistortionModelPtr`) |
| Thread identity check (follower, self-stop) | Custom TLS | `std::this_thread::get_id()` | Sufficient for debug asserts and the self-`Stop` guard |
| Queue drain at observer destruction | Own queue walk with a node list | `moodycamel::ConcurrentQueue::try_dequeue` loop (the queue's destructor also destroys what is left) | Keeps the priority counter right while alive |
| Leak detection for events | Own allocator tracking or `num_outstanding()` | A live-instance counter in a **test-only** event type + the ASan/LSan tree | There is no pool to count slots of; production types stay plain |
| Rejecting a lvalue `Notify` | A runtime check | `template <typename T> void Notify(std::shared_ptr<T>&&)` | Compile-time `[VERIFIED: probe]` |

**Key insight:** after this decision the only new concurrency primitives are two shared blocks (`Wiring`, `InboxCore`) and the lifecycle atomic; everything else is composition of standard, TSan-understood pieces (the project's TSan runs were clean on the event, Mapper and Slam tests for each prior feature, DESIGN lines 2480, 2688, 3414; and the shared_ptr/moodycamel hand-off probe was clean).

## Runtime State Inventory

Not a rename/refactor/migration phase in the sense of stored strings. Explicitly: **Stored data:** none (no persisted data uses these types). **Live service config:** none. **OS-registered state:** none. **Secrets/env vars:** none. **Build artifacts:** the `cmake-build-*` trees and `~/.conan2` packages of 0.3.0 become stale per release; each release needs a fresh `conan create` and the voyah_jni session pinned to the new version (`voyah_jni/conanfile.py` requires an older version string per the audit). The one cross-repo coupling is the binding: `[VERIFIED: ../voyah_jni/native/src/slam_session.cpp]` still passes `distortion_.get()` (a raw pointer, the pre-0.2.0 API), so that repo is already behind and every item here is a further break for it.

## R1 Change Inventory

Everything below was counted with `grep` / `ctest -N` this session `[VERIFIED]`; line numbers are the working tree at commit 5dd4ada.

### To delete (D-02)
| What | Where | Size |
|------|-------|------|
| `ObjectPool`, `Recyclable`, `SyncPolicy` | `libs/common/include/voyahslam/memory/{object_pool,recyclable,sync_policy}.h`, `libs/common/src/memory/object_pool.cpp` (+ `libs/common/CMakeLists.txt:11`) | 333 + 48 + 20 header lines; `sync_policy.h` is used only by the pool, `Recyclable` and the event headers/tests |
| Pool tests | `libs/common/tests/memory/object_pool_test.cpp` (+ `libs/common/tests/CMakeLists.txt:24`) | 17 tests (`ctest -N -R ObjectPool`: 17) |
| Pool-size options | `TrackerOptions::event_pool_size` (`tracker.h:93-96`), `MapperOptions::transaction_pool_size` (`mapper.h:86-89`) | 2 fields; the only users outside the library are `slam_test.cpp:1760,1778,1780` |
| Overflow accessors | `Tracker::event_pool_overflow()` (`tracker.h:198-203`), `Mapper::transaction_pool_overflow()` (`mapper.h:224-227`) | 2; used only by the two `StuckObserver...` tests in `slam_test.cpp` |
| Pool members and includes | `tracker.h:30,342-348` (3 pools), `mapper.h:40,774-775`, `loop_closer.h:25,182-185`, `global_bundle_adjuster.h:17,118-122` | 6 pools |
| Bounds (recommended, Pattern 1b) | `LoopCloserOptions::max_events_in_flight` (`common_region.h:95-97`), `GlobalBaOptions::max_events_in_flight` (`global_ba.h:24-25`), `num_dropped_loops()` (`loop_closer.h:118-119`, member `:178`), `num_results_dropped()` (`global_bundle_adjuster.h:74-75`, member `:113`) | 4 + the log lines (`loop_closer.cpp:36,249-251`, `global_bundle_adjuster.cpp:33,152-154`) |
| Per-event machinery | in each of the six event headers: the `memory::Recyclable<...>` base, `Reset()`, the two lambdas of the `Event` constructor | 6 headers (`frame_tracked.h` and `map_transaction.h` also lose the "called by the pool on reuse" comments) |
| `global_ba_` invariant | `slam.h:64-69` comment | 1 comment |
| `StopAll` guard | `monocular_full_slam.cpp:62-76`, `binocular_full_slam.cpp:66-80` | 2 structs |
| Stale tests of removed behaviour | `EventSystemTest`: `ListenerAutoRecycle`, `ListenerMoveSemantics`, `RecycledMapTransactionIsResetAndReusesItsBuffers`, `RecycledMapTransactionKeepsCapacity`, `NotifyWithOnlySyncHandlersRecyclesPooledEvent`, `NotifyWithoutConnectionsRecyclesPooledEvent`, `RecycledFrameTrackedIsReset`, `RecycledTrackingLostIsReset`, `RecycledGlobalBAFinishedIsReset`, `ObservableNoAsyncSkipsRefCount`; `AsyncObserverTest.TryPopDecrementsAndReusedEventIsNotCounted`. **Rewrite** (same intent, new mechanics): `EventTypeTag`, `ObservableSyncHandler`, `ObservableAsyncFanOut`, `CrossThreadFanOutReturnsEveryEventToPool` (now "every event is freed exactly once"), `ObservableSyncAndAsync`, `RecycledEventsReleaseTheirMeasurement`, `PriorityCountTracksPushPop`, the `LoopDetectedTypeTag`/`FrameTrackedTypeTag`/`NewEventTypeTags` tag tests; `SlamTest.StuckObserverOverflowsTrackerPoolsWithoutAborting` and `...MapTransactionPoolWithoutDropping` (keep the "nothing dropped, point ids contiguous" check; drop the overflow counters) | about 11 deleted, about 12 rewritten of the 28 `EventSystemTest|AsyncObserverTest|WakeSignalTest` tests (`ctest -N`: 28) |

### To migrate
| Area | Sites |
|------|-------|
| Publish sites (production) | Tracker `tracker.cpp:574,583,610`; Mapper `CreateTransaction` callers `mapper.cpp:225,532,929`, `mapper_global_ba.cpp:44`, `mapper_quality.cpp:311`, `mapper_loop.cpp:61`, `mapper_merge.cpp:106` and 7 `Notify` calls; LoopCloser `loop_closer.cpp:247`; GlobalBA `global_bundle_adjuster.cpp:150`: 12 `Notify` sites in total |
| Consume sites (production) | `tracker.cpp:82`, `global_bundle_adjuster.cpp:43,89`, `loop_closer.cpp:46` (all `Listener` loops: `Event*` becomes `EventPtr`), `mapper.cpp:66-86` + `mapper.h:734` (batch), `map_viewer.cpp:156-164`, `full_slam_common.h:199-203,222`; `ApplyEvent`/`Apply` signatures unchanged |
| Wiring call sites | 69 occurrences of `AddObserver`/`AddSyncObserver`/`ConnectAsync`/`ConnectSync` outside `libs/events/include`: `map_viewer_test.cpp` 13, `mapper_test.cpp` 10, `event_system_test.cpp` 10, `slam_test.cpp` 9, `slam.cpp` 6, `tracker_test_fixture.h` 4, `loop_closer_test.cpp` 3, both example apps 3 each, `tracker_test.cpp` 2, `global_bundle_adjuster_test.cpp` 2, `projection_matcher_test.cpp` 2, `map_viewer.h` 1 (a comment), `wake_signal_test.cpp` 1 |
| Test files that create events through pools | 14 files, 108 `Create()/TryCreate()` lines and 66 `ObjectPool<` declarations: `event_system_test` (42 / 24), `mapper_test` (24 / 9), `scene_state_test` (10 / 16), `tracker_test` (7 / 0, uses the fixture's `transaction_pool_`), `loop_closer_test` (7 / 2), `projection_matcher_test` (4 / 4), `map_viewer_test` (4 / 1), `wake_signal_test` (3 / 3), `local_map_test` (2 / 2), `atlas_replica_test`, `map_replica_test`, `relocalization_test`, `global_ba_test`, `global_bundle_adjuster_test` (1 / 1 each, relocalization 1 / 0) |
| Other test files | `tracker_test_fixture.h` (pool member `:312`, `Listener` `:141`, TearDown drain comment), `slam_test.cpp` (`StuckObserver`, `ProtocolRecorder`, `TryPop`, `queue()`, `async_observer_count` ×6) |
| Pattern for the 108 sites | `auto* e = pool.Create(); fill; x.Notify(e);` becomes `auto e = std::make_shared<T>(); fill; x.Notify(std::move(e));`; `e->Recycle();` lines disappear; fixture comments about "destroyed in this order so queues hold no dead pool's events" and their `TearDown` drains can go |
| Docs | DESIGN: §2 line 34 ("event pools"), §4 line 220, §5 table lines 332-334, 344, 348, §6 table rows (361-363, "Pooled (256 in the Mapper)", "Pool of 16", "Pool of `max_events_in_flight` (2)") and Rules lines 369-370, 419, 423, **§6.1 rewritten whole** (lines 455-605), viewer lines 715-738, Loop Closing line 2267, Global BA lines 2584-2587, 2602, 2672, decisions log (new row), OQ 14 and OQ 25 closed (lines 3263-3283), §10 lines 3404, 3413; TODO: lines 212-213, 225-233, 573-575, 817-818, 907-919, 941-948; `docs/ROADMAP.md:237-240`; `README.md:10` ("memory pools"); `.planning/ROADMAP.md` Phase 3 criterion 3 and `REQUIREMENTS.md` GBA-01 (bound removal); `.planning/phases/01-.../01-VALIDATION.md` row OWN-01 (still lists `ObjectPoolTest`); `.planning/codebase/*.md` are stale (refresh with `/gsd-map-codebase` later, optional) |
| Binding (`../voyah_jni`) | `slam_session.cpp:83-84,180-209` and `slam_session.h:121-126`: `SetWakeSignal` gone, `AddObserver` returns a `Subscription`, `Event*` becomes `EventPtr`, the declaration-order comment goes; the file still passes a raw `distortion_.get()` (pre-0.2.0), so it is already behind and needs one adaptation pass for 0.2.0 + 0.3.0 + this release |

## R1 Plan Breakdown

**Why slice by library and layer, not by component:** the delivery-layer signature change (`Event*` to `EventPtr`) is all-or-nothing per library, and `voyah_slam` is one library, so the Tracker, Mapper, LoopCloser and GlobalBA cannot compile one by one against the new layer. The tests of a library link into one binary for the same reason.

| Wave | Plan | Content | Gate (must be green before the next wave) |
|------|------|---------|-------------------------------------------|
| 1 | **P01 (tracer slice)** `voyah_events` | `UsageError` (`voyah_common`, with a tiny test); `Event`/`EventPtr` and the six event structs (delete `Recyclable`/`Reset`); `InboxCore`, `AsyncObserver` (owned `WakeSignal`, `Close()` drain, `TryPop`, `size_approx`, private queue), `Listener`; `Observable` + `Wiring` + `Subscription` + `RunState`; `EventNode` hooks (`BeginRun`/`EndRun` stand-ins for the component Start/Stop, name is discretion); rewrite `event_system_test.cpp` and `wake_signal_test.cpp`; new `subscription_test.cpp` (24 destruction orders, outlive-producer, last drop on another thread, `~AsyncObserver` drain, priority counter, D-05/D-07 on an `EventNode`, `*DeathTest`) | `voyah_events_tests` green in Debug, **TSan** and **ASan** (`setarch -R` for TSan). `voyah_slam` does not compile yet: expected, the branch is in progress |
| 2 | **P02** `voyah_slam` + `voyahviz` + examples (production code only) | Tracker (3 publish sites, options/accessor/pool removal, `EventPtr` loop); Mapper (7 sites, `batch_`, options); LoopCloser and GlobalBA (bounds dropped per R1-Q4, or the in-flight token); `Slam` (a `std::vector<events::Subscription>` member for its six internal wires, `global_ba_` comment removed, components' Start/Stop drive `RunState`); `MapViewer`; `TrajectoryWriter`; both apps (a subscription holder declared **before** `slam`, `StopAll` removed) | targets `voyah_slam`, `voyahviz`, `monocular_full_slam`, `binocular_full_slam` build (tests do not yet) |
| 3 | **P03** Mapper and Tracker tests (`mapper_test.cpp`, `tracker_test.cpp`, `tracker_test_fixture.h`, `local_map_test.cpp`, `relocalization_test.cpp`); **P04** LoopCloser, GlobalBA, replica, matcher and viewer tests (`loop_closer_test`, `global_ba_test`, `global_bundle_adjuster_test`, `projection_matcher_test`, `atlas_replica_test`, `map_replica_test`, `scene_state_test`, `map_viewer_test`); **P05** `slam_test.cpp` (rewrite the two `StuckObserver` tests as "events outlive the Slam", add Slam destruction-order tests, remove `global_ba_results_dropped`) | The file groups are disjoint, so the three plans can run in parallel, but all tests of a library link into one binary: verify at the end of the wave with `check` then `check_all` | `check` then `check_all` green (the 17 pool tests still run until P06 deletes them; about 11 event tests were already replaced in P01) |
| 4 | **P06** delete, document, release | Delete `memory/{object_pool,recyclable,sync_policy}` and the pool test (their absence from the tree is the proof nothing uses them); DESIGN §6/§6.1 rewrite + decisions-log row + closures; TODO/ROADMAP edits; version bump (`CMakeLists.txt`, `conanfile.py`, README, DESIGN layout line: Pitfall 17); `check_all`; TSan on `voyah_events_tests`, `voyah_slam_tests` (filters in "Code Examples"), `voyah_integration_tests`; `voyahviz_tests` in Debug; ASan on `voyah_events_tests`; three `conan create`; `[checkpoint:human-verify]`: the user's explicit "merge and push", and the voyah_jni message | all of DESIGN D-21's release list |

**Tracer-slice rationale:** P01 proves the two one-way decisions (D-01 `EventPtr`, D-06 `Subscription`) and the new inbox core on the smallest surface, with real sanitizers, before about 100 files migrate. If the inbox-core recommendation (R1-Q1) is rejected, only P01 changes.
**Ordering inside P01:** `UsageError` first (no dependencies), then event structs and `Event`, then `InboxCore`/`AsyncObserver`/`Listener`, then `Wiring`/`Subscription`/`Observable`/`EventNode`, then tests, so each task compiles `voyah_events`.
**Parallelism:** none in waves 1, 2 and 4 (shared headers); wave 3 is the only parallel wave.

## Common Pitfalls

### Pitfall 1: `const` and the producer's handle
**What goes wrong:** a helper returns `shared_ptr<const T>` before the producer has filled the event, or the producer keeps a non-const alias after `Notify`, which is a data race with every consumer (and `const` no longer protects).
**Why it happens:** the Mapper builds a transaction across many helpers (`FuseLoopPoints(..., *scratch)`, `LocalBundleAdjust(frame, MapTransaction& out_event, ...)`, `FinishRefinement(..., *refined)`), so the mutable handle lives long.
**How to avoid:** `CreateTransaction()` returns `std::shared_ptr<MapTransaction<FE>>` (mutable); helpers keep taking `MapTransaction&`; the only conversion to `const` is `Notify(std::move(p))`, an rvalue-only template (Pattern 1). Test accessors that build a scratch transaction (`mapper_test.cpp:60-67`) simply let the pointer go out of scope.
**Warning signs:** a TSan report on an event's vectors; a producer line after `Notify` that names the event.

### Pitfall 2: Events are freed on any thread
**What goes wrong:** the last holder frees the event on its own thread: the Tracker's worker may free a `MapTransaction` carrying a whole new map (descriptors and positions of a whole map in `PublishNewMap`, `mapper.cpp:928-953`) or the viewer thread a `GlobalBAFinished` of about 1.2 MB. A destructor that assumed one thread, or a free that costs more than expected on a latency-sensitive thread, shows up there.
**Why it happens:** D-03 accepts the general allocator; the pool used to keep these buffers alive.
**How to avoid:** the six event types contain only standard containers, Eigen and `shared_ptr`s `[VERIFIED: event headers read this session]`, so no destructor depends on a thread; add a unit test whose last drop happens on a second thread and records `std::this_thread::get_id()`. The free cost of a few large blocks is microseconds-scale against a 10-30 ms frame `[ASSUMED]`; measure with the corridor1 smoke only if the user asks. Aligned allocation: Eigen over-aligned members are covered by C++17 aligned `new` in `make_shared` `[ASSUMED]` (the project builds as C++20).

### Pitfall 3: A stalled observer now grows memory silently
**What goes wrong:** with pools, a consumer 256 events behind logged a growth warning (0.3.0, ownership audit #4, DESIGN 2026-10-12 row); without them the inbox just grows and nothing is logged. Display-only observers (viewer, trajectory writer) are the likely stalled ones; `MapViewer` keeps draining headless after the window closes "so producers' event pools never stall" (`map_viewer.h:36-38`, comment to reword).
**How to avoid:** keep the visibility with a warn-once-per-doubling log in the inbox (R1-Q5, recommended), and leave real back-pressure to EVT-01 (Phase 5). The two `StuckObserver` tests keep their value as "nothing is dropped and the run completes".

### Pitfall 4: Subscription in every destruction order
See Pattern 2 (the matrix) and R1-Q1. A test that destroys {producer, `Subscription`, `AsyncObserver`, `Listener`} in all 24 orders, with events published before and after, catches it; run it under ASan and TSan. A race to keep in mind: `Push` may pass the `closed` check just before `~AsyncObserver` closes and drains, so at most that one event stays in a closed core until the producer's wiring dies; it is harmless (valid, freed when the core goes) and the test must tolerate it.

### Pitfall 5: D-07 turns the examples' exception-unwinding path into a terminate
**What goes wrong:** a `Subscription` declared after `Slam` is destroyed first on unwinding while `Slam` still runs: log + `std::terminate`. A binding that closes a listener before stopping `Slam` is the same case.
**How to avoid (recommended, R1-Q2):** keep the `Subscription`s of external observers in a `std::vector<events::Subscription>` (or a small holder) declared **before** `slam`, filled after `slam` is constructed (`subs.push_back(slam.tracker().AddObserver(viewer.inbox()))`). On unwinding `~Slam` stops its producers first (its destructor calls `Stop()`), then the holder disconnects from a Stopped producer: no terminate, no `StopAll` guard, and the viewer and `TrajectoryWriter` may be declared in any order because events and inboxes own their memory. `viewer.Stop()` and `trajectory->Poll()` on the normal path stay where they are (they produce output), but nothing needs them for safety.
**Warning signs:** a "Subscription destroyed while its producer runs" log line before an abort.

### Pitfall 6: `[[nodiscard]]` is only a warning
There is no `-Werror` and no warning flags in the build `[VERIFIED: grep for -Wall/-Werror in CMakeLists.txt, libs/*/CMakeLists.txt, profiles, cmake, conanfile.py found none]`. A discarded `Subscription` compiles and disconnects immediately. **How to avoid:** put `[[nodiscard]]` on the class itself, add `-Werror=unused-result` to the test and example targets `[ASSUMED]`, and give tests a `std::vector<Subscription>` helper. `Slam` itself must store the six internal Subscriptions (slam.cpp:30-43 wiring) in a member declared **after** the components, so they are destroyed (stopped producers) before them.

### Pitfall 7: Death tests under TSan and threads
The D-07 terminate needs `EXPECT_DEATH`; there are none in the repo today `[VERIFIED: grep for EXPECT_DEATH/EXPECT_EXIT found none]`. Use `death_test_style = "threadsafe"` and name them `*DeathTest` so they can be excluded from TSan runs (fork with threads is unreliable under TSan).

### Pitfall 8: Disconnect and `Start` racing
`Subscription::reset()` on thread A while `Start()` runs on thread B must not corrupt the entry vector. With the `Starting` state in Pattern 3, disconnect treats `Starting` as "running" (throws) and `Start` treats an in-progress wiring edit as a usage error; both are detected by CAS rather than made safe.

### Pitfall 9: Start/Stop races (D-11)
(a) `thread_` is assigned in `Start` while `Stop` reads `joinable()`: serialize through the state. (b) The second of two concurrent `Stop()` calls must **wait** for the first to finish, otherwise a destructor returns while the worker runs. (c) `Stop()` from the component's own thread (a sync observer handler or a test hook, e.g. `global_ba.set_publish_hook([&]{ global_ba.Stop(); })` at `global_bundle_adjuster_test.cpp:497`) would self-join: detect `this_thread::get_id() == thread_.get_id()` and throw `UsageError` (that test drives `ProcessPending()` synchronously, so check it still passes). (d) `GlobalBundleAdjuster::stop_requested()` is documented "True from Stop() until the next Start()" `[VERIFIED: global_bundle_adjuster.h:76]`; with a terminal Stopped it becomes "true once Stopped"; update the comment and its tests. (e) `WorkerThreadTest.RestartAfterStop` `[VERIFIED: worker_thread_test.cpp:111]` encodes restart; decide whether `WorkerThread` stays restartable (the terminal rule lives in the component/EventNode) or becomes terminal too (Open Question 6).

### Pitfall 10: Tests that wire and count
`slam_test.cpp` wires an `AsyncObserver` visualizer and asserts `async_observer_count()` (6 uses `[VERIFIED: grep this session]`); with RAII the `Subscription` must be kept alive for the count to hold. The wiring call sites to convert are 69 occurrences over 14 files (table in "R1 Change Inventory"); give the fixtures a `std::vector<events::Subscription> subscriptions_` member (declared before the component under test, see Pitfall 5) and a tiny helper that pushes into it. `[[nodiscard]]` makes a forgotten one a warning only (Pitfall 6).

### Pitfall 11: Shared test-access definition
Defining `TrackerTestAccess` per test file violates the ODR within one binary if two TUs define it differently. One header, included everywhere.

### Pitfall 12: `PrintAtlas` and the examples after D-14
`PrintAtlas` reads Mapper-thread-only data after `Stop()`; once private it cannot compile. Use a `MapFollower` subscribed before `Start()`, `Update()` after `Stop()`.

### Pitfall 13: Accessors of the same class that D-14 does not list
`LoopCloser::atlas()`, `map_replica()` (loop_closer.h:97-102, "never read while the worker runs"), `GlobalBundleAdjuster::atlas()`/`map_replica()` (header lines 60-65), plus plain-`size_t` counters read from any thread while a worker writes them: `LoopCloser::num_dropped_loops_`, `query_stats_`, `last_stats_` and GBA `num_runs_started_`, `num_runs_interrupted_`, `num_results_dropped_` `[VERIFIED: loop_closer.h:176-179, global_bundle_adjuster.h:111-113]`. Same race class as OWN-03; see Open Question 4.

### Pitfall 14: Workspace sizing and perf
A workspace built for one image size must re-initialise on a different size (today: assert in `Build`, UB in Release). The allocating overload costs ~8 image allocations per call: the examples (`DetectAndExtract` at `monocular_full_slam.cpp:97`, `binocular_full_slam.cpp:101,103`) must hold a workspace (two for stereo) or per-frame cost regresses. The voyah_jni session (`TrackFrame`) should hold one under its existing mutex.

### Pitfall 15: Logger traps
(a) `CreateLogger` does `spdlog::get` then `register_logger`; two threads creating the same name race and `register_logger` throws (`throw_if_exists_` `[VERIFIED: registry-inl.h:259-263]`), which in a component constructor becomes an exception; guard the create with a function-static mutex (this is logger setup, not the event path). (b) Tests change global logger state; restore `Logger::Initialize({})` at the end of any test that calls it. (c) The shared sink must be a function-local static (static-init order). (d) The dist sink adds a mutex per log call on top of the existing `_mt` sinks; principle 1 is about the map/event path.

### Pitfall 16: `Image` extra defects found while reading
(a) `CopyTo`'s strided branch copies `width_ * C` **bytes**, not `* sizeof(T)` `[VERIFIED: image.h:116-117 "memcpy(other.GetData() + other.offset_ + y * other.stride_ * C, data_.get() + offset_ + y * stride_ * C, width_ * C);"]`, wrong for `float` images; `<cstring>` is not included. (b) The constructor allocates with unsigned arithmetic `[VERIFIED: image.h:19 "data_(std::shared_ptr<T[]>(new T[width * height * C]))"]`: `width * height * C` wraps for large dimensions. The binding passes caller-supplied `width`/`height` (`slam_session.cpp` builds `images::TWorkingImage image(width, height)` then `memcpy`s `width*height` bytes as `size_t`), so a wrapped allocation followed by a full-size copy is a heap overflow. Compute in `size_t` and throw/assert on overflow. (c) `operator[]` for a 1x1 image offsets by `i` without `* C`. (d) Move operations leave the source non-empty. Fix (a)-(d) with the D-20 changes; each needs a test.

### Pitfall 17: Release plumbing is easy to leave half-done
Version appears in: `CMakeLists.txt:2` (`project(voyah_slam VERSION 0.3.0)`), `conanfile.py:21`, `README.md:126,142,203`, `docs/DESIGN.md` layout line (2804) and a new decisions-log entry. SOVERSION is derived (`major.minor`), `version_major()/minor()` and the `build_config` test come from `project()`. Run `test_package` through `conan create`. Tell voyah_jni the new version and that it is a break.

### Pitfall 18: ThreadSanitizer specifics
On this kernel a TSan binary dies at start without `setarch -R` (see Validation Architecture). GCC 13 warns "`atomic_thread_fence` is not supported with `-fsanitize=thread`" when compiling the moodycamel header; it is a warning, and earlier runs were clean. The scratch tree only builds the targets you ask for.

## Code Examples

All sketches are `[ASSUMED]` designs (names are discretion) except where a `[VERIFIED]` quote or probe is cited.

### The event base and a plain event
```cpp
// libs/events/include/voyahslam/events/event_types.h  (EventType unchanged: see the verbatim enum in Pattern 1)
struct Event {
    EventType type;
    map::MapId map_id{};     // producers set before Notify
    bool priority = false;   // producers set before Notify; immutable afterwards (const)
protected:
    explicit Event(EventType t) : type(t) {}
    ~Event() = default;      // non-virtual and protected: events die through their shared_ptr control block
};
using EventPtr = std::shared_ptr<const Event>;

struct TrackingLost : Event {                        // was: Event + Recyclable + Reset() + two lambdas
    double timestamp = 0.0;
    bool final = true;
    TrackingLost() : Event(EventType::TrackingLost) {}
};
static_assert(!requires(EventPtr e) { e->priority = true; });   // a received event is immutable by type
```

### Inbox core, observer and listener
```cpp
class InboxCore {                                    // shared by AsyncObserver, Listeners and producer entries
public:
    void Push(EventPtr event) {
        if (closed_.load(std::memory_order_acquire)) return;          // observer gone: the copy is dropped here
        if (event->priority) pending_priority_.fetch_add(1, std::memory_order_acq_rel);   // before the enqueue
        queue_.enqueue(std::move(event));
        wake_signal_.Notify();
    }
    bool TryPop(EventPtr& out_event) {
        if (!queue_.try_dequeue(out_event)) return false;
        if (out_event->priority) pending_priority_.fetch_sub(1, std::memory_order_acq_rel);
        return true;
    }
    void Close() {                                   // ~AsyncObserver only
        closed_.store(true, std::memory_order_release);
        EventPtr dropped;
        while (TryPop(dropped)) dropped.reset();     // events released on this thread (D-08)
    }
    WakeSignal& wake_signal() { return wake_signal_; }
private:
    moodycamel::ConcurrentQueue<EventPtr> queue_;
    std::atomic<uint32_t> pending_priority_{0};
    std::atomic<bool> closed_{false};
    WakeSignal wake_signal_;
};

class Listener {                                     // a handle: copyable, no current_, no destructor work
public:
    explicit Listener(std::shared_ptr<InboxCore> core) : core_(std::move(core)) {}
    EventPtr GetNextEvent() { EventPtr event; core_->TryPop(event); return event; }   // empty: none queued
private:
    std::shared_ptr<InboxCore> core_;
};
```

### Wiring and Subscription
```cpp
struct Wiring {                                      // held by Observable and by every Subscription
    std::atomic<RunState> state{RunState::Created};
    uint64_t next_id = 1;
    std::vector<std::pair<uint64_t, std::shared_ptr<InboxCore>>> async;     // written only when not Running
    struct Sync { uint64_t id; void (*fn)(void*, const Event&); void* ctx; };
    std::vector<Sync> sync;
};

class [[nodiscard]] Subscription {
public:
    Subscription() = default;
    Subscription(Subscription&&) noexcept;           // move-only
    ~Subscription() {                                // D-07: destructors never throw
        if (!wiring_) return;
        if (IsRunning(wiring_->state.load(std::memory_order_acquire))) {
            LogMisuse("Subscription destroyed while its producer runs");
            std::terminate();
        }
        wiring_->Remove(id_);                        // safe if the producer is gone: only the shared block is edited
    }
    void reset() {                                   // D-07: explicit disconnect while running throws
        if (!wiring_) return;
        if (IsRunning(wiring_->state.load(std::memory_order_acquire))) throw UsageError("disconnect while the producer runs");
        wiring_->Remove(id_);
        wiring_.reset();
    }
private:
    std::shared_ptr<Wiring> wiring_;
    uint64_t id_ = 0;
};
```

### Publishing and consuming
```cpp
// Producer (Tracker, was: auto* event = frame_tracked_pool_.Create(); ... observable_.Notify(event);)
auto event = std::make_shared<events::FrameTracked<TFeatureExtractor>>();
event->map_id = current_map_id();
event->priority = event->urgent = options_.tracking.IsUrgent(pose.num_inliers);
// ... fill the payload ...
observable_.Notify(std::move(event));                // rvalue only; the producer must not touch the event afterwards

// Consumer (Tracker::ProcessPending, was: while (events::Event* event = inbox_listener_.GetNextEvent()))
while (events::EventPtr event = inbox_listener_.GetNextEvent()) {
    processed = true;
    ApplyEvent(*event);                              // const Event&; the event is released at the end of the iteration
}

// Mapper batch (was: std::vector<events::Event*> batch_ and batch_[i]->recycle_(batch_[i]))
events::EventPtr event;
while (inbox_.TryPop(event)) batch_.push_back(std::move(event));
// ... after ApplyEvent(*batch_[i]):
batch_[i].reset();                                   // the acknowledgement
```

### Destruction-order test (R1 acceptance)
```cpp
// Source: [ASSUMED] test shape for OWN-01 / OWN-02. Parametrise over all 24 permutations (std::next_permutation) of
// {producer, subscription, observer, listener}; a test-only event counts live instances.
struct Probe : Event { Probe() : Event(EventType::TrackingLost) { ++alive; } ~Probe() { --alive; } static inline std::atomic<int> alive{0}; };
struct TestNode : EventNode { void Publish(std::shared_ptr<Probe> e) { observable_.Notify(std::move(e)); } };

TEST_P(DestructionOrderTest, AnyOrderIsSafe) {
    auto producer = std::make_unique<TestNode>();               // an EventNode: it has the RunState D-05 needs
    auto observer = std::make_unique<AsyncObserver>();
    auto listener = std::make_unique<Listener>(observer->GetListener());
    auto subscription = std::make_unique<Subscription>(producer->AddObserver(*observer));
    producer->Publish(std::make_shared<Probe>());                // queued in the observer
    EventPtr held = listener->GetNextEvent();                    // held by "the consumer", outlives everything
    producer->Publish(std::make_shared<Probe>());
    for (int part : GetParam()) {                                // e.g. {2, 0, 3, 1}
        switch (part) { case 0: producer.reset(); break; case 1: subscription.reset(); break;
                        case 2: observer.reset(); break;  case 3: listener.reset(); break; }
    }
    ASSERT_NE(held, nullptr);
    EXPECT_EQ(held->type, EventType::TrackingLost);              // valid after every order (ASan)
    held.reset();
    EXPECT_EQ(Probe::alive.load(), 0);                           // every event freed exactly once
}
```

### Immutability, last drop on another thread, and the sanitizer commands
```cpp
// Last drop on a consumer thread: the destructor runs there, once.
TEST(EventLifetimeTest, LastDropFreesOnTheConsumerThread) {
    std::thread::id freed_on;
    struct Tagged : Event { std::thread::id* out; explicit Tagged(std::thread::id* o) : Event(EventType::TrackingLost), out(o) {}
                            ~Tagged() { *out = std::this_thread::get_id(); } };
    EventPtr event = std::make_shared<Tagged>(&freed_on);
    std::thread consumer([e = std::move(event)]() mutable { e.reset(); });
    const auto consumer_id = consumer.get_id();
    consumer.join();
    EXPECT_EQ(freed_on, consumer_id);
}
```
```bash
# ASan (events tests only; g2o-using tests abort under ASan, TODO PLAT-01)
cmake --build cmake-build-asan --target voyah_events_tests -j8
cmake-build-asan/libs/events/tests/voyah_events_tests
```

### TSan run (commands; the events binary was run on 2026-10-12)
```bash
cmake --build cmake-build-tsan --target voyah_events_tests voyah_slam_tests voyah_integration_tests voyah_common_tests -j8
setarch -R cmake-build-tsan/libs/events/tests/voyah_events_tests
setarch -R cmake-build-tsan/libs/slam/tests/voyah_slam_tests --gtest_filter='Tracker*:Mapper*:Slam*:GlobalBundle*:LoopCloser*:MapFollower*'
setarch -R cmake-build-tsan/libs/slam/tests/voyah_integration_tests
```

The TSan scratch tree has no `vizualization/` directory `[VERIFIED: ls cmake-build-tsan this session: CMakeCache.txt CMakeFiles cmake_install.cmake CTestTestfile.cmake libs Makefile]`, so `voyahviz_tests` (viewer, `SceneState`) run in the Debug tree only. The viewer's inbox drain is therefore not covered by TSan unless the viz option is added to the TSan tree `[ASSUMED]` (optional; the inbox itself is the same `AsyncObserver` the events tests exercise).

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Pooled, intrusively ref-counted `Recyclable` events owned by their component; consumers must release before the producer dies | Self-owning `std::shared_ptr<const T>` events; the last drop frees (any thread) | This phase (R1), decided 2026-10-13 | Removes OQ 25, OQ 14, the `global_ba_` ordering invariant, the apps' `StopAll` teardown, the pool-size options and overflow counters |
| `Listener` borrows an `Event*` and acknowledges on the next call or destruction | `Listener::GetNextEvent()` returns an owning `EventPtr` | R1 | No `current_`, no move-semantics trap; the old "borrow valid until the next call" rule is gone |
| Raw pointer observer registration, no disconnect, raw `WakeSignal*` | RAII `Subscription`, shared `Wiring`, shared `InboxCore` owning its `WakeSignal`; wiring only while stopped | R1 | The binding no longer needs declaration-order workarounds |
| Fixed-size pools with a drop on exhaustion for `LoopDetected` / `GlobalBAFinished` | No bound (recommended) | R1 | Removes the "dropped GBA result is not retried" defect |
| Comments: "Start/Stop from one controlling thread" | Atomic lifecycle, throws `UsageError` | R3 | First exception policy of the library |
| `mutable` pyramid in a `const` method | Caller-provided `Workspace` | R4 | `const` means thread-safe again |

**Deprecated/outdated:** `ObjectPool` and `Recyclable` (D-02); the `global_ba_` ordering comment in `slam.h:64-69`; DESIGN §6.1 (rewritten in R1) and OQ 14 / OQ 25 text; the 2026-09-30 decisions-log rows "Sync handlers run before async enqueue; sync-only notify skips ref counting" and "Event delivery layer ... carries `Event*`" (superseded: sync handlers still run first, but there is no ref count to skip, no stack events, and the layer carries `EventPtr`); the 2026-09-30 and 2026-10-02 rows about `Recyclable::Recycle()`, the Treiber-stack free list and "pooled objects are default-constructed once and kept alive"; the examples' `StopAll` struct (`monocular_full_slam.cpp:67`, `binocular_full_slam.cpp:71`).

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `UsageError` lives in `voyah_common` as `voyahslam/errors.h` | Responsibility Map | Low: only a name/location (discretion) |
| A2 | Releases are 0.4.0 ... 0.9.0, a minor bump each (D-21 says "minor bump" per release; the exact numbers are not stated) | Pattern 4 | Low; the user picks the numbers |
| A3 (revised) | A shared `InboxCore` held by observer, listeners and producer entries (observer-first is memory-safe, events are dropped after `Close()`) is preferable to a per-observer counter with `std::terminate` in `~AsyncObserver` | Pattern 2, R1-Q1 | Medium: it touches the part of D-06 ("observers held by `shared_ptr` were declined") and revisits the deferred "shared queue core" idea, only for destruction-order safety (D-05/D-07 are unchanged) |
| A4 (revised) | `AsyncObserver` owns its `WakeSignal` (inside the core) and `SetWakeSignal(WakeSignal*)` is removed | Pattern 2 | Medium: changes the binding's use of `SetWakeSignal` (one pass in voyah_jni); the alternative is `SetWakeSignal(std::shared_ptr<WakeSignal>)` |
| A5 | `MapSnapshot` contents (per-map ids/epoch, point ids + positions, frame ids + `T_cw`, active map) | Pattern 6 | Low: left to planning by D-13 |
| A6 | (withdrawn: `NoSyncPolicy` pools no longer exist) | - | - |
| A7 | Add `-Werror=unused-result` on test/example targets | Pitfall 6 | Low: a build-flag choice |
| A8 | ASan/LSan on the events tests is a useful extra gate in R1 | Standard Stack | Low: optional |
| A9 | `MapFollower` asserts single-thread use in debug | Pattern 6 | Low |
| A10 | TSan configure line (reconstructed from `CMakeCache.txt` values) would recreate the scratch tree | Environment Availability | Low: the existing tree works as is |
| A11 | One `make_shared` per event and freeing on the last holder's thread cost microseconds against a 10-30 ms frame (DESIGN 2026-10-13 row also says "estimate, not measured") | Pitfall 2 | Low: events are under 100 per second in total |
| A12 | Eigen over-aligned members in events are covered by C++17 aligned `new` in `make_shared` | Pitfall 2 | Low: the first ASan/UBSan run would show a misaligned access |
| A13 | Dropping the `LoopDetected` and `GlobalBAFinished` bounds is safe because both producers are paced by the Mapper's own transactions (reasoned from `loop_closer.cpp:105-107` and the GBA trigger; no experiment) | Pattern 1b | Medium: an unforeseen flood would grow memory; mitigation is the in-flight token sketch |
| A14 | `Notify` as an rvalue-only template is acceptable API for the user (it forces `std::move` at 12 production sites and every test site) | Pattern 1 | Low: a plain `EventPtr` parameter is the fallback |
| A15 | A warn-once-per-doubling log (first at 1024 queued) is the right visibility for a stalled observer | Pitfall 3, R1-Q5 | Low: the thresholds are a taste choice |
| A16 | The `Push`/`Close` race leaves at most one harmless orphan event in a closed core | Pitfall 4 | Low |

## Open Questions

Questions R1-Q1..R1-Q5 replace the earlier Open Questions 1-3 (rewritten for the 2026-10-13 design). Questions 4-7 are unchanged (R2/R3).

- **R1-Q1: Shared inbox core, or counter-and-terminate? (new, needs the user)**
   - What we know: D-06 gives `Subscription`; D-07 makes destroying one while its producer runs a terminate. Neither says what happens when the **observer** dies while subscribed. The earlier draft added a per-observer subscription counter and a terminate in `~AsyncObserver`. With self-owning events the pool hazard is gone, and the remaining one is the wiring.
   - What's unclear: whether the user accepts one more shared block (an `InboxCore` per `AsyncObserver`, one allocation per observer, none per event) so that "observer destroyed first" is safe instead of fatal, and an owned `WakeSignal` replacing `SetWakeSignal(WakeSignal*)`. This partly touches the declined "observers held by `shared_ptr`" and the deferred "shared queue core + closed flag"; it does **not** change D-05 (wiring only while stopped) or D-07.
   - Recommendation: accept it (Pattern 2). It meets the audit's principle literally ("shares ownership of everything it points to"), removes a failure rule the user has not seen, and makes the JNI case (any destruction order) safe. If rejected, fall back to the counter + terminate and keep `SetWakeSignal` behind a `Subscription`; only P01 changes.

- **R1-Q2: "`StopAll` guard is gone" versus D-07**
   - What we know: D-04 and Phase 1 criterion 1 remove the guard; D-07 terminates when a `Subscription` dies while its producer runs. Subscriptions are created from `slam.tracker()`, i.e. after `Slam`.
   - What's unclear: whether a subscription holder declared before `slam` (one declaration-order rule left, violated = loud terminate, not a UAF) counts as "the guard is gone", or whether the user wants even that rule gone.
   - Recommendation: holder before `slam` (Pitfall 5). It removes `StopAll`, the viewer-before-slam and trajectory-before-slam rules (Phase 1 criterion 2), and `viewer.Stop()` / `Poll()` are then only output steps. To remove the last rule too, a `Subscription` destroyed while running would have to tombstone its entry instead of erasing it (the deferred lock-free idea, now about 15 lines with the shared core); that reverses D-07, so only on the user's say-so.

- **R1-Q3: D-08 "drain on disconnect" reworded**
   - What we know: the viewer's inbox is wired to the Tracker and the Mapper `[VERIFIED: monocular_full_slam.cpp:78-79 "slam.tracker().AddObserver(viewer.inbox());" / "slam.mapper().AddObserver(viewer.inbox());"]`; queued events carry no producer tag. Events own their memory (D-01), so a queued event is valid until someone drops it.
   - What's unclear: nothing technical. Disconnecting one producer must not drop the other's events.
   - Recommendation: drain and release only in `~AsyncObserver` (this closes TODO "AsyncObserver destruction doesn't acknowledge events"); a disconnect leaves the queue alone. This amends the wording of D-08; confirm.

- **R1-Q4: Drop the `LoopDetected` and `GlobalBAFinished` bounds? (D-02 says "planner's call")**
   - What we know: Pattern 1b. The drop of a GBA result is a recorded defect; Phase 3 criterion 3 and GBA-01 plan a retry.
   - What's unclear: whether the user wants the cross-phase edit (Phase 3 criterion 3 and GBA-01 become moot).
   - Recommendation: drop both bounds and the four related members; edit `.planning/ROADMAP.md`, `REQUIREMENTS.md` and TODO line 573 in the same change; keep the in-flight token sketch as the fallback.

- **R1-Q5: Keep a "consumer is behind" warning?**
   - What we know: 0.3.0 logged pool growth; without pools nothing is logged (Pitfall 3). EVT-01 (Phase 5) owns real back-pressure.
   - Recommendation: log once when an inbox first holds 1024 events and again at each doubling (`size_approx()` in `Push`, one relaxed counter); name the observer if it has a name. Test with a `ringbuffer_sink_mt` (Pattern 8 note). Skip if the user prefers a smaller R1.

4. **D-14 scope**
   - What's unclear: whether `LoopCloser::atlas()/map_replica()`, `GlobalBundleAdjuster::atlas()/map_replica()` and the plain-`size_t` counters (Pitfall 13) are included, and whether "`num_keyframes()`" means only the `MapId` overload (the no-arg atomic is any-thread-safe and used by the examples).
   - Recommendation: include the LoopCloser/GBA replica accessors (same class, same comment); make the racy counters `std::atomic` (cheap, keeps tests unchanged) or private; keep the no-arg `num_keyframes()` public.

5. **Replacement for `PrintAtlas`**: `MapFollower` in the examples (recommended) versus an atomic summary on the Mapper. The follower dogfoods D-13 but costs a replica.

6. **`WorkerThread` restart**: `WorkerThreadTest.RestartAfterStop` exists. Recommend keeping `WorkerThread` a low-level restartable primitive and putting the terminal rule in the component lifecycle (D-10 says components and "WorkerThread users"); update that test only if the user wants `WorkerThread` itself terminal.

7. **`MapViewer` lifecycle**: its docs say "Restart after Stop() is allowed" and its tests exercise it. It is a consumer with its own thread, not a `WorkerThread` user; leave it as is unless the user wants D-10 applied to it. Its `AddObserver(viewer.inbox())` comments change (D-06/D-11).

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| g++ | Build | yes | 13.3.0 | - |
| CMake / CTest | Build, `check`, `check_all` | yes | `~/.local/bin/cmake` | - |
| Conan 2 | `conan create` | yes | `~/.local/bin/conan` | - |
| Debug tree `cmake-build-debug/build/Debug` | `check` | yes | 746 unit / 773 total tests `[VERIFIED: ctest -N]` | - |
| TSan tree `cmake-build-tsan` | OWN gate | yes | built `voyah_events_tests`; slam/integration binaries need `--target` build | reconfigure (A10) |
| ASan tree `cmake-build-asan` | R1 gate for the lifetime tests | yes (tree present; the events tests are the R1 gate, not run this session); g2o-using tests abort (TODO PLAT-01) | - | filter to `voyah_events_tests` |
| NDK r30 (`~/Android/Sdk/ndk/30.0.16248370`) | android-arm64 `conan create` | yes | 30.0.16248370 | - |
| clang | android profile uses NDK clang 21 | host `clang++` absent (fine: NDK's) | - | - |
| Local 0.3.0 packages | baseline / `test_package` | yes | linux static, linux shared, Android | - |
| g2o `@voyah/stable` (Debug/Release, Android) | package builds | yes | in cache | - |
| gdb, valgrind | debugging | yes | - | - |
| Cores / RAM | builds | 22 cores, 30 GB | - | - |

**Missing dependencies with no fallback:** none.
**Missing dependencies with fallback:** none. Note `setarch` is needed for TSan binaries (present).

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | GoogleTest 1.15.0 via `gtest_discover_tests`, labels `unit` / `integration` / `vocabulary` |
| Config file | `cmake/VoyahTests.cmake` (`voyah_add_test_binary`, `check`, `check_all`); per-library `tests/CMakeLists.txt` |
| Quick run command | `cmake --build cmake-build-debug/build/Debug --target check` (746 tests, 7.9 s measured) or one binary, e.g. `cmake-build-debug/build/Debug/libs/events/tests/voyah_events_tests --gtest_filter='Subscription*'` |
| Full suite command | `cmake --build cmake-build-debug/build/Debug --target check_all` (773 tests, about 30 s) |
| Sanitizer gate | TSan commands in "Code Examples"; ASan: `cmake --build cmake-build-asan --target voyah_events_tests -j8` then run the binary (the destruction-order and event-lifetime tests are only meaningful under ASan) |
| Release gate | `conan create . --build=missing -pr profiles/linux-gcc13` (static), `... -o shared=True` (shared), `conan create . --build=missing -pr:h profiles/android-arm64 -pr:b default` `[CITED: README.md:130-131, 174-175]`; `test_package` runs inside `conan create` |

**TSan note (falsification run, 2026-10-12):** `setarch -R cmake-build-tsan/libs/events/tests/voyah_events_tests` -> `34 tests ... PASSED`; the same binary without `setarch -R` -> `FATAL: ThreadSanitizer: unexpected memory mapping 0x5d9d542d4000-0x5d9d542ea000`. So every TSan command needs `setarch -R`.

### Phase Requirements -> Test Map
Test names are proposed; files marked "Wave 0" do not exist yet.

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| OWN-01 | A received event is immutable by type and `Notify` rejects an lvalue | compile-time | build of `voyah_events_tests` (`static_assert(!requires(EventPtr e) { e->priority = true; })`) | Wave 0 (in `event_lifetime_test.cpp`) |
| OWN-01 | An event held by a consumer outlives its producer (`EventNode` destroyed first) and the observer: valid (ASan), destroyed exactly once, on the thread of the last drop | unit | `voyah_events_tests --gtest_filter='EventLifetimeTest.*'` | Wave 0 `libs/events/tests/events/event_lifetime_test.cpp` |
| OWN-01 | Fan-out of 50 000 events to two observers on two threads frees every event exactly once; a queue destroyed with events inside releases them; the measurement is released by the last drop | unit under TSan | same filter (rewrites of `CrossThreadFanOutReturnsEveryEventToPool`, `RecycledEventsReleaseTheirMeasurement`) | rewrite in `event_system_test.cpp` |
| OWN-01 | Slam destroyed while a stuck observer still holds every event: events stay valid and readable, nothing is dropped, point ids stay contiguous | integration | `voyah_integration_tests --gtest_filter='SlamTest.EventsOutliveSlam*:SlamTest.StuckObserver*'` | rewrite of the two `StuckObserver...` tests in `slam_test.cpp` (no overflow counters) |
| OWN-01 | A loop / a GBA result is delivered while an earlier one is still held (the bound is gone) | unit | `voyah_slam_tests --gtest_filter='LoopCloserTest.*Held*:GlobalBundleAdjusterTest.*Held*'` | rewrite of `loop_closer_test.cpp:749-763`, `global_bundle_adjuster_test.cpp:510-519` |
| OWN-01 | No `ObjectPool`, `Recyclable`, `recycle_`, `StopAll`, `event_pool_size`, `transaction_pool_size`, `pool_overflow`, `max_events_in_flight`, `num_dropped_loops`, `num_results_dropped`, and no "Safe because the Mapper recycles" comment left | review | `grep -rnE 'ObjectPool\|Recyclable\|recycle_\|StopAll\|pool_overflow\|event_pool_size\|transaction_pool_size\|max_events_in_flight\|num_dropped_loops\|num_results_dropped' libs examples vizualization test_package` returns nothing | n/a |
| OWN-02 | All 24 destruction orders of {producer, subscription, observer, listener} are safe; every event is freed exactly once (live counter) | unit under ASan and TSan | `voyah_events_tests --gtest_filter='DestructionOrderTest/*'` | Wave 0 `libs/events/tests/events/subscription_test.cpp` |
| OWN-02 | An observer destroyed before the producer is never notified again: later `Notify`s drop the event at once (live counter back to 0); `~AsyncObserver` releases queued events and returns `pending_priority` to 0; a `Listener` outliving its observer returns empty | unit | `--gtest_filter='SubscriptionTest.ObserverDestroyed*:AsyncObserverTest.*Drain*'` | Wave 0 / extend `event_system_test.cpp` |
| OWN-02 | `AddObserver` and `reset()` while an `EventNode` is Running throw `UsageError`; destroying a `Subscription` of a running producer terminates | unit, `*DeathTest` | `--gtest_filter='SubscriptionDeathTest.*:SubscriptionTest.*Running*'` (death tests excluded from TSan) | Wave 0 |
| OWN-02 | `Push` notifies the observer's own `WakeSignal` and a consumer thread wakes on `observer.wake_signal()`; a push nobody waits on still enqueues | unit | `--gtest_filter='AsyncObserverTest.Push*'` (rewrite of `wake_signal_test.cpp` tests) | rewrite |
| OWN-02 | `Slam` destroyed while running stops first; an exception unwinding past a subscription holder declared before `slam` does not terminate; the examples contain no `StopAll` | integration + build | `voyah_integration_tests --gtest_filter='SlamTest.*Destroy*:SlamTest.*Unwind*'`; both example apps build and run a short `--max_frames` smoke `[ASSUMED]` flag name | Wave 0 in `slam_test.cpp`; existing apps |
| OWN-03 | The listed accessors are not callable from tests/apps except through `*TestAccess` | compile-time | build only (no public accessor left); review grep | n/a |
| OWN-03 | `MapFollower` applies the Mapper's transactions on the caller's thread and equals the Mapper's map; `Snapshot()` is immutable and usable from another thread | unit + TSan | `voyah_slam_tests --gtest_filter='MapFollowerTest.*'` | Wave 0 `libs/slam/tests/mapping/map_follower_test.cpp` (+ CMake entry) |
| OWN-04 | `Start` twice, `Start` after `Stop`, `AddObserver` while running, `Track` after `Stop` throw; `Track` before `Start` queues; `Stop` idempotent; `Start`/`Stop` from different threads | unit | `--gtest_filter='LifecycleTest.*:SlamTest.*Lifecycle*'` | Wave 0 `libs/events/tests/events/lifecycle_test.cpp` + `slam_test.cpp` |
| OWN-04 | Concurrent `Stop()` from N threads and destructor: none returns before the worker is joined | unit under TSan | TSan command above, `LifecycleTest.ConcurrentStop*` | Wave 0 |
| OWN-05 | Two threads, one shared extractor, one workspace each: results equal the single-thread result; golden digests unchanged | unit under TSan | `voyah_algs_tests --gtest_filter='*OrbExtractor*:*Workspace*'` | Wave 0 + existing `orb_extractor_golden_test.cpp` |
| OWN-05 | Workspace re-initialises on a different image size | unit | same filter | Wave 0 |
| OWN-06 | A component constructed before `Logger::Initialize` logs to the new sinks; `Initialize` twice; concurrent create + log + Initialize | unit | `voyah_common_tests --gtest_filter='LoggerTest.*'` | Wave 0 `libs/common/tests/logging/logger_test.cpp` (+ CMake entry) |
| OWN-07 | Default `Image` is empty (0 width/height/stride/offset, null data); `Convert`, `operator*=`, `operator*` on a sub-image view touch only the view; `Set` non-const; `CopyTo` of float views; overflow guarded | unit | `voyah_common_tests --gtest_filter='ImageTest.*'` | Wave 0 `libs/common/tests/images/image_test.cpp` (+ CMake entry) |
| All | DESIGN decisions-log entry per item, `check_all` green, TSan clean, packages recreated, voyah_jni told | process | `check_all`; TSan commands; three `conan create`; message to the user | n/a |

### Sampling Rate
- **Per task commit:** the touched test binary's filtered run (seconds), then `--target check` (about 8 s plus build).
- **Per wave merge:** `--target check_all` (about 30 s).
- **Phase gate (each release):** `check_all` green; TSan on `voyah_events_tests`, `voyah_slam_tests` (Tracker/Mapper/Slam/GlobalBundle/LoopCloser/MapFollower filters) and `voyah_integration_tests`; three `conan create`; the user's explicit "merge and push".
- **Regression guard (PROJECT.md success metric):** ATE on TUM-VI corridor must stay no worse than the recorded baselines; this phase changes no algorithm, so it is a smoke at most (a `--max-frames` run of the apps with and without the change `[ASSUMED]`).

### Wave 0 Gaps
- [ ] `libs/events/tests/events/subscription_test.cpp` - OWN-02: `DestructionOrderTest` (24 permutations), `SubscriptionTest`, `SubscriptionDeathTest` (add to `libs/events/tests/CMakeLists.txt`)
- [ ] `libs/events/tests/events/event_lifetime_test.cpp` - OWN-01: outlive-producer, last drop on another thread, queue destroyed with events, immutability `static_assert`
- [ ] A test-only event with a live-instance counter (`Probe`) in a small shared test header, since there is no pool to count slots of
- [ ] Fixture support: a `std::vector<events::Subscription>` member declared before the component under test in `tracker_test_fixture.h`, `mapper_test.cpp`, `loop_closer_test.cpp`, `global_bundle_adjuster_test.cpp`, `slam_test.cpp`, `map_viewer_test.cpp`, `projection_matcher_test.cpp`
- [ ] `libs/events/tests/events/lifecycle_test.cpp` - OWN-04 (R3)
- [ ] `libs/common/tests/images/image_test.cpp` - OWN-07 (R6, add to `libs/common/tests/CMakeLists.txt`)
- [ ] `libs/common/tests/logging/logger_test.cpp` - OWN-06 (R5; reuse the `ringbuffer_sink_mt` capture pattern from the deleted pool test, Pattern 8 note)
- [ ] `libs/slam/tests/mapping/map_follower_test.cpp` and a shared `*TestAccess` header - OWN-03 (R2)
- [ ] A workspace/concurrency test in `libs/algs/tests/features/` - OWN-05 (R4)
- [ ] Death-test convention (`threadsafe` style, `*DeathTest` names, excluded from TSan)
- [ ] Framework install: none (GTest is a Conan test requirement)

## Security Domain

`security_enforcement` is not set to `false` (no `.planning/config.json`), so it applies. The library has no network, auth or crypto; the relevant surface is memory safety at the JNI boundary.

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | no | - |
| V3 Session Management | no (the "session" is a SLAM run, not a user session) | - |
| V4 Access Control | no | - |
| V5 Input Validation | yes | Dimension/size checks in `Image` (Pitfall 16b); `UsageError` for invalid call sequences |
| V6 Cryptography | no | - |
| V14 / memory-safety and concurrency hygiene (analogous control) | yes | Self-owning `shared_ptr<const T>` events (no UAF), shared `Wiring` and `InboxCore`, TSan/ASan gates |

### Known Threat Patterns for this stack

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Use-after-free of an event whose producer or pool was destroyed | Tampering / DoS | Events own their memory (`make_shared`); structurally impossible, ASan test with an event outliving everything |
| Use-after-free via observer destroyed before producer (and the reverse) | Tampering / DoS | Shared `Wiring` + shared `InboxCore` (Pattern 2), 24-permutation test |
| Data race from a producer touching a published event | Tampering | `const` after publish, rvalue-only `Notify` |
| Unbounded memory from a stalled observer (was: pool growth, logged) | DoS | Warn-once-per-doubling log (R1-Q5); real back-pressure is EVT-01 |
| Data race on map data from a wrong-thread accessor call | Tampering | Private accessors + `*TestAccess` (D-14), `MapFollower` (R2) |
| Heap overflow from `Image(width, height)` unsigned wrap with caller-supplied dimensions | Tampering / EoP | Compute the size in `size_t`, reject overflow (Pitfall 16b) |
| Deadlock / corruption from concurrent `Start`/`Stop`/self-`Stop` | DoS | Atomic lifecycle with wait, self-stop detection (Pattern 3, Pitfall 9) |
| Exception escaping from a destructor or across the JNI boundary | DoS | Destructors never throw (D-07 terminates instead); binding maps `UsageError` |

## Sources

### Primary (HIGH confidence)
- In-repo files read this session (2026-10-13 refresh): `libs/events/include/voyahslam/events/{event_types,observable,event_node,async_observer,listener,wake_signal,worker_thread,frame_tracked,global_ba_finished,initialization_measurement,loop_detected,tracking_lost,map_transaction,events}.h`, `libs/common/include/voyahslam/memory/{recyclable,sync_policy}.h` and `libs/common/src/memory/object_pool.cpp`, `libs/slam/src/{tracking/tracker.cpp,mapping/mapper.cpp,mapping/mapper_{global_ba,quality,loop,merge}.cpp (Notify and CreateTransaction sites),loop_closing/loop_closer.cpp,global_ba/global_bundle_adjuster.cpp,slam/slam.cpp}`, `libs/slam/include/voyahslam/{slam/slam.h,tracking/tracker.h,mapping/mapper.h,loop_closing/loop_closer.h,loop_closing/common_region.h,global_ba/global_bundle_adjuster.h,global_ba/global_ba.h}` (option, accessor, pool and signature lines), `examples/full_slam/{monocular_full_slam.cpp,binocular_full_slam.cpp,full_slam_common.h}`, `vizualization/{include/voyahviz/map_viewer.h,src/map_viewer.cpp}`, tests `libs/events/tests/events/*`, `libs/slam/tests/{tracking/tracker_test_fixture.h,tracking/tracker_test.cpp,mapping/mapper_test.cpp,integration/slam_test.cpp}`, `libs/common/tests/memory/object_pool_test.cpp` (log-capture pattern), `docs/{DESIGN.md §6,§6.1,§8,§9,TODO.md}`, `.planning/{REQUIREMENTS,ROADMAP}.md`, `CLAUDE.md`, `../voyah_jni/docs/voyah_slam_ownership_audit.md`, `../voyah_jni/native/src/slam_session.{h,cpp}`.
- Earlier session (2026-10-12), unchanged parts: the other files listed in the previous revision of this section (Patterns 3-9, Pitfalls 6-9 and 11-18).
- moodycamel `concurrentqueue.h` 1.0.4 in the local Conan cache (`~ExplicitProducer` destroys undequeued elements) and spdlog 1.17.0 headers (`dist_sink.h`, `registry-inl.h`).
- Commands and probes run this session: `ctest -N` (773 tests; 17 `ObjectPool`; 28 `EventSystemTest|AsyncObserverTest|WakeSignalTest`), `grep` counts quoted in "R1 Change Inventory", a ThreadSanitizer probe (`setarch -R`, 50 000 `shared_ptr<const Event>` through two moodycamel queues to two consumer threads, plus a queue destroyed with 100 events: 50 100 of 50 100 destroyed, no TSan report), a compile probe for the rvalue-only `Notify` template.

### Secondary (MEDIUM confidence)
- README.md lines 126-131, 174-175 for the `conan create` commands (cited, not re-run).

### Tertiary (LOW confidence)
- None. Context7 and web lookups were not needed: no external library behaviour beyond locally readable headers and a local probe is relied on. The `research-plan` seam was therefore not used.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - nothing new; pins re-read from `conanfile.py`; the shared_ptr/moodycamel hand-off was run under TSan.
- Architecture: MEDIUM - code facts are verified; the inbox core, owned wake signal, bound removal and `MapSnapshot` shape are recommendations inside the user's discretion or filling gaps (R1-Q1..R1-Q5, Open Questions 4-7).
- Pitfalls: HIGH - each was found in code read this session or a command run; Pitfall 2's cost estimate is `[ASSUMED]`.

**Research date:** 2026-10-12 (R1 refreshed 2026-10-13)
**Valid until:** 2026-11-12 (stable; invalidated earlier if another session changes `libs/events`, the event headers or the components' publish sites before R1 starts)

## Project Constraints (from CLAUDE.md)

- Read `docs/DESIGN.md` first; **record every design decision in the decisions log (and the relevant section) in the same change**; add deferred work to `docs/TODO.md`; ordered next steps are in `docs/ROADMAP.md` (keep `.planning/ROADMAP.md` in sync per its sync rule).
- No locks on map data; threads communicate only via events, each thread owns a map replica; the Mapper is the only map writer; Tracker and LoopCloser never mutate map objects.
- Frame ids are never reused in a run; map point ids are append-only per map; `(map_id, id)` identifies a point.
- Google style naming: `PascalCase` types/functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- Layout: four libraries under `libs/<lib>/{include,src,tests}`; `voyah_events` and `voyah_algs` depend on common only; `voyah_slam` on all three; a lower layer cannot include an upper one (so `UsageError` and the pool live in common, `Subscription` in events).
- Build/test: `cmake --build cmake-build-debug/build/Debug --target check` (746 unit tests) and `--target check_all` (773) at the end of a task and before a merge; tests run with `VOYAHSLAM_OPTIMIZE_TESTS=ON` (-O2, asserts on).
- Project memory: say whether the work fixes the reported symptom, with criteria on the symptom (here: "objects die in any order, calls from any thread, cannot crash the library") rather than proxies; merges, pushes and version bumps only on the user's explicit "merge and push".
