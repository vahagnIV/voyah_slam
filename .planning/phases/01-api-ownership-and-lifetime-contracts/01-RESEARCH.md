# Phase 1: API Ownership and Lifetime Contracts - Research

**Researched:** 2026-10-12
**Domain:** C++20 library API hardening (lock-free event/pool lifetime, RAII subscriptions, lifecycle state machine, thread-affinity, logging, image views), for a Java/JNI consumer whose objects die in any order
**Confidence:** HIGH on code facts (every file below was read this session); MEDIUM on the design recommendations marked `[ASSUMED]` (they sit inside "Claude's Discretion" or expose a gap in a locked decision)

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**Pool lifetime (#3, OWN-01)**
- **D-01:** `ObjectPool` becomes a handle to a heap-allocated pool **core** (chunks, free list, counters). The core keeps an **intrusive atomic count**: 1 for the owner plus 1 per slot handed out (one atomic add in `Create`/`TryCreate`, one subtract in `Recycle`; no allocation per event). `~ObjectPool` drops the owner's count; whichever `Recycle` brings it to 0 frees the core. `Recyclable` keeps its raw back-pointer, now to the core, valid by construction. Chosen over a `shared_ptr<PoolCore>` member per event (the same cost, a redundant second refcount, and a self-destruction trap in `Recycle`) and over static/global pools (memory never freed, shared across `Slam` instances, an exit-order problem). - **Reversibility:** costly - the pool and `Recyclable` internals are used by every event type and pool test.
- **D-02:** Returning an event after its producer is destroyed is **legal**. Leak reporting: when the owner leaves with slots still out, log the outstanding count once; expose `num_outstanding()` on the pool; add a test check that every pool reaches 0 outstanding at the end of the Slam / integration tests. The audit's literal "`~ObjectPool` asserts every slot came back" is **not** adopted (it would fire on legitimate late returns).
- **D-03:** When the last event returns after the owner is gone, the pool is freed **on that consumer's thread** (viewer, JNI or GC thread). Record a DESIGN rule: pooled types' destructors must not depend on the destroying thread.
- **D-04:** Afterwards remove the `global_ba_` ordering invariant (`slam.h`, DESIGN) and the apps' `StopAll` teardown guard (`examples/full_slam/*_full_slam.cpp`). This closes OQ 25.

**Subscription API (#2, OWN-02)**
- **D-05:** Connecting and disconnecting are allowed **only while the producer is stopped** (Created or Stopped). Disconnecting while it runs, without locks, was offered and declined.
- **D-06:** `AddObserver` and `AddSyncObserver` return a `[[nodiscard]]`, move-only RAII **`Subscription`**; its destructor or `reset()` disconnects. This covers `ConnectAsync`, the `ConnectSync` `void* ctx`, and the raw `WakeSignal*` of `SetWakeSignal` where it crosses object boundaries. Observers held by `shared_ptr` were declined. - **Reversibility:** one-way - the public wiring API of every component and of the voyah_jni binding changes.
- **D-07:** Misuse while running: an explicit `reset()`/disconnect, or an `AddObserver`, while the producer runs **throws** (the usage error of D-09). A `Subscription` **destroyed** while its producer runs **logs and calls `std::terminate`** in every build (destructors cannot throw; this gives a defined, loud failure instead of a later use-after-free).
- **D-08:** On disconnect, or when an `AsyncObserver` is destroyed, the events still queued are **drained and released** (this closes TODO "AsyncObserver destruction doesn't acknowledge events"); safe even after the producer is gone, because of D-01.

**Misuse errors and lifecycle (#6, OWN-04)**
- **D-09:** Invalid lifecycle calls **throw a `std::logic_error` subclass** (e.g. `voyahslam::UsageError`) in every build; the binding maps it to `IllegalStateException`. This is the library's first exception policy, so record it in DESIGN (decisions log and conventions). It is introduced with the #3+#2 release, because D-07 needs it.
- **D-10:** The state machine is Created -> Running -> Stopped, and **Stopped is terminal**: `Start` after `Stop` throws, `Start` twice throws, and `Stop` is idempotent (destructors call it). A new session means a new `Slam`. Applies to `Slam` and its components (Tracker, Mapper, LoopCloser, GlobalBA, `WorkerThread` users).
- **D-11:** `Start`/`Stop` are safe **from any thread**, serialized by an atomic state CAS (a losing `Start` throws, `Stop` is idempotent). This replaces "must be called from one controlling thread", because a Java `close()` or GC cleaner may stop or destroy from another thread.
- **D-12:** `Track()` **before `Start()` keeps queuing** (today's behaviour); `Track()` **after `Stop()` throws**.

**Map read path and thread-affinity accessors (#5, OWN-03)**
- **D-13:** A library **`MapFollower`** owns an inbox and an `AtlasReplica`. The app subscribes it to the Mapper's `MapTransaction`s, calls `Update()` on its own thread (which applies pending transactions) and reads the replica on that thread. It can also produce an immutable **`std::shared_ptr<const MapSnapshot>`** (points, keyframe poses, map ids; exact contents set at planning) for other threads or for Java. Snapshot cost is O(map) per call, paid only on request.
- **D-14:** The "Mapper thread only" accessors (`Mapper::covisibility()`, `Reference()`, `NumVisible()`, `NumFound()`, `last_local_ba()`, `BowVectorOf()`, `Parent()`, `LoopEdges()`, `KeyFrame()`, `map_ids()`, `active_map()`, `num_keyframes()`) and the "never while running" ones (`Tracker::atlas()`, `map_replica()`, `last_matches()`) become **private behind friend `*TestAccess` structs** (the `MapperTestAccess` pattern), so misuse fails at compile time. Tests and tools move to the access structs.

**Extractor thread safety (#7, OWN-05)**
- **D-15:** The **caller provides the workspace**: `DetectAndExtract(image, n, Workspace&) const`. The extractor becomes immutable and shareable, and the pyramid moves into the workspace (no per-frame allocation when the workspace is reused). Chosen over making the call non-const or using thread-local storage. - **Reversibility:** costly - every caller (apps, tests, benchmarks, voyah_jni) changes.
- **D-16:** A convenience **allocating overload `DetectAndExtract(image, n) const` stays**. It builds a temporary workspace per call, which is fine for tests and one-offs; document that a loop should reuse a workspace.
- **D-17:** The caller owns the workspaces, **one per extraction thread** (stereo: two if left and right are extracted in parallel). The workspace type is documented as not thread-safe.

**Logger (#8, OWN-06)**
- **D-18:** Every logger gets **one shared spdlog `dist_sink_mt`**. `Logger::Initialize()` swaps its child sinks, so creation order stops mattering by construction, and `Initialize` may be called again at any time. `SetLevel` applies to every registered logger, not only to later ones. The cost is one extra indirection per log call.
- **D-19:** The Android logcat default sink stays in Phase 5 (PLAT-01). The binding can already pass spdlog's `android_sink` to `Initialize()`.

**Image (#9, OWN-07)**
- **D-20:** **Fix in place**, with no separate `ImageView` type:
  - default-initialize every member (an empty image);
  - make `Convert()`, `operator*=` and `operator*` row-wise, honouring `offset_` / `stride_`;
  - make `Set()` non-const;
  - document that views alias the parent's buffer.

**Releases**
- **D-21:** **#3 and #2 ship together** as one branch and one minor version bump (they are designed together and both reshape the event API). Then #5, #6, #7, #8 and #9 each get **their own branch, minor bump and release**, as #1 and #4 did. Each release:
  - a review, then a merge only on the user's explicit "merge and push";
  - a DESIGN decisions-log entry and `check_all`;
  - ThreadSanitizer on the event and Slam tests (#3+#2 and #6 at least);
  - `conan create` for linux-gcc13 (static and shared) and android-arm64;
  - the voyah_jni session is told the new version (every item is an API break for it).

### Claude's Discretion
- The internal layout of the pool core (cache-line placement of the counter, how `NoSyncPolicy` pools count), as long as D-01..D-03 hold.
- The exact `Subscription` type (one type for sync and async, or two) and how components track the "stopped" state used by D-05 (`WorkerThread::running()` exists).
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
| OWN-01 | Pool ownership shared by handed-out events; `global_ba_` invariant and apps' `StopAll` guard removed; release-after-producer-destroyed test | Pool core design (Pattern 1), Pitfalls 1-3, existing tests that must change (Pitfall 3), Validation rows OWN-01 |
| OWN-02 | RAII `Subscription` for `ConnectAsync`/`ConnectSync`/`SetWakeSignal`; connect/disconnect while running fails loudly; observer destroyed first is never notified | Pattern 2 (shared wiring block), Open Questions 1-3, Pitfalls 4-8 |
| OWN-03 | Thread-affinity accessors enforced; supported map read path | Pattern 5 (TestAccess), Pattern 6 (`MapFollower`), Open Question 4, Pitfalls 11-13 |
| OWN-04 | Lifecycle state machine; invalid transitions error; restart decided | Pattern 3 (atomic lifecycle), Pitfall 9, Open Question 6 |
| OWN-05 | `DetectAndExtract` concurrently safe or impossible to misuse | Pattern 7 (workspace), Pitfall 14 |
| OWN-06 | `Logger::Initialize` retargets earlier loggers; `sinks_` protected | Pattern 8 (`dist_sink_mt`), Pitfall 15 |
| OWN-07 | `Image` default-init, row-wise view ops, non-const `Set`, aliasing documented | Pattern 9, Pitfall 16 (extra findings: `CopyTo`, constructor overflow) |
</phase_requirements>

## Summary

The phase is six releases of API hardening inside one codebase: R1 (#3 pool + #2 subscriptions, 0.4.0 if each release bumps the minor), then #5, #6, #7, #8, #9. The code is small and well-isolated (event system: 800 lines in `libs/events`; pool: 333 lines in one header), but the blast radius is wide: about 70 `AddObserver`/`ConnectAsync` call sites, about 200 test calls to the Mapper-thread accessors, every pool test, both example apps, and the `MapViewer`. The baseline is green: `check` ran 746 tests in 7.9 s on 2026-10-12 `[VERIFIED: ran cmake --build cmake-build-debug/build/Debug --target check]`; CLAUDE.md gives 773 for `check_all`.

Three design gaps in the locked decisions must be resolved before the planner commits to tasks (they are in "Open Questions", with a recommendation each): (1) a `Subscription` must work in **both** destruction orders (producer first is the JNI case), which needs a shared wiring block, not just a pointer into the producer; (2) D-07 (destroying a `Subscription` of a running producer terminates) means the examples cannot drop all teardown logic: a one-line `slam.Stop()` on every exit path remains, so "`StopAll` gone" needs the user's reading of it; (3) D-08 as worded ("on disconnect, drain the queue") would also drop events from *other* producers that share the observer's inbox (the viewer's inbox receives from Tracker and Mapper), so the drain must be at `~AsyncObserver` (and not per disconnect).

Tooling is ready: a ThreadSanitizer tree (`cmake-build-tsan`) and an ASan tree (`cmake-build-asan`) already exist; TSan needs `setarch -R` on this kernel `[VERIFIED: ran the events tests under TSan with and without it, see Validation Architecture]`. Conan, the NDK r30 and the 0.3.0 packages for linux static/shared and android-arm64 are in the local cache, so the release step is repeatable.

**Primary recommendation:** Build R1 bottom-up in this order: `UsageError` + `Lifecycle` state in `voyah_events` (so #6 only adds throws later), pool core + `Recyclable` retarget, shared `Wiring` block + `Subscription` + `~AsyncObserver` drain, then migrate `Slam`, `MapViewer`, examples and ~70 test sites, and finish with the destruction-order permutation test, `num_outstanding()` leak checks, TSan and the three `conan create` runs; end each release with a human checkpoint for "merge and push".

## Architectural Responsibility Map

The "tiers" here are the repo's libraries and consumers (the dependency rule: a lower layer cannot include an upper one, DESIGN section 7).

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| `UsageError` (exception policy) | `voyah_common` | - | Lowest layer that needs it: the pool/log code and `voyah_events` both throw it. `[ASSUMED]` header name `voyahslam/errors.h` |
| Pool core, `Recyclable`, `num_outstanding()` | `voyah_common` (`memory/`) | - | Already there; pool is used by every event type |
| `Subscription`, shared wiring block, `Lifecycle` state, `AsyncObserver` drain | `voyah_events` | - | `Observable`/`EventNode`/`AsyncObserver` live here and depend on common only |
| Start/Stop state machine of each component, `Track()` check | `voyah_slam` (components) over `voyah_events` `WorkerThread`/`EventNode` | `Slam` (composite state) | Components own the threads; `Slam` composes four of them |
| `MapFollower`, `MapSnapshot` | `voyah_slam` | - | Wraps `AtlasReplica`/`MapReplica` (`voyah_slam`); not in `voyahviz` (deferred) |
| `*TestAccess` friends | `voyah_slam` headers (forward declaration) + `libs/slam/tests` (definition) | - | `MapperTestAccess` pattern; definition lives with the tests |
| Extractor `Workspace` | `voyah_common` (declaration, next to `OrbFeatureExtractor` and `orb/image_pyramid.h`) | `voyah_algs` (definition) | The extractor class is declared in common and defined in algs (DESIGN section 7 "Placement") |
| `Logger` | `voyah_common` | - | `logging/logger.{h,cpp}` |
| `Image` | `voyah_common` (`images/image.h`) | tests currently in `voyah_algs` tests | Header-only template in common; no image test exists in `libs/common/tests` |
| Example wiring / teardown | `examples/full_slam`, `voyahviz` | - | Consumers: they hold `Subscription`s and decide the stop order |

## Standard Stack

This phase adds **no new external package**. Everything is existing code plus APIs of the pinned dependencies.

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| spdlog | 1.17.0 `[VERIFIED: conanfile.py:64 "spdlog/1.17.0"]` | `dist_sink_mt` for the shared sink (D-18) | Part of spdlog; `set_sinks` takes the sink mutex, so it is safe against concurrent `log` `[VERIFIED: dist_sink.h:42-45 "void set_sinks(std::vector<std::shared_ptr<sink>> sinks) { std::lock_guard<Mutex> lock(base_sink<Mutex>::mutex_); sinks_ = std::move(sinks); }"]` |
| concurrentqueue (moodycamel) | 1.0.4 `[VERIFIED: conanfile.py "concurrentqueue/1.0.4"]` | Event queues; `try_dequeue` drains for D-08 | Already the event transport |
| GoogleTest | 1.15.0 `[VERIFIED: conanfile.py "gtest/1.15.0"]` | Tests, incl. death tests for the D-07 terminate | Existing framework |
| Eigen | 3.4.0 | `MapSnapshot` storage (SoA `TPoint3DF` columns) | Existing |

### Supporting
| Tool | Purpose | When to Use |
|------|---------|-------------|
| `cmake-build-tsan` (scratch tree, gitignored) | ThreadSanitizer runs | Every release; event + Slam + integration tests |
| `cmake-build-asan` (scratch tree) | AddressSanitizer + LeakSanitizer on pool/event tests | R1 (pool core freed on a consumer thread; late recycle) `[ASSUMED]` extra gate, not in the success criteria |
| Conan 2 + NDK r30 | `conan create` for linux-gcc13 static/shared and android-arm64 | End of each release |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| Intrusive core count (D-01, locked) | `shared_ptr<PoolCore>` per event | Declined by the user (redundant refcount, self-destruction trap) |
| Shared `Wiring` block for subscriptions (recommended) | Subscription holds a raw `Observable*` | Dangles when the producer dies first (the JNI case); see Open Question 1 |
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
 Producer thread (Tracker/Mapper/LoopCloser/GlobalBA worker)           Consumer threads (viewer, JNI dispatcher, GC cleaner)
 ───────────────────────────────────────────────────────────           ─────────────────────────────────────────────────────
  pool.Create()  ──► core.count++ (1 owner + 1 per slot out)
  fill payload
  Observable::Notify(e)
     ├─ sync handlers (inline)
     └─ for entry in Wiring.entries:  AsyncObserver::Push(e) ───────────►  queue ──► Listener::GetNextEvent()/~Listener
                                                                                          └─► e->Recycle() ► core.Recycle(slot)
                                                                                                 1. push slot on free list
                                                                                                 2. count-- ; if it hit 0 → free core
                                                                                                    (runs on THIS consumer's thread, D-03)

 App thread                                                    Ownership of the wiring
 ──────────                                                    ─────────────────────────
  sub = producer.AddObserver(obs)  (state != Running, else UsageError)      Wiring (shared_ptr, held by Observable AND by each Subscription)
  producer.Start()   Created ─CAS─► Running                                 ├─ std::atomic<RunState> state   (set by the component)
  producer.Stop()    Running ─────► Stopped (terminal), any thread          ├─ entries: vector<{id, AsyncObserver* | fn+ctx}>   (mutated only when !Running)
  sub.reset()        only if state != Running (else throws)                 └─ outlives the producer if a Subscription still exists
  ~Subscription      Running → log + std::terminate (D-07)
  ~AsyncObserver     drain queue: try_dequeue + recycle_ (D-08)

 Map read path (OWN-03):  Mapper ─MapTransaction─► follower.inbox  ──(app thread)──► MapFollower::Update() ──► AtlasReplica ──► Snapshot() → shared_ptr<const MapSnapshot>
```

### Recommended Project Structure
```
libs/common/include/voyahslam/
├── errors.h                      # UsageError : std::logic_error            (new, [ASSUMED] name/location)
├── memory/object_pool.h          # ObjectPool handle + PoolCore             (changed)
├── memory/recyclable.h           # back-pointer type → core                 (changed)
├── features/orb_feature_extractor.h   # + Workspace, workspace overload     (changed)
├── images/image.h                # fixes                                    (changed)
└── logging/logger.h              # (changed)
libs/events/include/voyahslam/events/
├── lifecycle.h                   # RunState + atomic transitions            (new)
├── subscription.h                # Wiring + Subscription                    (new)
├── observable.h / event_node.h / async_observer.h / worker_thread.h        (changed)
libs/slam/include/voyahslam/
├── mapping/map_follower.h, map_snapshot.h   # (new, #5)
└── {tracking,mapping,loop_closing,global_ba,slam}/…                          (changed)
libs/slam/tests/
├── test_access.h (or mapping/mapper_test_access.h)   # shared *TestAccess definitions (new, #5)
└── …
```

### Pattern 1: Pool core with intrusive count (D-01..D-03)
**What:** `ObjectPool<T,P>` holds `PoolCore<T,P>*`; the core is today's pool body plus `std::atomic<uint32_t> refs_` (1 for the owner, +1 per slot out). `Recyclable::object_pool_` becomes a `PoolCore*`.
**Current code to change** `[VERIFIED: object_pool.h:198 "obj->object_pool_ = this;"]`, `[VERIFIED: recyclable.h:24 "ObjectPool<T, SyncPolicy>* object_pool_ = nullptr;"]`, `[VERIFIED: recyclable.h:37 "object_pool_->Recycle(static_cast<T*>(this));"]`.
**Rules that make it correct:**
1. `TryCreate`: after a successful pop, `refs_.fetch_add(1, relaxed)`. `Recycle(obj)`: **push the slot on the free list first**, then `if (refs_.fetch_sub(1, acq_rel) == 1) delete this;`. Pushing first keeps the slot memory valid until the count is dropped; `acq_rel` makes the destroying thread see every other thread's construction writes (`Chunk::constructed` is plain `bool`, read in `~Chunk`).
2. `~ObjectPool` (owner): `prev = refs_.fetch_sub(1, acq_rel)`; if `prev == 1` delete; else log once (logger `ObjectPool`, warn) "N events still out" (D-02). `num_outstanding() = refs_ - 1` while the owner lives.
3. `NoSyncPolicy` pools (single-thread, tests only; all production pools are `ThreadSafePolicy`) may use a plain counter via `if constexpr (SyncPolicy::kThreadSafe)` `[ASSUMED]` (Claude's discretion).
4. The owner can no longer `Create()` after its destructor, so no count can rise from 0.
**Consequence:** `T` destructors (all chunks) run on whichever thread returns the last event. All current pooled types hold only `std::vector`, Eigen, `shared_ptr<const ...>` and `ColumnBuffer`, which are thread-agnostic `[VERIFIED: frame_tracked.h, global_ba_finished.h, map_transaction.h read]`.

### Pattern 2: Shared wiring block + `Subscription` (D-05..D-08)
**What:** every `EventNode` creates one `std::shared_ptr<Wiring>` at construction (one allocation per component, none per event). `Observable::Notify` reads `wiring_.get()->entries` through a raw cached pointer (no refcount traffic on the hot path). `Subscription` holds `shared_ptr<Wiring>` + an entry id, so it is valid after the producer is gone; disconnecting after the producer died only edits the (shared) block.
**Why not a raw `Observable*` in the Subscription:** the JNI case closes `Slam` first and the listener later; a raw pointer would dangle. `[VERIFIED: voyah_jni slam_session.h comment "inbox_ is declared before slam_ and outlives it"]` shows the binding's current workaround for exactly this.
**Current code** `[VERIFIED: observable.h:15 "async_observers_.push_back(&observer);"]`, `[VERIFIED: observable.h:19 "sync_handlers_.push_back({fn, ctx});"]`, `[VERIFIED: event_node.h:24 "void AddObserver(AsyncObserver& observer) { observable_.ConnectAsync(observer); }"]`.
**Mutation rule (lock-free, TSan-clean):** entries are written only when `state != Running`; `Start()` publishes `Running` with a release store before the thread is created (thread creation is a happens-before edge); `Stop()`'s join gives the reverse edge for later disconnects. No mutex.
**Observer-destroyed-first:** keep a per-`AsyncObserver` atomic count of live Subscriptions (incremented in `AddObserver`, decremented in `Subscription` release); `~AsyncObserver` with a non-zero count logs and terminates (same policy as D-07) `[ASSUMED]`. See Open Question 1.
**`SetWakeSignal`:** the EventNode's inbox points at the node's own `wake_signal_` `[VERIFIED: event_node.h:19 "EventNode() { inbox_.SetWakeSignal(&wake_signal_); }"]` (member order `observable_`, `inbox_`, then private `wake_signal_` means `wake_signal_` is destroyed *first*, a latent hazard). For signals that cross objects (the binding's own `WakeSignal`) the cleanest fix is for `AsyncObserver` to **own** its `WakeSignal` and expose `wake_signal()`; the consumer waits on it `[ASSUMED]` (discretion). `SetWakeSignal(nullptr)` behaviour is tested (`AsyncObserverTest.PushWithoutSignalStillEnqueues`), so keep an equivalent test.

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
**Do:** a function-local `static std::shared_ptr<spdlog::sinks::dist_sink_mt>` (not a namespace-scope static: `CLASS_LOGGER` runs in component constructors, possibly during static init); `CreateLogger` builds every logger with `{dist_sink}`; `Initialize(sinks)` calls `dist_sink->set_sinks(sinks)` (empty vector keeps today's default stdout sink). Tests that capture log lines add a sink to a logger directly (`logger->sinks().push_back(sink)` in `object_pool_test.cpp:319-320`); this keeps working because the logger's own sink vector now holds the dist sink and the test appends a second one.
**Already true, do not re-implement:** `Logger::SetLevel` already applies to existing loggers, because `spdlog::set_level` loops over the registry `[VERIFIED: spdlog registry-inl.h:149-155 "for (auto &l : loggers_) { l.second->set_level(log_level); }"]`. What is still racy is the plain static `level_` (read in `CreateLogger`, written in `SetLevel`): make it `std::atomic`.

### Pattern 9: `Image` fixes (D-20)
**Current** `[VERIFIED: image.h:15 "Image() = default;"]` with members `[VERIFIED: image.h:128 "size_t offset_;"]` followed by `width_`, `height_`, `stride_` (uninitialised), `[VERIFIED: image.h:50 "void Set(size_t y, size_t x, size_t c, const T &value) const {"]`, `[VERIFIED: image.h:59 "for (int i = 0; i < width_ * height_; ++i) {"]` in `Convert()` (no `* C`, no offset/stride).
**Do:** brace-initialise every member (`offset_{0}, width_{0}, height_{0}, stride_{0}`), also reset the moved-from image in the move constructor/assignment (they leave `width_`/`stride_` set on a null buffer); implement `Convert`, `operator*=`, `operator*` as `for y in rows, for x in width*C` starting at `offset_ + y*stride_*C`; `Set` non-const; document that a view `(other, x, y, w, h)` shares the parent's buffer (`shared_ptr<T[]>`).
**No callers exist** (`Convert<`, `.Convert`, `operator*` have zero uses outside `image.h` and no test), so every changed operation needs a new test on a sub-image view; there is no `libs/common/tests/images` directory, so add `images/image_test.cpp` to `libs/common/tests/CMakeLists.txt`.

### Anti-Patterns to Avoid
- **Raw pointer from a `Subscription` into the producer:** dangles when the producer dies first (Pattern 2).
- **`std::shared_ptr` per event for the pool:** declined (D-01) and adds a second refcount.
- **A `std::mutex` on the event path** (principle 1). A control-plane atomic with `wait` is fine.
- **Asserting in `~ObjectPool` that all slots came back:** declined (D-02); it would fire on legitimate late returns.
- **Draining the whole inbox on a single disconnect** (Open Question 3).

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Fan-out to swappable log sinks | Own sink list + lock | `spdlog::sinks::dist_sink_mt::set_sinks` | Already mutex-protected against concurrent `log`; D-18 |
| Sleeping a losing `Start`/`Stop` | Spin loop | `std::atomic::wait/notify` (C++20) | Already how `WakeSignal` works `[VERIFIED: wake_signal.h:21-27]` |
| Cross-thread immutable map data | Copy-on-write scheme | `std::shared_ptr<const MapSnapshot>` | The project's pattern for shared immutable data (vocabulary, `TSharedMeasurement`, `TDistortionModelPtr`) |
| Thread identity check (follower, self-stop) | Custom TLS | `std::this_thread::get_id()` | Sufficient for debug asserts and the self-`Stop` guard |
| Queue drain | Own queue walk | `moodycamel::ConcurrentQueue::try_dequeue` (as `AsyncObserver::TryPop` does) | Keeps the priority counter right |
| Leak detection | Own allocator tracking | `num_outstanding()` on the core + LSan/ASan trees | One atomic already exists (the core count) |

**Key insight:** the only new concurrency primitives are the pool-core count and a lifecycle atomic; everything else is composition of existing, TSan-verified pieces (the project's TSan runs were clean on the event, Mapper and Slam tests for each prior feature, DESIGN lines 2480, 2688, 3414).

## Runtime State Inventory

Not a rename/refactor/migration phase in the sense of stored strings. Explicitly: **Stored data:** none (no persisted data uses these types). **Live service config:** none. **OS-registered state:** none. **Secrets/env vars:** none. **Build artifacts:** the `cmake-build-*` trees and `~/.conan2` packages of 0.3.0 become stale per release; each release needs a fresh `conan create` and the voyah_jni session pinned to the new version (`voyah_jni/conanfile.py` requires an older version string per the audit). The one cross-repo coupling is the binding: `[VERIFIED: ../voyah_jni/native/src/slam_session.cpp]` still passes `distortion_.get()` (a raw pointer, the pre-0.2.0 API), so that repo is already behind and every item here is a further break for it.

## Common Pitfalls

### Pitfall 1: Free-list push must precede the count decrement
**What goes wrong:** decrementing first lets another thread free the core while this thread pushes into its free list (use-after-free).
**How to avoid:** `PushChain` then `fetch_sub(1, acq_rel)` (Pattern 1). **Warning signs:** TSan/ASan report in `PushChain` after `~ObjectPool`.

### Pitfall 2: Core freed on the consumer's thread
**What goes wrong:** `T` destructors run on the viewer/JNI/GC thread; a destructor that touches thread-affine state breaks. **How to avoid:** DESIGN rule (D-03); add a unit test that returns the last event from a different thread and checks the destroyed count. Also: `LogPoolGrowth` and the new "outstanding" log use a function-static spdlog logger (`object_pool.cpp:9`), which is fine, but at process exit a late return after static destruction must not log (do not log from the late-free path).

### Pitfall 3: Existing tests that deliberately leave events outstanding
**What goes wrong:** `ObjectPoolTest.DestructorDestroysEveryConstructedObjectOnce` ends its scope with two slots still out (`(void)pool.Create(); (void)c;`) and expects `g_destroyed == 3` `[VERIFIED: object_pool_test.cpp:221-236]`; with D-01 the core (and its objects) stays alive, so it fails (and leaks under LSan). Many `event_system_test.cpp` tests do `pool.Create()` without recycling. **How to avoid:** recycle everything in those tests and add the new positive tests (destroy owner with slots out, then return them; objects destroyed exactly once, on the last return). **Warning signs:** `check` red in `ObjectPoolTest` / `EventSystemTest` after the core lands.

### Pitfall 4: Subscription in both destruction orders
See Pattern 2 and Open Question 1. A test that destroys {Subscription, observer, producer} in all six orders catches it.

### Pitfall 5: D-07 turns the examples' exception-unwinding path into a terminate
**What goes wrong:** a `Subscription` declared after `Slam` is destroyed first on unwinding while `Slam` still runs: log + `std::terminate`. Declaring it before `Slam` is impossible (it is created from `slam.tracker()`). **How to avoid:** the pool/viewer-drain half of `StopAll` (`viewer.Stop(); trajectory->Poll();` after `slam.Stop()`) goes away, but a plain `slam.Stop()` on every exit path (try/catch or a tiny scope guard declared after the Subscriptions) stays. See Open Question 2; this needs the user's reading of "StopAll guard is gone".

### Pitfall 6: `[[nodiscard]]` is only a warning
There is no `-Werror` and no warning flags in the build `[VERIFIED: grep for -Wall/-Werror in CMakeLists.txt, libs/*/CMakeLists.txt, profiles, cmake, conanfile.py found none]`. A discarded `Subscription` compiles and disconnects immediately. **How to avoid:** put `[[nodiscard]]` on the class itself, add `-Werror=unused-result` to the test and example targets `[ASSUMED]`, and give tests a `std::vector<Subscription>` helper. `Slam` itself must store the six internal Subscriptions (slam.cpp:30-43 wiring) in a member declared **after** the components, so they are destroyed (stopped producers) before them.

### Pitfall 7: Death tests under TSan and threads
The D-07 terminate needs `EXPECT_DEATH`; there are none in the repo today `[VERIFIED: grep for EXPECT_DEATH/EXPECT_EXIT found none]`. Use `death_test_style = "threadsafe"` and name them `*DeathTest` so they can be excluded from TSan runs (fork with threads is unreliable under TSan).

### Pitfall 8: Disconnect and `Start` racing
`Subscription::reset()` on thread A while `Start()` runs on thread B must not corrupt the entry vector. With the `Starting` state in Pattern 3, disconnect treats `Starting` as "running" (throws) and `Start` treats an in-progress wiring edit as a usage error; both are detected by CAS rather than made safe.

### Pitfall 9: Start/Stop races (D-11)
(a) `thread_` is assigned in `Start` while `Stop` reads `joinable()`: serialize through the state. (b) The second of two concurrent `Stop()` calls must **wait** for the first to finish, otherwise a destructor returns while the worker runs. (c) `Stop()` from the component's own thread (a sync observer handler or a test hook, e.g. `global_ba.set_publish_hook([&]{ global_ba.Stop(); })` at `global_bundle_adjuster_test.cpp:497`) would self-join: detect `this_thread::get_id() == thread_.get_id()` and throw `UsageError` (that test drives `ProcessPending()` synchronously, so check it still passes). (d) `GlobalBundleAdjuster::stop_requested()` is documented "True from Stop() until the next Start()" `[VERIFIED: global_bundle_adjuster.h:76]`; with a terminal Stopped it becomes "true once Stopped"; update the comment and its tests. (e) `WorkerThreadTest.RestartAfterStop` `[VERIFIED: worker_thread_test.cpp:111]` encodes restart; decide whether `WorkerThread` stays restartable (the terminal rule lives in the component/EventNode) or becomes terminal too (Open Question 6).

### Pitfall 10: Tests that wire and count
`slam_test.cpp:60-70` wires an `AsyncObserver visualizer` and asserts `async_observer_count()`; with RAII the Subscription must be kept alive for the count to hold. Counts of `AddObserver`-family call sites: examples 6, `event_system_test` 10, `wake_signal_test` 3, `slam.cpp` 6, `projection_matcher_test` 2, `global_bundle_adjuster_test` 2, `slam_test` 9, `loop_closer_test` 3, `mapper_test` 10, `tracker_test` 2, `tracker_test_fixture.h` 4, `map_viewer.h` 1, `map_viewer_test.cpp` 13 `[VERIFIED: grep -c]`.

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

### Pool core release path
```cpp
// Source: derived from the verified object_pool.h structure (this session); sketch, not final.
void Recycle(T* obj) {                      // PoolCore<T,P>::Recycle
    PushChain(IndexOf(obj), IndexOf(obj));  // 1. slot back on the free list
    if (refs_.fetch_sub(1, std::memory_order_acq_rel) == 1) delete this;  // 2. last reference frees the core
}
// ObjectPool<T,P>::~ObjectPool():
//   const uint32_t prev = core_->refs_.fetch_sub(1, std::memory_order_acq_rel);
//   if (prev == 1) delete core_; else /* log once: prev - 1 outstanding */;
```

### Subscription lifecycle (shape only)
```cpp
// [ASSUMED] sketch; names are discretion.
class [[nodiscard]] Subscription {
public:
    Subscription() = default;
    Subscription(Subscription&&) noexcept;               // move-only
    ~Subscription() {                                    // D-07
        if (wiring_ && wiring_->state.load(std::memory_order_acquire) == RunState::Running) {
            LogMisuse("Subscription destroyed while its producer runs");
            std::terminate();
        }
        Disconnect();                                    // safe even if the producer is gone
    }
    void reset();                                        // throws UsageError if Running
private:
    std::shared_ptr<Wiring> wiring_;
    uint64_t entry_id_ = 0;
    AsyncObserver* observer_ = nullptr;                  // for the per-observer subscription count
};
```

### Destruction-order test (R1 acceptance)
```cpp
// Source: [ASSUMED] test shape for OWN-01 / OWN-02. Parametrise over std::next_permutation of {slam, subscription, observer, listener}.
TEST_P(DestructionOrderTest, AnyOrderIsSafe) {
    auto slam = std::make_unique<OrbSlam>();
    auto observer = std::make_unique<events::AsyncObserver>();
    auto listener = std::make_unique<events::Listener>(observer->GetListener());
    auto sub = std::make_unique<events::Subscription>(slam->tracker().AddObserver(*observer));
    // publish events (synchronous ProcessPending, or a started+stopped Slam), then destroy per GetParam()
    // expectation: no UAF/TSan report; every pool reports num_outstanding() == 0 afterwards
}
```

### TSan run (verified commands)
```bash
cmake --build cmake-build-tsan --target voyah_events_tests voyah_slam_tests voyah_integration_tests voyah_common_tests -j8
setarch -R cmake-build-tsan/libs/events/tests/voyah_events_tests
setarch -R cmake-build-tsan/libs/slam/tests/voyah_slam_tests --gtest_filter='Tracker*:Mapper*:Slam*:GlobalBundle*:LoopCloser*:MapFollower*'
setarch -R cmake-build-tsan/libs/slam/tests/voyah_integration_tests
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Event pool owned by the component, consumers must release first | Pool core freed by the last returned event | This phase (R1) | Removes every destruction-order rule from `Slam` users |
| Raw pointer observer registration, no disconnect | RAII `Subscription`, wiring only while stopped | This phase (R1) | Binding no longer needs declaration-order workarounds |
| Comments: "Start/Stop from one controlling thread" | Atomic lifecycle, throws `UsageError` | R3 | First exception policy of the library |
| `mutable` pyramid in a `const` method | Caller-provided `Workspace` | R4 | `const` means thread-safe again |

**Deprecated/outdated:** `ObjectPool` by-value member semantics for events handed out (D-01); the `global_ba_` ordering comment in `slam.h:64-69`; DESIGN OQ 25 text; the examples' `StopAll` struct (`monocular_full_slam.cpp:67`, `binocular_full_slam.cpp:71`); DESIGN section 6 "Rules" statement "Consumers never call `Recycle()`" stays true.

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `UsageError` lives in `voyah_common` as `voyahslam/errors.h` | Responsibility Map | Low: only a name/location (discretion) |
| A2 | Releases are 0.4.0 ... 0.9.0, a minor bump each (D-21 says "minor bump" per release; the exact numbers are not stated) | Pattern 4 | Low; the user picks the numbers |
| A3 | A per-`AsyncObserver` live-subscription counter with log + terminate in `~AsyncObserver` is the right handling of "observer destroyed while still subscribed" | Pattern 2, OQ 1 | Medium: alternative designs exist; it adds a loud-failure rule the user has not seen |
| A4 | `AsyncObserver` should own its `WakeSignal` instead of storing a raw pointer for cross-object use | Pattern 2 | Medium: changes the binding's `SetWakeSignal` use; alternative is a `Subscription` for the pointer |
| A5 | `MapSnapshot` contents (per-map ids/epoch, point ids + positions, frame ids + `T_cw`, active map) | Pattern 6 | Low: left to planning by D-13 |
| A6 | `NoSyncPolicy` pools use a plain counter | Pattern 1 | Low: discretion |
| A7 | Add `-Werror=unused-result` on test/example targets | Pitfall 6 | Low: a build-flag choice |
| A8 | ASan/LSan on common+events tests is a useful extra gate in R1 | Standard Stack | Low: optional |
| A9 | `MapFollower` asserts single-thread use in debug | Pattern 6 | Low |
| A10 | TSan configure line (reconstructed from `CMakeCache.txt` values) would recreate the scratch tree | Environment Availability | Low: the existing tree works as is |

## Open Questions

1. **Subscription across destruction orders (blocks the R1 design)**
   - What we know: D-06/D-07 define the Subscription's behaviour while the producer *runs*. They do not say what a Subscription destroyed **after its producer** touches (JNI closes `Slam` first), nor what happens if the **observer** is destroyed while a Subscription still refers to it and the producer is merely Created/Stopped (it would be pushed into on a later `Start`/`Notify`).
   - What's unclear: whether the user accepts a shared wiring block (one extra allocation per component) and a loud failure for "observer destroyed while subscribed".
   - Recommendation: Pattern 2 (shared `Wiring`, per-observer subscription counter, terminate on a violation). It keeps D-06's public shape and adds no per-event cost. Confirm with the user at planning.

2. **"`StopAll` guard is gone" versus D-07**
   - What we know: D-04/success criterion 1 say the guard goes; D-07 says destroying a Subscription of a running producer terminates. The Subscriptions must be created after `Slam` (from its components), so on exception unwinding they die before `Slam` stops.
   - What's unclear: whether a one-line `slam.Stop()` guard on error paths counts as the guard "gone" (the viewer stop and trajectory drain would be gone).
   - Recommendation: keep exactly `slam.Stop()` on every exit path (try/catch or a scope guard declared after the Subscriptions), remove the viewer/trajectory parts, and state it in DESIGN and in the user-facing summary. The CONTEXT "specifics" already say the binding must stop `Slam` before closing listeners.

3. **D-08 drain on disconnect would steal other producers' events**
   - What we know: the viewer's inbox is connected to Tracker and Mapper `[VERIFIED: monocular_full_slam.cpp:78-79 "slam.tracker().AddObserver(viewer.inbox()); slam.mapper().AddObserver(viewer.inbox());"]`; queued events carry no producer tag, so disconnecting from one producer cannot drain "its" events.
   - Recommendation: drain only in `~AsyncObserver` (and optionally on the last disconnect); events left queued after a disconnect stay valid (D-01) and are acknowledged by the consumer normally. This amends the wording of D-08; confirm.

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
| ASan tree `cmake-build-asan` | optional gate | yes | `ObjectPool*` 16 tests pass `[VERIFIED: ran]`; g2o-using tests abort (TODO PLAT-01) | filter to common/events tests |
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
| Sanitizer gate | TSan commands in "Code Examples"; ASan: `cmake --build cmake-build-asan --target voyah_common_tests voyah_events_tests -j8` then run with `--gtest_filter='ObjectPool*:EventSystem*:Subscription*'` |
| Release gate | `conan create . --build=missing -pr profiles/linux-gcc13` (static), `... -o shared=True` (shared), `conan create . --build=missing -pr:h profiles/android-arm64 -pr:b default` `[CITED: README.md:130-131, 174-175]`; `test_package` runs inside `conan create` |

**TSan note (falsification run, 2026-10-12):** `setarch -R cmake-build-tsan/libs/events/tests/voyah_events_tests` -> `34 tests ... PASSED`; the same binary without `setarch -R` -> `FATAL: ThreadSanitizer: unexpected memory mapping 0x5d9d542d4000-0x5d9d542ea000`. So every TSan command needs `setarch -R`.

### Phase Requirements -> Test Map
Test names are proposed; files marked "Wave 0" do not exist yet.

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| OWN-01 | Event returned after its producer (pool owner) is destroyed recycles safely; objects destroyed once, on the last return (any thread) | unit | `voyah_common_tests --gtest_filter='ObjectPoolTest.*Outlive*'` | extend `libs/common/tests/memory/object_pool_test.cpp` (and fix `DestructorDestroysEveryConstructedObjectOnce`) |
| OWN-01 | `num_outstanding()` counts slots out; owner leaving with slots out logs once; 0 after all returned | unit | `voyah_common_tests --gtest_filter='ObjectPoolTest.*Outstanding*'` | Wave 0 (same file) |
| OWN-01 | Listener / AsyncObserver outliving the producer recycles safely; Slam/Tracker+Mapper destroyed in any order | unit + integration | `voyah_events_tests --gtest_filter='*Outlive*'`, `voyah_integration_tests --gtest_filter='SlamTest.*Order*'` | Wave 0 |
| OWN-01 | Every Slam pool reaches 0 outstanding after Stop + consumer drain (D-02 test check) | integration | `voyah_integration_tests` (add checks to existing `SlamTest.*` end-of-test, via accessors like `event_pool_overflow()`) | extend `slam_test.cpp` |
| OWN-01 | `global_ba_` invariant comment and `StopAll` gone | review | grep: no `StopAll`, no "Safe because the Mapper recycles" | n/a |
| OWN-02 | Observer destroyed (after its Subscription) before the producer is never notified again | unit | `voyah_events_tests --gtest_filter='SubscriptionTest.*'` | Wave 0 `libs/events/tests/events/subscription_test.cpp` |
| OWN-02 | Producer destroyed first, Subscription after: no UAF (all 6 orders) | unit | same | Wave 0 |
| OWN-02 | `AddObserver`/`reset()` while Running throws `UsageError`; destroying a Subscription of a running producer terminates | unit, `*DeathTest` | `--gtest_filter='SubscriptionDeathTest.*'` (excluded from TSan) | Wave 0 |
| OWN-02 | `~AsyncObserver` drains and releases queued events; priority counter returns to 0 | unit | `--gtest_filter='AsyncObserverTest.*Drain*'` | extend `event_system_test.cpp` |
| OWN-02 | Examples compile and run with the new wiring | build + smoke | `cmake --build cmake-build-debug/build/Debug` then `monocular_full_slam` on a short synthetic/`--max-frames` run `[ASSUMED]` | existing apps |
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
- [ ] `libs/events/tests/events/subscription_test.cpp` - OWN-02 (add to `libs/events/tests/CMakeLists.txt`)
- [ ] `libs/events/tests/events/lifecycle_test.cpp` - OWN-04
- [ ] `libs/common/tests/images/image_test.cpp` - OWN-07 (add to `libs/common/tests/CMakeLists.txt`)
- [ ] `libs/common/tests/logging/logger_test.cpp` - OWN-06
- [ ] `libs/slam/tests/mapping/map_follower_test.cpp` and a shared `*TestAccess` header - OWN-03
- [ ] A workspace/concurrency test in `libs/algs/tests/features/` - OWN-05
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
| V14 / memory-safety and concurrency hygiene (analogous control) | yes | Pool core ownership (no UAF), shared wiring block, TSan/ASan gates |

### Known Threat Patterns for this stack

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Use-after-free via event recycled into a destroyed pool | Tampering / DoS | Pool core with intrusive count (D-01) |
| Use-after-free via observer destroyed before producer (and the reverse) | Tampering / DoS | Shared wiring block + Subscription (Pattern 2) |
| Data race on map data from a wrong-thread accessor call | Tampering | Private accessors + `*TestAccess` (D-14), `MapFollower` |
| Heap overflow from `Image(width, height)` unsigned wrap with caller-supplied dimensions | Tampering / EoP | Compute the size in `size_t`, reject overflow (Pitfall 16b) |
| Deadlock / corruption from concurrent `Start`/`Stop`/self-`Stop` | DoS | Atomic lifecycle with wait, self-stop detection (Pattern 3, Pitfall 9) |
| Exception escaping from a destructor or across the JNI boundary | DoS | Destructors never throw (D-07 terminates instead); binding maps `UsageError` |

## Sources

### Primary (HIGH confidence)
- In-repo files read this session: `libs/common/include/voyahslam/memory/{object_pool,recyclable,sync_policy}.h`, `libs/common/src/memory/object_pool.cpp`, `libs/events/include/voyahslam/events/{observable,event_node,async_observer,listener,event_types,wake_signal,worker_thread,frame_tracked,global_ba_finished,map_transaction}.h`, `libs/events/src/events/worker_thread.cpp`, `libs/slam/include/voyahslam/{slam/slam.h,tracking/tracker.h,mapping/mapper.h,loop_closing/loop_closer.h,global_ba/global_bundle_adjuster.h,tracking/{atlas_replica,map_replica}.h}`, `libs/slam/src/{slam/slam.cpp,tracking/tracker.cpp,mapping/mapper.cpp,global_ba/global_bundle_adjuster.cpp}`, `libs/common/include/voyahslam/{images/image.h,logging/logger.h,features/orb_feature_extractor.h,features/orb/image_pyramid.h}`, `libs/common/src/logging/logger.cpp`, `libs/algs/src/features/{orb_feature_extractor.cpp,orb/image_pyramid.cpp}`, `examples/full_slam/*`, `vizualization/include/voyahviz/{map_viewer,scene_state}.h`, tests and CMake files cited, `docs/{DESIGN,TODO,ROADMAP}.md`, `CLAUDE.md`, `../voyah_jni/docs/voyah_slam_ownership_audit.md`, `../voyah_jni/native/src/slam_session.{h,cpp}`.
- spdlog 1.17.0 headers in the local Conan cache (`dist_sink.h`, `registry-inl.h`): the verbatim quotes above.
- Commands run this session: `check` (746 pass), `ctest -N` counts, TSan build and run of `voyah_events_tests`, ASan run of `ObjectPool*`, `conan list`.

### Secondary (MEDIUM confidence)
- README.md lines 126-131, 174-175 for the `conan create` commands (cited, not re-run).

### Tertiary (LOW confidence)
- None. Context7 and web lookups were not needed: no external library behaviour beyond the locally readable spdlog headers is relied on. The `research-plan` seam was therefore not used.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH - nothing new; pins read from `conanfile.py`.
- Architecture: MEDIUM - code facts are verified; the wiring block, observer counter and `MapSnapshot` shape are recommendations inside the user's discretion or filling gaps (Open Questions 1-3).
- Pitfalls: HIGH - each was found in code read this session or a command run.

**Research date:** 2026-10-12
**Valid until:** 2026-11-11 (stable; invalidated earlier if another session changes `libs/events` or the pool before R1 starts)

## Project Constraints (from CLAUDE.md)

- Read `docs/DESIGN.md` first; **record every design decision in the decisions log (and the relevant section) in the same change**; add deferred work to `docs/TODO.md`; ordered next steps are in `docs/ROADMAP.md` (keep `.planning/ROADMAP.md` in sync per its sync rule).
- No locks on map data; threads communicate only via events, each thread owns a map replica; the Mapper is the only map writer; Tracker and LoopCloser never mutate map objects.
- Frame ids are never reused in a run; map point ids are append-only per map; `(map_id, id)` identifies a point.
- Google style naming: `PascalCase` types/functions, `snake_case` variables, `snake_case_` members, `out_` prefix for non-const reference parameters.
- Layout: four libraries under `libs/<lib>/{include,src,tests}`; `voyah_events` and `voyah_algs` depend on common only; `voyah_slam` on all three; a lower layer cannot include an upper one (so `UsageError` and the pool live in common, `Subscription` in events).
- Build/test: `cmake --build cmake-build-debug/build/Debug --target check` (746 unit tests) and `--target check_all` (773) at the end of a task and before a merge; tests run with `VOYAHSLAM_OPTIMIZE_TESTS=ON` (-O2, asserts on).
- Project memory: say whether the work fixes the reported symptom, with criteria on the symptom (here: "objects die in any order, calls from any thread, cannot crash the library") rather than proxies; merges, pushes and version bumps only on the user's explicit "merge and push".
