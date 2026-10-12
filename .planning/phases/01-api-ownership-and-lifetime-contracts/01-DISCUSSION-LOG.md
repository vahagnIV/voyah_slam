# Phase 1: API Ownership and Lifetime Contracts - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-10-12
**Phase:** 01-api-ownership-and-lifetime-contracts
**Areas discussed:** Pool lifetime (#3), Subscription API (#2), Misuse errors & restart, Map read path (#5), Extractor safety (#7), Logger re-targeting (#8), Image fixes (#9), Plans/branches/versions

---

## Roadmap correction (before the discussion)

`/gsd-discuss-phase 1` first opened on "Loop-Closing False-Positive Rejection". The user pointed
out that the voyah_jni ownership audit was in flight (#1 and #4 already merged) and that the
onboarding had treated it as an overlap note on a later phase.

| Option | Description | Selected |
|--------|-------------|----------|
| Audit becomes Phase 1 | New Phase 1 for #2, #3, #5-#9; loop closing to Phase 2; record the audit in docs/TODO.md and docs/ROADMAP.md | ✓ |
| Only #3+#2 as Phase 1 | Rest folded into a later phase | |
| Keep loop first | Audit inserted after loop closing | |
| Skip GSD for the audit | Only add to TODO | |

---

## Pool lifetime (#3)

| Option | Description | Selected |
|--------|-------------|----------|
| Intrusive core refcount | Pool = handle to heap core counting owner + handed-out slots; last Recycle frees | ✓ |
| shared_ptr<PoolCore> per event | Standard type; self-destruction trap in Recycle; redundant refcount | |
| Static / global pools | Never freed, shared across instances, exit-order problem | |

**Notes:** The user first asked for the pros and cons of each option, then asked what the point
of pools is if `shared_ptr` is used. The answer: the `shared_ptr` would point to the pool, not to
each event, and copying it does not allocate, so publishing stays allocation-free.

| Leak reporting | Selected |
|---|---|
| Log + `num_outstanding()` + test check | ✓ |
| Debug assert at owner drop | |
| Nothing | |

| Pool freed on the last consumer's thread | Selected |
|---|---|
| Yes, free there (DESIGN rule on destructors) | ✓ |
| No, hand back to the owner thread | |

---

## Subscription API (#2)

| Disconnect while the producer runs? | Selected |
|---|---|
| Yes, any time (lock-free closed flag) | |
| Only while stopped | ✓ |
| Explain more | |

| Connection form | Selected |
|---|---|
| RAII Subscription | ✓ |
| shared_ptr observers | |
| Both | |

| Subscription destroyed while running | Selected |
|---|---|
| Log + std::terminate (an explicit disconnect throws) | ✓ |
| Debug assert only | |
| Defer to next Stop | |

| Queued events on disconnect | Selected |
|---|---|
| Drain and release | ✓ |
| Leave to the consumer | |

---

## Misuse errors & restart (#6)

| Failure mode | Selected |
|---|---|
| Throw logic_error subclass | ✓ |
| Status return | |
| Debug assert only | |

| Stop then Start | Selected |
|---|---|
| No, Stopped is terminal | ✓ |
| Yes, resume | |
| Yes, as reset | |

| Controlling thread | Selected |
|---|---|
| Any thread, serialized by an atomic CAS | ✓ |
| Enforce one thread | |

| Track before Start / after Stop | Selected |
|---|---|
| Throw in both cases | |
| Queue before Start, throw after Stop | ✓ |

---

## Map read path (#5)

| Read path | Selected |
|---|---|
| MapFollower + snapshots | ✓ |
| MapFollower only | |
| Snapshots only | |

| Accessor enforcement | Selected |
|---|---|
| Move behind TestAccess | ✓ |
| Debug thread assert | |
| Throw if running | |

| Session facade | Selected |
|---|---|
| Defer | ✓ |
| Include now | |

| voyahviz onto MapFollower | Selected |
|---|---|
| No, later | ✓ |
| Yes, dogfood | |

---

## Extractor safety (#7)

| Option | Selected |
|---|---|
| Non-const | |
| Caller-provided workspace | ✓ |
| Thread-local workspace | |

| Convenience overload without a workspace | Selected |
|---|---|
| No, workspace required | |
| Yes, allocating overload | ✓ |

| Workspace owner | Selected |
|---|---|
| Caller, one per thread | ✓ |
| The extractor hands them out | |

---

## Logger re-targeting (#8)

| Option | Selected |
|---|---|
| One shared dist_sink | ✓ |
| Re-target via the registry | |

| Logcat sink in this item | Selected |
|---|---|
| No, stays Phase 5 | ✓ |
| Yes | |

---

## Image fixes (#9)

| Option | Selected |
|---|---|
| Fix in place | ✓ |
| Separate ImageView type | |

---

## Plans, branches, versions

| Option | Selected |
|---|---|
| #3+#2 together, the rest one release each | ✓ |
| One release per audit item | |
| Batch #7+#8+#9 | |

---

## Claude's Discretion

- The pool core's internal layout; the `Subscription` type split; the `UsageError` name and location; `MapSnapshot` contents and the `MapFollower` API; the extractor `Workspace` name and contents.

## Deferred Ideas

- Disconnect while running (lock-free); the `voyahslam::Session` facade; voyahviz `SceneState` on `MapFollower`; the logcat default sink (Phase 5); lossy delivery (Phase 5).
