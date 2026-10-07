# voyahslam

Lock-free, event-driven, ORB-SLAM3-inspired SLAM library (C++20, Eigen, Conan 2).

The ordered plan of next steps is in `docs/ROADMAP.md`.
Pending work is tracked in `docs/TODO.md` — check it, and add items there
whenever something is deferred.

**Read `docs/DESIGN.md` first** — it holds the architecture, principles, coding
conventions, decisions log, open questions and next steps. Keep it up to date:
whenever a design decision is made or changed, record it there (decisions log +
relevant section) in the same change.

Key rules (details in DESIGN.md):
- No locks on map data. Threads communicate only via events; each thread owns a map replica.
- Mapper is the only map writer. Tracker and LoopCloser never mutate map objects.
- Frame IDs are never reused within a run. Map point IDs are never reused within a
  map (append-only); they restart per map, and `(map_id, id)` identifies a point.
- Google style naming: `PascalCase` types/functions, `snake_case` variables,
  `snake_case_` members, `out_` prefix for non-const reference parameters.
