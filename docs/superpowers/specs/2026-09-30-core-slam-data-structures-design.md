# Core SLAM Data Structures — Design Spec

## Context

voyahslam is an ORB-SLAM3 inspired SLAM library using an event-driven architecture. Three main threads (Tracker, Mapper, LoopCloser) each maintain their own map copy. The Mapper is the single map writer and publishes imperative diff events through lock-free concurrent queues (moodycamel::ConcurrentQueue). Other threads apply these diffs to their local map copies.

This spec covers the core data layer: memory infrastructure, map storage, frames, observations, camera rig, event system, and Observer/Observable pub-sub.

## 1. Memory Infrastructure

Port from `slamlab2/include/slamlab/memory/` with namespace change to `voyahslam::memory`.

### ObjectPool\<T, SyncPolicy\>

Pre-allocated slab allocator with intrusive free-list. Zero heap allocation on `Create()`/`Recycle()`.

- Constructor pre-allocates `max_count` slots of `max(sizeof(T), sizeof(FreeNode))` bytes
- `Create(args...)` — pops from free-list, placement-new constructs T, sets `object_pool_` pointer
- `Recycle(T*)` — calls destructor, pushes slot back to free-list
- SyncPolicy controls locking: `ThreadSafePolicy` (mutex) or `NoSyncPolicy` (zero-cost)

### Recyclable\<T, SyncPolicy\> (CRTP mixin)

Adds atomic reference counting to any type. When ref count reaches 0, the object is returned to its pool.

- `ref_count_` — `std::atomic_int`, starts at 1
- `AddRef()` — relaxed atomic increment
- `Recycle()` — relaxed atomic decrement; if result is 0, calls `object_pool_->Recycle(this)`
- **New addition:** `SetRefCount(int n)` — atomic store, used by Observable::Notify() to set count to N observers in one operation instead of N-1 AddRef() calls

### SyncPolicy

- `ThreadSafePolicy` — `std::mutex` + RAII `Lock` wrapper
- `NoSyncPolicy` — no-op `Lock`

**Files:** `include/voyahslam/memory/object_pool.h`, `recyclable.h`, `sync_policy.h`

## 2. Observer / Observable

Template-based pub-sub with no virtual functions. Each class can inherit from multiple `Observable<TEventBase>` and `Observer<TEventBase>`.

### Observer\<TEventBase\>

```cpp
template <typename TEventBase>
class Observer {
    moodycamel::ConcurrentQueue<TEventBase*> queue_;
public:
    bool TryDequeue(TEventBase*& out);
    moodycamel::ConcurrentQueue<TEventBase*>& queue();
};
```

### Observable\<TEventBase\>

```cpp
template <typename TEventBase>
class Observable {
    std::vector<moodycamel::ConcurrentQueue<TEventBase*>*> observers_;
public:
    void Subscribe(Observer<TEventBase>& observer);
    void Notify(TEventBase* event);   // sets refcount = observer_count, enqueues to all
    size_t observer_count() const;
};
```

`Notify()` expects the event to be `Recyclable`. It calls `SetRefCount(observers_.size())` then enqueues the raw pointer to each observer's queue. Each observer calls `Recycle()` on the concrete type after processing — the last one returns the event to its pool.

**Files:** `include/voyahslam/events/observer.h`, `include/voyahslam/events/observable.h`

## 3. Event System

### Base Event

```cpp
enum class EventType : uint8_t {
    NewKeyFrameCandidate,    // Tracker → Mapper
    MapPointsAdded,          // Mapper → consumers
    MapPointsUpdated,        // Mapper → consumers
    MapPointsRemoved,        // Mapper → consumers
    KeyFrameCreated,         // Mapper → consumers
    KeyFramePoseUpdated,     // Mapper → consumers
    LoopDetected,            // LoopCloser → Mapper
    PositionUpdated,         // Tracker → Visualizer/consumers
};

struct Event {
    EventType type;
protected:
    explicit Event(EventType t) : type(t) {}
};
```

No virtual functions. `Event` is a tag-carrying base; consumers switch on `type` and `static_cast` to the concrete type.

### Concrete Events

Each concrete event inherits from both `Event` (for the type tag) and `Recyclable<ConcreteType, ThreadSafePolicy>` (for pool recycling). Templated on `TFeatureExtractor` where they carry descriptors.

**Mapper diff events (templated on TFeatureExtractor):**

```
MapPointsAdded<FE>:     start_id, count, positions (3×N), descriptors (DescSize×N), normals (3×N)
MapPointsUpdated<FE>:   ids[], positions (3×N), descriptors (DescSize×N)  
MapPointsRemoved:       ids[]
KeyFrameCreated<FE,CF>: frame (Frame<FE,CF>)
KeyFramePoseUpdated:    frame_id, pose (TPose)
```

**Tracker events:**

```
NewKeyFrameCandidate<FE,CF>: frame, observations (mappoint_id, feature_idx pairs)
PositionUpdated:              frame_id, pose, timestamp
```

**LoopCloser events:**

```
LoopDetected: correction data (pose corrections, point fusions)
```

Each concrete event type has its own `ObjectPool` instance. The `Recyclable` mixin's `object_pool_` pointer routes `Recycle()` back to the correct pool.

**Consumer pattern:**
```cpp
Event* event;
while (observer.TryDequeue(event)) {
    switch (event->type) {
        case EventType::MapPointsAdded: {
            auto* e = static_cast<MapPointsAdded<FE>*>(event);
            // apply diff to local map...
            e->Recycle();
            break;
        }
    }
}
```

**Files:** `include/voyahslam/events/event_types.h`, `include/voyahslam/events/map_events.h`

## 4. Strong ID Types

Lightweight wrappers over `uint32_t`. Prevent mixing map-point and frame indices.

```cpp
struct MapPointId {
    uint32_t value;
    bool operator==(const MapPointId&) const = default;
    auto operator<=>(const MapPointId&) const = default;
};

struct FrameId {
    uint32_t value;
    // same operators
};
```

**Files:** `include/voyahslam/map/map_point_id.h`, `include/voyahslam/map/frame_id.h`

## 5. MapPointStorage (SoA)

Structure-of-Arrays storage for all map point properties. Templated on `TFeatureExtractor`.

```cpp
template <typename TFeatureExtractor>
class MapPointStorage {
    using TDescScalar = typename TFeatureExtractor::TDescriptorScalarType;
    static constexpr int kDescSize = TFeatureExtractor::DescriptorSize;

    geometry::TPoints3DF positions_;               // 3 × capacity
    Eigen::Matrix<TDescScalar, kDescSize, Eigen::Dynamic> descriptors_;
    geometry::TPoints3DF normals_;                 // 3 × capacity  
    Eigen::VectorXi num_observations_;
    std::vector<bool> valid_;
    uint32_t size_ = 0;
    uint32_t capacity_ = 0;
    std::vector<uint32_t> free_list_;

public:
    MapPointId Add(const geometry::TPoint3DF& position,
                   const typename TFeatureExtractor::TDescriptor& descriptor,
                   const geometry::TPoint3DF& normal);
    void Remove(MapPointId id);
    
    auto Position(MapPointId id);          // Eigen column ref
    auto Descriptor(MapPointId id);        // Eigen column ref
    auto Normal(MapPointId id);
    bool IsValid(MapPointId id) const;
    uint32_t size() const;
    
    const geometry::TPoints3DF& positions() const;
    const auto& descriptors() const;
};
```

**ID management:** Dense IDs with free-list. `Add()` reuses from `free_list_` or appends (growing capacity). `Remove()` marks slot invalid and pushes ID to free-list. Iteration must check `valid_[i]`.

**File:** `include/voyahslam/map/map_point_storage.h`

## 6. StereoRig

Immutable camera rig configuration. Shared by all frames and threads.

```cpp
struct StereoRig {
    geometry::TMatrix33F K_left;
    geometry::TMatrix33F K_right;
    geometry::TMatrix33F R_rl;               // rotation right-to-left
    geometry::TVector3DF t_rl;               // translation right-to-left
    precision_t baseline;                     // ||t_rl||
    const distortion_models::DistortionModel* distortion_left = nullptr;
    const distortion_models::DistortionModel* distortion_right = nullptr;
};
```

Constructed once at startup. Const after construction — no synchronization needed.

**File:** `include/voyahslam/features/stereo_rig.h`

## 7. BinocularCameraFrame

Two CameraFrames (left + right) with stereo matching results. Same template convention as CameraFrame: `<TScalar, Length>`.

```cpp
template <typename TScalar, unsigned int Length>
class BinocularCameraFrame {
    CameraFrame<TScalar, Length> left_;
    CameraFrame<TScalar, Length> right_;
    geometry::TRowVectorF depths_;              // per left-feature metric depth
    Eigen::VectorXi right_indices_;             // matched right feature idx (-1 = unmatched)
    
public:
    BinocularCameraFrame(
        ImageFeatures<TScalar, Length>&& left_features,
        ImageFeatures<TScalar, Length>&& right_features,
        const StereoRig& rig);
    
    const CameraFrame<TScalar, Length>& left() const;
    const CameraFrame<TScalar, Length>& right() const;
    const geometry::TRowVectorF& depths() const;
    const Eigen::VectorXi& right_indices() const;
    precision_t depth(int idx) const;
};
```

Does NOT own or copy rig data. Constructor takes rig by const ref to access K, distortion, baseline for stereo matching and depth computation.

**File:** `include/voyahslam/features/binocular_camera_frame.h`

## 8. Frame (KeyFrame equivalent)

In our terminology, Frame = KeyFrame in ORB-SLAM3. Only keyframes become Frame objects. Tracker holds current tracking state internally; non-keyframe tracking data doesn't need a Frame.

```cpp
template <typename TFeatureExtractor, typename TCameraFrameType>
class Frame {
    FrameId id_;
    geometry::TPose pose_;
    TCameraFrameType camera_frame_;
    double timestamp_;

public:
    Frame(FrameId id, const geometry::TPose& pose,
          TCameraFrameType&& camera_frame, double timestamp);
    
    FrameId id() const;
    const geometry::TPose& pose() const;
    geometry::TPose& mutable_pose();
    const TCameraFrameType& camera_frame() const;
    double timestamp() const;
};
```

**Mono/stereo switching:**
```cpp
template <typename FE>
using MonoFrame = Frame<FE, 
    CameraFrame<typename FE::TDescriptorScalarType, FE::DescriptorSize>>;

template <typename FE>
using StereoFrame = Frame<FE, 
    BinocularCameraFrame<typename FE::TDescriptorScalarType, FE::DescriptorSize>>;
```

**File:** `include/voyahslam/map/frame.h`

## 9. ObservationStorage (SoA)

Flat table linking map points to frames. Templated on observation type for mono/stereo.

```cpp
enum class ObservationType { Monocular, Stereo };

template <ObservationType Type>
class ObservationStorage {
    std::vector<uint32_t> mappoint_ids_;
    std::vector<uint32_t> frame_ids_;
    std::vector<uint16_t> feature_indices_;
    std::vector<precision_t> depths_;               // empty for Monocular
    std::vector<int16_t> right_feature_indices_;    // empty for Monocular

public:
    void Add(MapPointId mp, FrameId fr, uint16_t feat_idx);
    
    // Stereo overload — enabled only for Stereo type via if constexpr or SFINAE
    void Add(MapPointId mp, FrameId fr, uint16_t feat_idx,
             precision_t depth, int16_t right_idx);
    
    size_t size() const;
    
    MapPointId mappoint_id(size_t idx) const;
    FrameId frame_id(size_t idx) const;
    uint16_t feature_index(size_t idx) const;
};
```

For Monocular, the stereo vectors remain empty and the stereo `Add` overload is disabled at compile time. Zero overhead.

**File:** `include/voyahslam/map/observation_storage.h`

## 10. Dependency

Add `concurrentqueue` (moodycamel) to conanfile.txt. It's available as `concurrentqueue/1.0.4` on conan-center.

## 11. Thread Ownership Summary

```
Tracker:
  - Observable<Event>  → publishes to: Mapper, Visualizer
  - Observer<Event>    → subscribes to: Mapper (map diffs)
  - Owns: local MapPointStorage, local ObservationStorage, current tracking pose

Mapper:
  - Observable<Event>  → publishes to: Tracker, LoopCloser, Visualizer  
  - Observer<Event>    → subscribes to: Tracker (keyframe candidates), LoopCloser (corrections)
  - Owns: authoritative MapPointStorage, Frames, ObservationStorage
  - Single writer of map data

LoopCloser:
  - Observable<Event>  → publishes to: Mapper
  - Observer<Event>    → subscribes to: Mapper (map diffs)
  - Owns: local MapPointStorage (for loop detection)

Visualizer (future):
  - Observer<Event>    → subscribes to: Tracker (poses), Mapper (map diffs)
  - Owns: local MapPointStorage (for rendering)
```

## 12. File Organization

```
include/voyahslam/
├── memory/
│   ├── object_pool.h
│   ├── recyclable.h
│   └── sync_policy.h
├── events/
│   ├── event_types.h          // EventType enum + Event base struct
│   ├── map_events.h           // all concrete event structs
│   ├── observer.h
│   └── observable.h
├── map/
│   ├── map_point_id.h
│   ├── frame_id.h
│   ├── map_point_storage.h
│   ├── frame.h
│   └── observation_storage.h
├── features/
│   ├── stereo_rig.h           // new
│   ├── binocular_camera_frame.h // new
│   └── ... (existing, unchanged)
```

## 13. Implementation Order

1. Memory module (port + SetRefCount addition)
2. Strong IDs (MapPointId, FrameId)
3. StereoRig
4. BinocularCameraFrame
5. MapPointStorage
6. Frame
7. ObservationStorage
8. Event types + Observer/Observable
9. Add moodycamel dependency
10. Tests for each component

## 14. Verification

- Unit tests for each new class
- MapPointStorage: add/remove/reuse cycle, bulk access, valid/invalid slot checks
- BinocularCameraFrame: construction with StereoRig
- ObservationStorage: add observations mono + stereo, size checks
- Memory: pool create/recycle lifecycle, ref counting, multi-observer Recycle ordering
- Observer/Observable: publish to multiple observers, ref count reaches 0
- Build with both conan-release and conan-debug presets
- Existing 78 tests still pass
