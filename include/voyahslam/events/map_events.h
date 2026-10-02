#ifndef VOYAHSLAM_EVENTS_MAP_EVENTS_H
#define VOYAHSLAM_EVENTS_MAP_EVENTS_H

#include <cstdint>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/column_buffer.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

template <typename TFeatureExtractor>
struct MapPointsAdded
    : Event
    , memory::Recyclable<MapPointsAdded<TFeatureExtractor>, memory::ThreadSafePolicy> {
    // New points occupy the contiguous id range [start_id, start_id + count()).
    map::MapPointId start_id{};
    memory::ColumnBuffer<precision_t, 3> positions;
    memory::ColumnBuffer<typename TFeatureExtractor::TDescriptorScalarType,
                         TFeatureExtractor::DescriptorSize> descriptors;
    memory::ColumnBuffer<precision_t, 3> normals;

    // Sets the number of points in all columns; allocates only beyond capacity.
    void Resize(uint32_t count) {
        positions.Resize(count);
        descriptors.Resize(count);
        normals.Resize(count);
    }
    [[nodiscard]] uint32_t count() const { return static_cast<uint32_t>(positions.cols()); }
    void Reset() {
        start_id = {};
        Resize(0);
    }

    MapPointsAdded()
        : Event(EventType::MapPointsAdded,
                [](Event* e) {
                    static_cast<MapPointsAdded*>(e)->
                        memory::Recyclable<MapPointsAdded, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<MapPointsAdded*>(e)->SetRefCount(n);
                }) {}
};

template <typename TFeatureExtractor>
struct MapPointsUpdated
    : Event
    , memory::Recyclable<MapPointsUpdated<TFeatureExtractor>, memory::ThreadSafePolicy> {
    std::vector<map::MapPointId> ids;
    memory::ColumnBuffer<precision_t, 3> positions;
    memory::ColumnBuffer<typename TFeatureExtractor::TDescriptorScalarType,
                         TFeatureExtractor::DescriptorSize> descriptors;
    memory::ColumnBuffer<precision_t, 3> normals;

    // Sets the number of points in ids and all columns; allocates only beyond capacity.
    void Resize(uint32_t count) {
        ids.resize(count);
        positions.Resize(count);
        descriptors.Resize(count);
        normals.Resize(count);
    }
    [[nodiscard]] uint32_t count() const { return static_cast<uint32_t>(ids.size()); }
    void Reset() { Resize(0); }

    MapPointsUpdated()
        : Event(EventType::MapPointsUpdated,
                [](Event* e) {
                    static_cast<MapPointsUpdated*>(e)->
                        memory::Recyclable<MapPointsUpdated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<MapPointsUpdated*>(e)->SetRefCount(n);
                }) {}
};

struct MapPointsRemoved
    : Event
    , memory::Recyclable<MapPointsRemoved, memory::ThreadSafePolicy> {
    std::vector<map::MapPointId> ids;

    void Reset() { ids.clear(); }

    MapPointsRemoved()
        : Event(EventType::MapPointsRemoved,
                [](Event* e) {
                    static_cast<MapPointsRemoved*>(e)->
                        memory::Recyclable<MapPointsRemoved, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<MapPointsRemoved*>(e)->SetRefCount(n);
                }) {}
};

struct KeyFramePoseUpdated
    : Event
    , memory::Recyclable<KeyFramePoseUpdated, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;

    KeyFramePoseUpdated()
        : Event(EventType::KeyFramePoseUpdated,
                [](Event* e) {
                    static_cast<KeyFramePoseUpdated*>(e)->
                        memory::Recyclable<KeyFramePoseUpdated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<KeyFramePoseUpdated*>(e)->SetRefCount(n);
                }) {}
};

struct PositionUpdated
    : Event
    , memory::Recyclable<PositionUpdated, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;
    double timestamp;

    PositionUpdated()
        : Event(EventType::PositionUpdated,
                [](Event* e) {
                    static_cast<PositionUpdated*>(e)->
                        memory::Recyclable<PositionUpdated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<PositionUpdated*>(e)->SetRefCount(n);
                }) {}
};

template <typename TFeatureExtractor, typename TCameraFrameType>
struct KeyFrameCreated
    : Event
    , memory::Recyclable<KeyFrameCreated<TFeatureExtractor, TCameraFrameType>, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;
    TCameraFrameType camera_frame;
    double timestamp;

    KeyFrameCreated()
        : Event(EventType::KeyFrameCreated,
                [](Event* e) {
                    static_cast<KeyFrameCreated*>(e)->
                        memory::Recyclable<KeyFrameCreated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<KeyFrameCreated*>(e)->SetRefCount(n);
                }) {}
};

template <typename TFeatureExtractor, typename TCameraFrameType>
struct NewKeyFrameCandidate
    : Event
    , memory::Recyclable<NewKeyFrameCandidate<TFeatureExtractor, TCameraFrameType>, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;
    TCameraFrameType camera_frame;
    double timestamp;

    NewKeyFrameCandidate()
        : Event(EventType::NewKeyFrameCandidate,
                [](Event* e) {
                    static_cast<NewKeyFrameCandidate*>(e)->
                        memory::Recyclable<NewKeyFrameCandidate, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<NewKeyFrameCandidate*>(e)->SetRefCount(n);
                }) {}
};

struct LoopDetected
    : Event
    , memory::Recyclable<LoopDetected, memory::ThreadSafePolicy> {
    map::FrameId query_frame_id;
    map::FrameId match_frame_id;

    LoopDetected()
        : Event(EventType::LoopDetected,
                [](Event* e) {
                    static_cast<LoopDetected*>(e)->
                        memory::Recyclable<LoopDetected, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<LoopDetected*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_MAP_EVENTS_H
