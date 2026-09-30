#ifndef VOYAHSLAM_EVENTS_MAP_EVENTS_H
#define VOYAHSLAM_EVENTS_MAP_EVENTS_H

#include <cstdint>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

template <typename TFeatureExtractor>
struct MapPointsAdded
    : Event
    , memory::Recyclable<MapPointsAdded<TFeatureExtractor>, memory::ThreadSafePolicy> {
    map::MapPointId start_id;
    uint32_t count;
    geometry::TPoints3DF positions;
    typename TFeatureExtractor::TDescriptors descriptors;
    geometry::TPoints3DF normals;

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
    geometry::TPoints3DF positions;
    typename TFeatureExtractor::TDescriptors descriptors;
    geometry::TPoints3DF normals;

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
