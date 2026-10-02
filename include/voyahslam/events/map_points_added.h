#ifndef VOYAHSLAM_EVENTS_MAP_POINTS_ADDED_H
#define VOYAHSLAM_EVENTS_MAP_POINTS_ADDED_H

#include <cstdint>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
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

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_MAP_POINTS_ADDED_H
