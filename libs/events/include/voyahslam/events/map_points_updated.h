#ifndef VOYAHSLAM_EVENTS_MAP_POINTS_UPDATED_H
#define VOYAHSLAM_EVENTS_MAP_POINTS_UPDATED_H

#include <cstdint>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/column_buffer.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

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

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_MAP_POINTS_UPDATED_H
