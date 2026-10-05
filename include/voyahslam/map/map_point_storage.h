#ifndef VOYAHSLAM_MAP_MAP_POINT_STORAGE_H
#define VOYAHSLAM_MAP_MAP_POINT_STORAGE_H

#include <cassert>
#include <cstdint>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::map {

// Points of one map. Ids are append-only: Add() issues 0, 1, 2, ... in order and
// Remove() only marks a point dead, so an id never refers to two points. Ids
// restart only with a fresh storage (a new map); (map_id, id) identifies a point.
template <typename TFeatureExtractor>
class MapPointStorage {
public:
    using TDescScalar = typename TFeatureExtractor::TDescriptorScalarType;
    using TDescriptor = typename TFeatureExtractor::TDescriptor;
    using TDescriptors = typename TFeatureExtractor::TDescriptors;
    static constexpr int kDescSize = TFeatureExtractor::DescriptorSize;

    MapPointStorage() = default;

    MapPointId Add(const geometry::TPoint3DF& pos,
                   const TDescriptor& desc,
                   const geometry::TPoint3DF& normal);
    void Remove(MapPointId id);

    [[nodiscard]] auto Position(MapPointId id) { return positions_.col(id.value); }
    [[nodiscard]] auto Position(MapPointId id) const { return positions_.col(id.value); }
    [[nodiscard]] auto Descriptor(MapPointId id) { return descriptors_.col(id.value); }
    [[nodiscard]] auto Descriptor(MapPointId id) const { return descriptors_.col(id.value); }
    [[nodiscard]] auto Normal(MapPointId id) { return normals_.col(id.value); }
    [[nodiscard]] auto Normal(MapPointId id) const { return normals_.col(id.value); }

    [[nodiscard]] bool IsValid(MapPointId id) const;
    // Number of live (not removed) points.
    [[nodiscard]] uint32_t size() const { return size_; }
    // Number of ids ever issued; the next Add() returns MapPointId{num_ids()}.
    [[nodiscard]] uint32_t num_ids() const { return num_ids_; }
    [[nodiscard]] const geometry::TPoints3DF& positions() const { return positions_; }
    [[nodiscard]] const TDescriptors& descriptors() const { return descriptors_; }

private:
    static constexpr uint32_t kInitialCapacity = 256;

    geometry::TPoints3DF positions_;
    TDescriptors descriptors_;
    geometry::TPoints3DF normals_;
    Eigen::VectorXi num_observations_;
    std::vector<bool> valid_;
    uint32_t size_ = 0;
    uint32_t num_ids_ = 0;
    uint32_t capacity_ = 0;

    void Grow();
};

template <typename TFeatureExtractor>
MapPointId MapPointStorage<TFeatureExtractor>::Add(
    const geometry::TPoint3DF& pos,
    const TDescriptor& desc,
    const geometry::TPoint3DF& normal) {
    if (num_ids_ == capacity_) {
        Grow();
    }
    const uint32_t slot = num_ids_++;

    positions_.col(slot) = pos;
    descriptors_.col(slot) = desc;
    normals_.col(slot) = normal;
    num_observations_(slot) = 0;
    valid_[slot] = true;
    ++size_;

    return MapPointId{slot};
}

template <typename TFeatureExtractor>
void MapPointStorage<TFeatureExtractor>::Remove(MapPointId id) {
    assert(id.value < capacity_ && "MapPointId out of range");
    if (!valid_[id.value]) return;
    valid_[id.value] = false;
    --size_;
}

template <typename TFeatureExtractor>
bool MapPointStorage<TFeatureExtractor>::IsValid(MapPointId id) const {
    return id.value < capacity_ && valid_[id.value];
}

template <typename TFeatureExtractor>
void MapPointStorage<TFeatureExtractor>::Grow() {
    uint32_t new_capacity = (capacity_ == 0) ? kInitialCapacity : capacity_ * 2;
    positions_.conservativeResize(Eigen::NoChange, new_capacity);
    descriptors_.conservativeResize(Eigen::NoChange, new_capacity);
    normals_.conservativeResize(Eigen::NoChange, new_capacity);
    num_observations_.conservativeResize(new_capacity);
    valid_.resize(new_capacity, false);
    capacity_ = new_capacity;
}

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_MAP_POINT_STORAGE_H
