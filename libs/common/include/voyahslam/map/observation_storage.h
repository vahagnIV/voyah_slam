#ifndef VOYAHSLAM_MAP_OBSERVATION_STORAGE_H
#define VOYAHSLAM_MAP_OBSERVATION_STORAGE_H

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::map {

enum class ObservationType { Monocular, Stereo };

template <ObservationType Type>
class ObservationStorage {
public:
    void Add(MapPointId mp, FrameId fr, uint16_t feat_idx);

    template <ObservationType T = Type,
              std::enable_if_t<T == ObservationType::Stereo, int> = 0>
    void Add(MapPointId mp, FrameId fr, uint16_t feat_idx,
             precision_t depth, int16_t right_idx);

    [[nodiscard]] size_t size() const { return mappoint_ids_.size(); }
    [[nodiscard]] MapPointId mappoint_id(size_t idx) const { return MapPointId{mappoint_ids_[idx]}; }
    [[nodiscard]] FrameId frame_id(size_t idx) const { return FrameId{frame_ids_[idx]}; }
    [[nodiscard]] uint16_t feature_index(size_t idx) const { return feature_indices_[idx]; }

    [[nodiscard]] precision_t depth(size_t idx) const requires (Type == ObservationType::Stereo) {
        return depths_[idx];
    }
    [[nodiscard]] int16_t right_feature_index(size_t idx) const requires (Type == ObservationType::Stereo) {
        return right_feature_indices_[idx];
    }

private:
    std::vector<uint32_t> mappoint_ids_;
    std::vector<uint32_t> frame_ids_;
    std::vector<uint16_t> feature_indices_;
    std::vector<precision_t> depths_;
    std::vector<int16_t> right_feature_indices_;
};

template <ObservationType Type>
void ObservationStorage<Type>::Add(MapPointId mp, FrameId fr, uint16_t feat_idx) {
    mappoint_ids_.push_back(mp.value);
    frame_ids_.push_back(fr.value);
    feature_indices_.push_back(feat_idx);
    if constexpr (Type == ObservationType::Stereo) {
        depths_.push_back(-1.0);
        right_feature_indices_.push_back(-1);
    }
}

template <ObservationType Type>
template <ObservationType T, std::enable_if_t<T == ObservationType::Stereo, int>>
void ObservationStorage<Type>::Add(MapPointId mp, FrameId fr, uint16_t feat_idx,
                                   precision_t depth, int16_t right_idx) {
    mappoint_ids_.push_back(mp.value);
    frame_ids_.push_back(fr.value);
    feature_indices_.push_back(feat_idx);
    depths_.push_back(depth);
    right_feature_indices_.push_back(right_idx);
}

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_OBSERVATION_STORAGE_H
