#include <cassert>
#include <cstdint>
#include <memory>
#include <thread>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/map_follower.h>

namespace voyahslam::mapping {

template <typename TFeatureExtractor>
MapFollower<TFeatureExtractor>::MapFollower() : inbox_("MapFollower"), listener_(inbox_.GetListener()) {}

template <typename TFeatureExtractor>
MapFollower<TFeatureExtractor>::~MapFollower() = default;

template <typename TFeatureExtractor>
events::AsyncObserver& MapFollower<TFeatureExtractor>::inbox() {
    return inbox_;
}

template <typename TFeatureExtractor>
size_t MapFollower<TFeatureExtractor>::Update() {
    ClaimThread();
    size_t applied = 0;
    while (events::EventPtr event = listener_.GetNextEvent()) {
        if (event->type != events::EventType::MapTransaction) {
            continue;
        }
        const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(*event);
        if (atlas_.Apply(transaction)) {
            ++applied;
        }
    }
    num_applied_ += applied;
    return applied;
}

template <typename TFeatureExtractor>
const tracking::AtlasReplica<TFeatureExtractor>& MapFollower<TFeatureExtractor>::atlas() const {
    AssertOwner();
    return atlas_;
}

template <typename TFeatureExtractor>
std::shared_ptr<const MapSnapshot> MapFollower<TFeatureExtractor>::Snapshot() const {
    AssertOwner();
    auto snapshot = std::make_shared<MapSnapshot>();
    snapshot->active_map = atlas_.active_map();
    snapshot->num_transactions_applied = num_applied_;
    for (const map::MapId id : atlas_.map_ids()) {
        const auto* replica = atlas_.map(id);
        MapSnapshot::Map& out = snapshot->maps.emplace_back();
        out.id = id;
        out.epoch = replica->map_epoch();
        for (uint32_t index = 0; index < replica->num_points(); ++index) {
            const map::MapPointId point{index};
            if (replica->IsAlive(point)) {
                out.point_ids.push_back(point);
                out.point_positions.push_back(replica->Position(point));
            }
        }
        out.frame_ids = replica->keyframe_ids();
        out.frame_poses.reserve(out.frame_ids.size());
        for (const map::FrameId frame : out.frame_ids) {
            out.frame_poses.push_back(replica->keyframe(frame)->T_cw);
        }
    }
    return snapshot;
}

template <typename TFeatureExtractor>
size_t MapFollower<TFeatureExtractor>::num_transactions_applied() const {
    return num_applied_;
}

template <typename TFeatureExtractor>
void MapFollower<TFeatureExtractor>::ClaimThread() {
    if (owner_ == std::thread::id{}) {
        owner_ = std::this_thread::get_id();
    }
    AssertOwner();
}

template <typename TFeatureExtractor>
void MapFollower<TFeatureExtractor>::AssertOwner() const {
    assert((owner_ == std::thread::id{} || owner_ == std::this_thread::get_id()) &&
           "MapFollower: Update(), atlas() and Snapshot() belong to one thread");
}

INSTANTIATE_CLASS(MapFollower)

}  // namespace voyahslam::mapping
