#ifndef VOYAHSLAM_MAPPING_MAP_FOLLOWER_H
#define VOYAHSLAM_MAPPING_MAP_FOLLOWER_H

#include <cassert>
#include <cstddef>
#include <memory>
#include <thread>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/mapping/map_snapshot.h>
#include <voyahslam/tracking/atlas_replica.h>

namespace voyahslam::mapping {

// The supported way for an app to read the map from its own thread (D-13): an
// inbox plus a private AtlasReplica that the app brings up to date by calling
// Update() on its own thread. Nothing here writes the Mapper's map or shares
// mutable storage with another replica; the Mapper stays the only writer.
//
// Wiring: subscribe the inbox to the Mapper before the Mapper starts,
//   events::Subscription s = slam.mapper().AddObserver(follower.inbox());
// Because connections change only while the producer is stopped (D-05), a
// follower sees every transaction from the first one on; it cannot join a
// running Slam. Declare the Subscription before the Slam it connects to, and
// the follower before the Subscription (the inbox must outlive the wire).
//
// Threads: Update(), atlas() and Snapshot() belong to one thread, the one that
// makes the first Update() (checked by assert in debug builds). Snapshot()
// returns immutable plain data that any thread may hold and read.
//
// Cost: a full AtlasReplica per follower (descriptors, BoW, covisibility) and
// an O(map) copy per Snapshot(). A follower that is never updated lets its
// inbox grow; the "consumer is behind" warning names it "MapFollower" (D-24).
template <typename TFeatureExtractor>
class MapFollower {
public:
    MapFollower() : inbox_("MapFollower") {}
    MapFollower(const MapFollower&) = delete;
    MapFollower& operator=(const MapFollower&) = delete;

    // Subscribe before the Mapper starts: Subscription s = mapper.AddObserver(follower.inbox());
    [[nodiscard]] events::AsyncObserver& inbox() { return inbox_; }

    // Applies every pending MapTransaction on the calling thread, in order;
    // other events are released. Returns the number of transactions applied by
    // this call (those the replica ignores, such as a removed map's, are not
    // counted).
    size_t Update() {
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

    // The follower's own Atlas; same thread as Update().
    [[nodiscard]] const tracking::AtlasReplica<TFeatureExtractor>& atlas() const {
        AssertOwner();
        return atlas_;
    }

    // A copy of the Atlas as it is now; same thread as Update(), but the result
    // is immutable and any thread may hold it. O(map).
    [[nodiscard]] std::shared_ptr<const MapSnapshot> Snapshot() const {
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

    // Transactions applied so far; same thread as Update().
    [[nodiscard]] size_t num_transactions_applied() const { return num_applied_; }

private:
    void ClaimThread() {
        if (owner_ == std::thread::id{}) {
            owner_ = std::this_thread::get_id();
        }
        AssertOwner();
    }
    void AssertOwner() const {
        assert((owner_ == std::thread::id{} || owner_ == std::this_thread::get_id()) &&
               "MapFollower: Update(), atlas() and Snapshot() belong to one thread");
    }

    // The inbox first: the listener (and the events it holds) is destroyed before it.
    events::AsyncObserver inbox_;
    events::Listener listener_ = inbox_.GetListener();
    tracking::AtlasReplica<TFeatureExtractor> atlas_;
    size_t num_applied_ = 0;
    std::thread::id owner_;  // the thread of the first Update(); empty before
};

}  // namespace voyahslam::mapping

#endif  // VOYAHSLAM_MAPPING_MAP_FOLLOWER_H
