#include <optional>
#include <utility>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/loop_closing/loop_closer.h>

#include "loop_closing/common_region_internal.h"

namespace voyahslam::loop_closing {

using detail::FixesScale;
using detail::IsValid;
using detail::PointMatches;
using detail::PointsOf;
using detail::ToSim3;

template <typename TFeatureExtractor>
LoopCloser<TFeatureExtractor>::~LoopCloser() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Stop() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
bool LoopCloser<TFeatureExtractor>::ProcessPending() {
    bool processed = false;
    while (const events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;  // acknowledged by the next GetNextEvent()
        if (vocabulary_) {
            ApplyEvent(*event);
        }
    }
    return processed;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    if (event.map_id.value == 0 || event.map_id <= lost_map_) {
        return;  // map 0 is exempt; a lost map's late events are stale
    }
    if (event.type == events::EventType::TrackingLost) {
        if (static_cast<const events::TrackingLost&>(event).final && event.map_id == replica_.map_id()) {
            ClearMap(event.map_id);
        }
        return;
    }
    if (event.type != events::EventType::MapTransaction) {
        return;
    }
    const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(event);
    if (event.map_id < replica_.map_id()) {
        return;  // an older map's
    }
    if (transaction.new_map || event.map_id > replica_.map_id()) {
        pending_.reset();  // the replica clears itself too
    }
    replica_.Apply(event);
    for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {
        if (replica_.keyframe(added.id) != nullptr) {
            DetectLoop(added.id);
        }
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::ClearMap(map::MapId lost_map) {
    replica_ = {};
    pending_.reset();
    lost_map_ = lost_map;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectLoop(map::FrameId query) {
    if (replica_.num_keyframes() < options_.min_map_frames) {
        return;
    }
    // A pending region is first checked against this query, from the last
    // confirming query's Sim3 moved by their relative pose.
    if (pending_ && pending_->num_coincidences > 0) {
        const auto* last = replica_.keyframe(pending_->last_query);
        const auto* current = replica_.keyframe(query);
        std::optional<CommonRegion> confirmed;
        if (last != nullptr && current != nullptr) {
            const geometry::Sim3 S_query_world =
                geometry::Compose(ToSim3(current->T_cw * last->T_cw.Inverse()), pending_->S_last_world);
            confirmed = RefineCommonRegion(replica_, query, pending_->match_frame, S_query_world, options_);
        }
        if (confirmed) {
            ++pending_->num_coincidences;
            pending_->last_query = query;
            pending_->S_last_world = confirmed->S_query_world;
            pending_->matches = std::move(confirmed->matches);
            pending_->num_not_found = 0;
            log_->debug("Loop region of Frame {} confirmed by Frame {} ({}/{})", pending_->match_frame.value,
                        query.value, pending_->num_coincidences, options_.required_coincidences);
            if (pending_->num_coincidences >= options_.required_coincidences) {
                PublishLoop(query, *pending_);
                pending_.reset();
            }
            return;
        }
        if (last == nullptr || ++pending_->num_not_found >= options_.max_unconfirmed_queries) {
            pending_.reset();
        }
    }

    const CommonRegionResult result = DetectCommonRegion(replica_, query, options_);
    last_stats_ = result.stats;
    if (!result.region) {
        return;
    }
    const CommonRegion& region = *result.region;
    log_->debug("Frame {} shares a region with Frame {}: {} matches, {} confirmations", query.value,
                region.match_frame.value, region.num_matches, region.num_coincidences);
    pending_ = PendingRegion{region.match_frame, query,           region.S_query_world, region.matches,
                             region.num_coincidences, 0};
    if (pending_->num_coincidences >= options_.required_coincidences) {
        PublishLoop(query, *pending_);
        pending_.reset();
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::PublishLoop(map::FrameId query, const PendingRegion& region) {
    const auto* query_keyframe = replica_.keyframe(query);
    const auto* match_keyframe = replica_.keyframe(region.match_frame);
    if (query_keyframe == nullptr || match_keyframe == nullptr) {
        return;
    }
    geometry::Sim3 S_query_match = geometry::Compose(region.S_last_world, ToSim3(match_keyframe->T_cw.Inverse()));
    if (FixesScale(*query_keyframe)) {
        S_query_match.scale = 1.0;
    }
    if (!IsValid(S_query_match)) {
        return;
    }
    auto* event = loop_detected_pool_.TryCreate();
    if (event == nullptr) {
        ++num_dropped_loops_;
        log_->warn("Loop of Frame {} with Frame {} dropped: {} LoopDetected events in flight", query.value,
                   region.match_frame.value, options_.max_events_in_flight);
        return;
    }
    event->map_id = replica_.map_id();
    event->query_map_id = replica_.map_id();
    event->match_map_id = replica_.map_id();
    event->query_frame = query;
    event->match_frame = region.match_frame;
    event->S_query_match = S_query_match;
    const PointMatches query_points = PointsOf(replica_, *query_keyframe);
    for (size_t i = 0; i < region.matches.size() && i < query_points.size(); ++i) {
        if (region.matches[i] && query_points[i] && *region.matches[i] != *query_points[i]) {
            event->matched_points.emplace_back(*query_points[i], *region.matches[i]);
        }
    }
    log_->info("Loop detected: Frame {} with Frame {} (scale {:.3f}, {} matched points)", query.value,
               region.match_frame.value, S_query_match.scale, event->matched_points.size());
    observable_.Notify(event);
}

INSTANTIATE_CLASS(LoopCloser)

} // namespace voyahslam::loop_closing
