#include <algorithm>
#include <chrono>
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
    const bool was_running = worker_.running();
    worker_.Stop();
    if (was_running && query_stats_.detected > 0) {  // once per run
        log_->info("Loop detection: {} queries, {:.1f} ms mean, {:.1f} ms max; {} queries and {} loops dropped",
                   query_stats_.detected,
                   1e3 * query_stats_.total_seconds / static_cast<double>(query_stats_.detected),
                   1e3 * query_stats_.max_seconds, query_stats_.dropped, num_dropped_loops_);
    }
}

template <typename TFeatureExtractor>
bool LoopCloser<TFeatureExtractor>::ProcessPending() {
    // Every event is applied on receipt and so released at once: however far
    // detection lags, it holds none of the Mapper's pooled transactions. Only
    // the queries (Frame ids) wait.
    bool processed = false;
    while (const events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;  // acknowledged by the next GetNextEvent()
        if (vocabulary_ && options_.enabled) {
            ApplyEvent(*event);
        }
    }
    if (queries_.size() > options_.max_query_backlog) {
        const size_t dropped = queries_.size() - options_.max_query_backlog;
        if (!dropping_queries_) {  // once until the backlog empties
            log_->warn("Loop detection behind: queries beyond {} dropped, from Frame {} ({} so far)",
                       options_.max_query_backlog, queries_.front().value, query_stats_.dropped + dropped);
        }
        dropping_queries_ = true;
        queries_.erase(queries_.begin(), queries_.begin() + static_cast<std::ptrdiff_t>(dropped));
        query_stats_.dropped += dropped;
    }
    if (queries_.empty()) {
        dropping_queries_ = false;
        return processed;
    }
    // One query per call: the inbox is drained again before the next, and
    // Stop() waits for this one only.
    const map::FrameId query = queries_.front();
    queries_.pop_front();
    DetectQuery(query);
    return true;
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
        queries_.clear();
    } else if (transaction.map_epoch > replica_.map_epoch()) {
        pending_.reset();  // a loop correction: its Sim3 is in the old coordinates
    }
    replica_.Apply(event);
    for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {
        if (replica_.keyframe(added.id) != nullptr) {
            queries_.push_back(added.id);
        }
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::ClearMap(map::MapId lost_map) {
    replica_ = {};
    pending_.reset();
    queries_.clear();
    lost_map_ = lost_map;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectQuery(map::FrameId query) {
    // A query culled while it waited is skipped; so is one of a small map.
    if (replica_.keyframe(query) == nullptr || replica_.num_keyframes() < options_.min_map_frames) {
        return;
    }
    if (query_hook_) {
        query_hook_(query);
    }
    const auto start = std::chrono::steady_clock::now();
    DetectLoop(query);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    ++query_stats_.detected;
    query_stats_.total_seconds += seconds;
    query_stats_.max_seconds = std::max(query_stats_.max_seconds, seconds);
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectLoop(map::FrameId query) {
    // A pending region is first checked against this query, from the last
    // confirming query's Sim3 moved by their relative pose.
    if (pending_ && pending_->num_coincidences > 0) {
        const auto* last = replica_.keyframe(pending_->last_query);
        const auto* current = replica_.keyframe(query);
        std::optional<CommonRegion> confirmed;
        if (last != nullptr && current != nullptr) {
            const geometry::Sim3 S_query_world =
                geometry::Compose(ToSim3(current->T_cw * last->T_cw.Inverse()), pending_->S_last_world);
            CommonRegionStats refine_stats;
            confirmed =
                RefineCommonRegion(replica_, query, pending_->match_frame, S_query_world, options_, &refine_stats);
            LogNormalRejections(refine_stats);
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
    LogNormalRejections(result.stats);
    if (!result.region) {
        return;
    }
    const CommonRegion& region = *result.region;
    log_->debug("Frame {} shares a region with Frame {}: {} matches, {} confirmations, {:.2f} of {} normals disagree",
                query.value, region.match_frame.value, region.num_matches, region.num_coincidences,
                region.normals.disagreement, region.normals.pairs);
    pending_ = PendingRegion{region.match_frame, query,           region.S_query_world, region.matches,
                             region.num_coincidences, 0};
    if (pending_->num_coincidences >= options_.required_coincidences) {
        PublishLoop(query, *pending_);
        pending_.reset();
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::LogNormalRejections(const CommonRegionStats& stats) {
    for (const NormalRejection& rejection : stats.normal_rejections) {
        log_->debug("Normal gate: Frame {} against the region of Frame {} rejected ({:.2f} of {} pairs disagree)",
                    rejection.frame.value, rejection.match_frame.value, rejection.agreement.disagreement,
                    rejection.agreement.pairs);
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
    // Urgent: the Mapper's running local BA stops for the correction. A loop
    // found again before that correction reaches this replica (no new epoch
    // since the last loop published) is a repeat the Mapper drops: it waits
    // for the local BA.
    const std::pair<map::MapId, uint32_t> epoch{replica_.map_id(), replica_.map_epoch()};
    event->priority = last_loop_epoch_ != epoch;
    last_loop_epoch_ = epoch;
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
