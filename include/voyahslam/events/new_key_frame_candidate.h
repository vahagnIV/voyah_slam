#ifndef VOYAHSLAM_EVENTS_NEW_KEY_FRAME_CANDIDATE_H
#define VOYAHSLAM_EVENTS_NEW_KEY_FRAME_CANDIDATE_H

#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

template <typename TFeatureExtractor, typename TMeasurement>
struct NewKeyFrameCandidate
    : Event
    , memory::Recyclable<NewKeyFrameCandidate<TFeatureExtractor, TMeasurement>, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;
    TMeasurement measurement;  // carries the timestamp

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

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_NEW_KEY_FRAME_CANDIDATE_H
