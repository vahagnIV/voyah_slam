#ifndef VOYAHVIZ_PANGOLIN_RENDERER_H
#define VOYAHVIZ_PANGOLIN_RENDERER_H

#include <cstddef>
#include <string>
#include <voyahviz/map_viewer.h>
#include <voyahviz/scene_state.h>

namespace voyahviz {

// Private to voyahviz: owns the Pangolin window and its GL context. Every call,
// including construction and destruction, must happen on the viewer loop's
// thread (the context is bound to it).
template <typename TFeatureExtractor>
class PangolinRenderer {
public:
    // Creates the window and binds its context to the calling thread; throws
    // on failure (e.g. no display).
    explicit PangolinRenderer(const MapViewerOptions& options);
    // Destroys the window.
    ~PangolinRenderer();
    PangolinRenderer(const PangolinRenderer&) = delete;
    PangolinRenderer& operator=(const PangolinRenderer&) = delete;

    // Draws one frame; returns false once the user closed the window.
    bool Draw(const SceneState<TFeatureExtractor>& scene, size_t events_processed);

private:
    std::string window_name_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_PANGOLIN_RENDERER_H
