#ifndef VOYAHVIZ_PANGOLIN_RENDERER_H
#define VOYAHVIZ_PANGOLIN_RENDERER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <pangolin/display/view.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/handler/handler.h>
#include <pangolin/var/var.h>
#include <string>
#include <vector>
#include <voyahviz/map_viewer.h>
#include <voyahviz/scene_state.h>

namespace voyahviz {

// On a Linux Wayland session with PANGOLIN_WINDOW_URI unset, sets it to
// "x11://" (Pangolin's Wayland backend stalls; see the definition). Calls
// setenv, so it must run before other threads start: MapViewer's constructor
// calls it when show_window is set.
void PreferX11BackendOnWayland();

// Private to voyahviz: owns the Pangolin window and its GL context. Every call,
// including construction and destruction, must happen on the viewer loop's
// thread (the context is bound to it).
//
// Draws alive map points (tracked green, visible-unmatched blue, others grey),
// keyframe frustums (local-map ones orange), the current camera and its
// trajectory, with a side panel of display toggles and scene statistics.
// Pangolin vars are global by name: a renderer recreated after Stop()/Start()
// re-attaches to the same vars, so the toggles keep their last values.
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
    void UpdatePanel(const SceneState<TFeatureExtractor>& scene, size_t events_processed);
    // Applies "Follow camera" to the render state (before drawing).
    void UpdateFollow(const SceneState<TFeatureExtractor>& scene);

    std::string window_name_;
    double camera_size_;
    pangolin::OpenGlMatrix initial_model_view_;
    pangolin::OpenGlRenderState render_state_;
    // The map view points at it; the destructor body destroys the window (and
    // with it the view) before members are destroyed.
    std::unique_ptr<pangolin::Handler3D> handler_;
    pangolin::View* map_view_ = nullptr;            // owned by the window's context

    pangolin::Var<bool> show_points_;
    pangolin::Var<bool> show_keyframes_;
    pangolin::Var<bool> show_trajectory_;
    pangolin::Var<bool> follow_camera_;
    pangolin::Var<std::string> map_text_;
    pangolin::Var<std::string> points_text_;
    pangolin::Var<std::string> keyframes_text_;
    pangolin::Var<std::string> status_text_;
    pangolin::Var<std::string> events_text_;
    bool following_ = false;

    // Scratch, reused every frame: how each point is drawn.
    enum class PointClass : uint8_t { Other, Visible, Tracked };
    std::vector<PointClass> point_class_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_PANGOLIN_RENDERER_H
