#include "pangolin_renderer.h"

#include <cstdlib>
#include <pangolin/display/display.h>
#include <pangolin/display/widgets.h>
#include <pangolin/gl/gl.h>
#include <voyahslam/features/instantiation.h>
#include <voyahviz/camera_frustum.h>

namespace voyahviz {

namespace {

constexpr int kPanelWidthPx = 175;

pangolin::OpenGlMatrix ToOpenGl(const voyahslam::geometry::RigidTransformation& transformation) {
    return pangolin::OpenGlMatrix(transformation.ToMatrix44());
}

void Vertex(const voyahslam::geometry::TPoint3DF& point) {
    glVertex3d(point.x(), point.y(), point.z());
}

// The 4 centre->corner edges and the far rectangle, as GL_LINES.
void DrawFrustum(const voyahslam::geometry::RigidTransformation& T_cw, double size) {
    const CameraFrustum frustum = FrustumInWorld(T_cw, size);
    glBegin(GL_LINES);
    for (size_t i = 0; i < frustum.corners.size(); ++i) {
        Vertex(frustum.centre);
        Vertex(frustum.corners[i]);
        Vertex(frustum.corners[i]);
        Vertex(frustum.corners[(i + 1) % frustum.corners.size()]);
    }
    glEnd();
}

const char* StatusName(TrackingStatus status) {
    switch (status) {
        case TrackingStatus::Tracking:
            return "tracking";
        case TrackingStatus::Lost:
            return "lost";
        case TrackingStatus::None:
            break;
    }
    return "none";
}

} // namespace

void PreferX11BackendOnWayland() {
#if defined(__linux__)
    // Pangolin's Wayland backend (its default here) blocks in eglSwapBuffers on
    // GNOME after the first frame, which would stall the viewer's inbox drain and
    // with it the producers' event pools. On a Wayland session prefer X11
    // (XWayland) unless the user chose a backend; if X11 is unavailable the
    // renderer's constructor throws and the viewer runs headless.
    if (std::getenv("WAYLAND_DISPLAY") != nullptr && std::getenv("PANGOLIN_WINDOW_URI") == nullptr) {
        setenv("PANGOLIN_WINDOW_URI", "x11://", /*overwrite=*/0);
    }
#endif
}

template <typename TFeatureExtractor>
PangolinRenderer<TFeatureExtractor>::PangolinRenderer(const MapViewerOptions& options)
    : window_name_("voyahslam map"),
      camera_size_(options.camera_size),
      // Behind and above the first camera, up = -y (the world is camera 1: y down).
      initial_model_view_(pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)),
      render_state_(pangolin::ProjectionMatrix(options.width, options.height, 500, 500,
                                               options.width / 2.0, options.height / 2.0, 0.01, 1000),
                    initial_model_view_),
      handler_(std::make_unique<pangolin::Handler3D>(render_state_)),
      show_points_("menu.Show points", true, true),
      show_keyframes_("menu.Show keyframes", true, true),
      show_trajectory_("menu.Show trajectory", true, true),
      follow_camera_("menu.Follow camera", false, true),
      map_text_("menu.Map", "", pangolin::META_FLAG_READONLY),
      points_text_("menu.Points", "", pangolin::META_FLAG_READONLY),
      keyframes_text_("menu.Keyframes", "", pangolin::META_FLAG_READONLY),
      status_text_("menu.Status", "", pangolin::META_FLAG_READONLY),
      events_text_("menu.Events", "", pangolin::META_FLAG_READONLY) {
    pangolin::CreateWindowAndBind(window_name_, options.width, options.height);
    glEnable(GL_DEPTH_TEST);
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(kPanelWidthPx));
    map_view_ = &pangolin::CreateDisplay()
                     .SetBounds(0.0, 1.0, pangolin::Attach::Pix(kPanelWidthPx), 1.0,
                                -static_cast<double>(options.width) / options.height)
                     .SetHandler(handler_.get());
}

template <typename TFeatureExtractor>
PangolinRenderer<TFeatureExtractor>::~PangolinRenderer() {
    pangolin::DestroyWindow(window_name_);
}

template <typename TFeatureExtractor>
bool PangolinRenderer<TFeatureExtractor>::Draw(const SceneState<TFeatureExtractor>& scene,
                                               size_t events_processed) {
    UpdatePanel(scene, events_processed);
    UpdateFollow(scene);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    map_view_->Activate(render_state_);

    if (show_points_) {
        const auto& positions = scene.point_positions();
        const auto& alive = scene.point_alive();
        glPointSize(2);
        glColor3f(0.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < positions.size(); ++i) {
            if (alive[i]) Vertex(positions[i]);
        }
        glEnd();
    }

    glLineWidth(1);
    if (show_keyframes_) {
        glColor3f(0.0f, 0.0f, 1.0f);
        for (const auto& [id, T_cw] : scene.keyframes()) {
            DrawFrustum(T_cw, camera_size_);
        }
    }

    if (show_trajectory_ && scene.trajectory().size() > 1) {
        glColor3f(0.5f, 0.5f, 0.5f);
        glBegin(GL_LINE_STRIP);
        for (const auto& centre : scene.trajectory()) Vertex(centre);
        glEnd();
    }

    if (scene.current_pose()) {
        if (scene.status() == TrackingStatus::Lost) {
            glColor3f(1.0f, 0.0f, 0.0f);
        } else {
            glColor3f(0.0f, 0.8f, 0.0f);
        }
        glLineWidth(2);
        DrawFrustum(*scene.current_pose(), camera_size_);
        glLineWidth(1);
    }

    // Leave GL state as Pangolin's widgets expect it.
    glPointSize(1);
    glLineWidth(1);
    glColor3f(1.0f, 1.0f, 1.0f);
    pangolin::FinishFrame();
    return !pangolin::ShouldQuit();
}

template <typename TFeatureExtractor>
void PangolinRenderer<TFeatureExtractor>::UpdatePanel(const SceneState<TFeatureExtractor>& scene,
                                                      size_t events_processed) {
    map_text_ = std::to_string(scene.current_map_id().value);
    points_text_ = std::to_string(scene.num_alive_points());
    keyframes_text_ = std::to_string(scene.keyframes().size());
    status_text_ = StatusName(scene.status());
    events_text_ = std::to_string(events_processed);
}

template <typename TFeatureExtractor>
void PangolinRenderer<TFeatureExtractor>::UpdateFollow(const SceneState<TFeatureExtractor>& scene) {
    if (!follow_camera_) {
        if (following_) {
            // Unfollow keeps the view where it is on screen.
            render_state_.Unfollow();
            following_ = false;
        }
        return;
    }
    const voyahslam::geometry::RigidTransformation* T_cw = nullptr;
    if (scene.current_pose()) {
        T_cw = &*scene.current_pose();
    } else if (!scene.keyframes().empty()) {
        T_cw = &scene.keyframes().rbegin()->second;
    }
    if (T_cw == nullptr) return;
    render_state_.Follow(ToOpenGl(T_cw->Inverse()));
    if (!following_) {
        // Switching on: the model-view becomes relative to the camera; start it
        // from the initial look-at (behind and above the camera).
        render_state_.SetModelViewMatrix(initial_model_view_);
        following_ = true;
    }
}

INSTANTIATE_CLASS(PangolinRenderer)

} // namespace voyahviz
