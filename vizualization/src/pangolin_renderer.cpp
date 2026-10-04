#include "pangolin_renderer.h"

#include <pangolin/display/display.h>
#include <pangolin/gl/gl.h>
#include <voyahslam/features/instantiation.h>

namespace voyahviz {

template <typename TFeatureExtractor>
PangolinRenderer<TFeatureExtractor>::PangolinRenderer(const MapViewerOptions& options)
    : window_name_("voyahslam map") {
    pangolin::CreateWindowAndBind(window_name_, options.width, options.height);
}

template <typename TFeatureExtractor>
PangolinRenderer<TFeatureExtractor>::~PangolinRenderer() {
    pangolin::DestroyWindow(window_name_);
}

template <typename TFeatureExtractor>
bool PangolinRenderer<TFeatureExtractor>::Draw(const SceneState<TFeatureExtractor>& /*scene*/,
                                               size_t /*events_processed*/) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();
    return !pangolin::ShouldQuit();
}

INSTANTIATE_CLASS(PangolinRenderer)

} // namespace voyahviz
