#ifndef VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_KANNALA_BRANDT_DISTORTION_MODEL_H
#define VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_KANNALA_BRANDT_DISTORTION_MODEL_H

#include <voyahslam/defs.h>
#include <voyahslam/geometry/distortion_models/distortion_model.h>

#include <map>
#include <vector>

namespace voyahslam::geometry::distortion_models {

class KannalaBrandtDistortionModel : public DistortionModel {
public:
  explicit KannalaBrandtDistortionModel(
      const std::vector<precision_t> &coefficients);

  KannalaBrandtDistortionModel(precision_t k1, precision_t k2,
                               precision_t k3 = 0.0, precision_t k4 = 0.0);

  KannalaBrandtDistortionModel();

  void Undistort(geometry::THomogenousPointsF &points) const override;
  void Distort(geometry::THomogenousPointsF &points) const override;
  [[nodiscard]] geometry::TMatrix22F DistortJacobian(
      const geometry::TPoint2DF &undistorted) const override;

  [[nodiscard]] const std::vector<precision_t> &GetCoefficients() const {
    return coefficients_;
  }

  void SetCoefficients(const std::vector<precision_t> &coefficients);

  void PrecomputeInitialGuessLookup(geometry::TMatrix33F &K, size_t image_width,
                                    size_t image_height);

  [[nodiscard]] bool IsLookupTablePrecomputed() const {
    return !initial_guess_lookup_.empty();
  }

  void ClearLookupTable() { initial_guess_lookup_.clear(); }

private:
  std::vector<precision_t> coefficients_;
  std::vector<precision_t> initial_guess_lookup_;
  precision_t precomputed_step_inv_{0.1};

  [[nodiscard]] precision_t ComputeRadialDistance(precision_t theta) const;
  [[nodiscard]] precision_t SolveForTheta(precision_t r_target, int max_iterations = 10,
                            precision_t tolerance = 1e-6) const;
  [[nodiscard]] precision_t ComputeRadialDistanceDerivative(precision_t theta) const;
  [[nodiscard]] precision_t GetBestGuess(precision_t r_target) const;

  static precision_t ComputeSmallestStepInPrecomputedMap(
      const std::map<precision_t, precision_t> &precomputed_lookup);

  void ComputeInitialGuess(const std::map<precision_t, precision_t> &precomputed_lookup);
};

} // namespace voyahslam::geometry::distortion_models

#endif // VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_KANNALA_BRANDT_DISTORTION_MODEL_H
