#pragma once

#include <ceres/ceres.h>

#include "uwb_localization/posegraph_backend.hpp"

namespace uwb_localization {

// Pose-graph backend on Ceres. Parameter blocks are the State::state vectors
// themselves ([x, y, z, yaw]); planar nodes keep z fixed with a
// SubsetManifold, and nodes outside the window are set constant. Factors are
// the autodiff cost functions in CostFunctions.hpp / posegraph.hpp.
class CeresBackend : public PoseGraphBackend {
public:
  CeresBackend(const PoseGraphBackendOptions& options, const rclcpp::Logger& logger)
      : options_(options), logger_(logger) {}

  std::string name() const override { return "ceres"; }
  bool optimize(PoseGraphInput& in, PoseGraphSolveStats& stats) override;

private:
  void buildProblem(ceres::Problem& problem, PoseGraphInput& in);
  bool solve(ceres::Problem& problem, PoseGraphSolveStats& stats);
  bool updateCovariances(ceres::Problem& problem, PoseGraphInput& in);

  bool addMeasurementConstraints(ceres::Problem& problem, PoseGraphInput& in, bool uav,
                                 const posegraph::VectorOfConstraints* constraints,
                                 ceres::LossFunction* loss);
  bool addEncounterConstraints(ceres::Problem& problem, PoseGraphInput& in,
                               const posegraph::VectorOfConstraints* constraints,
                               ceres::LossFunction* loss);

  PoseGraphBackendOptions options_;
  rclcpp::Logger logger_;
};

} // namespace uwb_localization
