#pragma once

#include "uwb_localization/posegraph_backend.hpp"

namespace uwb_localization {

// Pose-graph backend on GTSAM (batch Levenberg-Marquardt over the window).
// Builds a NonlinearFactorGraph of gtsam_factors::Pose4DoFFactor with the
// same factors, noise and robust loss as CeresBackend; see gtsam_factors.hpp.
//
// Keys: Symbol('a', id) AGV nodes, Symbol('u', id) UAV nodes,
//       Symbol('A', 0) AGV anchor, Symbol('U', 0) UAV anchor.
class GtsamBackend : public PoseGraphBackend {
public:
  GtsamBackend(const PoseGraphBackendOptions& options, const rclcpp::Logger& logger)
      : options_(options), logger_(logger) {}

  std::string name() const override { return "gtsam"; }
  bool optimize(PoseGraphInput& in, PoseGraphSolveStats& stats) override;

private:
  PoseGraphBackendOptions options_;
  rclcpp::Logger logger_;
};

} // namespace uwb_localization
