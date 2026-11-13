#pragma once

// Solver-independent interface to the pose-graph optimizer.
//
// PoseOptimizationNode builds the graph inputs (nodes, priors and constraints)
// and hands them to a backend, which builds its own problem, solves it and
// writes the result back into the states. Two backends implement it:
//   - CeresBackend (ceres_backend.hpp): ceres::Problem + trust-region solver.
//   - GtsamBackend (gtsam_backend.hpp): gtsam::NonlinearFactorGraph + LM.
//     Only built when GTSAM >= 4.3 is found (UWB_LOCALIZATION_WITH_GTSAM).
// Both solve exactly the same model (a keyframe sliding window whose older
// nodes are held constant); see the backends for the details.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "uwb_localization/posegraph.hpp"

namespace uwb_localization {

// Everything a backend needs for one optimization. States are read as the
// initial guess and, after the solve, the free ones are overwritten with the
// result (and their covariances updated).
//
// Sliding window: a node is free while posegraph::isNodeFixedKF(...) is false
// for it, otherwise it is held constant. Anchors are free only when at least
// one encounter constraint is used (otherwise nothing observes them); one of
// them (fix_uav_anchor) then gets a very strong prior, which removes the gauge
// freedom of the common frame.
struct PoseGraphInput {
  posegraph::MapOfStates* agv_map = nullptr;
  posegraph::MapOfStates* uav_map = nullptr;
  State* anchor_agv = nullptr;   // AGV odometry frame in the common (map) frame; planar
  State* anchor_uav = nullptr;   // UAV odometry frame in the common (map) frame
  int agv_current_id = 0, uav_current_id = 0;
  int max_keyframes = 0, min_keyframes = 0;

  // Priors: on each robot's first node (only while it is still free) and on
  // the fixed anchor (only when the anchors are free): the AGV anchor, or the
  // UAV anchor if fix_uav_anchor.
  const posegraph::PriorConstraint* prior_agv = nullptr;
  const posegraph::PriorConstraint* prior_uav = nullptr;
  const posegraph::PriorConstraint* prior_anchor = nullptr;
  bool fix_uav_anchor = false;

  State* fixedAnchor() const { return fix_uav_anchor ? anchor_uav : anchor_agv; }

  // Intra-robot constraints: id_begin/id_end are nodes of the same robot.
  const posegraph::VectorOfConstraints* odom_agv = nullptr;
  const posegraph::VectorOfConstraints* odom_uav = nullptr;
  const posegraph::VectorOfConstraints* radar_agv = nullptr;
  const posegraph::VectorOfConstraints* radar_uav = nullptr;

  // Inter-robot constraints, through the anchors: id_begin is an AGV node,
  // id_end a UAV node (see posegraph::AnchorResidual / Anchor factor).
  const posegraph::VectorOfConstraints* encounters_uwb = nullptr;
  const posegraph::VectorOfConstraints* encounters_radar = nullptr;

  bool isFree(bool uav, int id) const {
    return !posegraph::isNodeFixedKF(uav ? uav_current_id : agv_current_id, id, max_keyframes, min_keyframes);
  }
};

struct PoseGraphSolveStats {
  bool converged = false;
  bool covariance_ok = false;
  int iterations = 0;
  int num_factors = 0;
  int num_variables = 0;          // free nodes + free anchors
  double initial_cost = 0.0;      // 0.5 * sum of robust squared residuals (same definition in both)
  double final_cost = 0.0;
  double build_time = 0.0;        // s, building the backend's problem
  double solve_time = 0.0;        // s, the nonlinear solve
  double covariance_time = 0.0;   // s, node covariances
};

struct PoseGraphBackendOptions {
  double loss_threshold = 2.5;    // Cauchy loss scale, in sigmas (UWB and radar factors; odometry has none)
  int max_iterations = 100;
  double function_tolerance = 1e-6;  // stop when the relative cost decrease is below this
  int num_threads = 12;           // Ceres only (GTSAM uses TBB if it was built with it)
};

class PoseGraphBackend {
public:
  virtual ~PoseGraphBackend() = default;
  virtual std::string name() const = 0;

  // Builds and solves the graph described by `in`. Free states are written
  // back whenever the solver returns a solution; covariances only on
  // convergence. Returns true on convergence.
  virtual bool optimize(PoseGraphInput& in, PoseGraphSolveStats& stats) = 0;
};

// "ceres" or "gtsam". Throws std::invalid_argument for an unknown name, or
// for "gtsam" when the package was built without GTSAM.
std::unique_ptr<PoseGraphBackend> makePoseGraphBackend(const std::string& name,
                                                        const PoseGraphBackendOptions& options,
                                                        const rclcpp::Logger& logger);

} // namespace uwb_localization
