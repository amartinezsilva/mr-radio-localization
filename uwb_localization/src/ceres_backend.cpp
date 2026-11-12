#include "uwb_localization/ceres_backend.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include "uwb_localization/CostFunctions.hpp"

namespace uwb_localization {

namespace {
double secondsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

bool CeresBackend::optimize(PoseGraphInput& in, PoseGraphSolveStats& stats) {
  stats = PoseGraphSolveStats{};

  auto t0 = std::chrono::steady_clock::now();
  ceres::Problem problem;
  buildProblem(problem, in);
  stats.build_time = secondsSince(t0);
  stats.num_factors = problem.NumResidualBlocks();
  std::vector<double*> blocks;
  problem.GetParameterBlocks(&blocks);
  stats.num_variables = static_cast<int>(std::count_if(blocks.begin(), blocks.end(),
      [&](double* b) { return !problem.IsParameterBlockConstant(b); }));

  // Ceres writes the solution into the State vectors whatever the outcome.
  stats.converged = solve(problem, stats);
  if (!stats.converged) return false;

  t0 = std::chrono::steady_clock::now();
  stats.covariance_ok = updateCovariances(problem, in);
  stats.covariance_time = secondsSince(t0);
  return true;
}

// Intra-robot factors (odometry, radar) between nodes of one robot.
bool CeresBackend::addMeasurementConstraints(ceres::Problem& problem, PoseGraphInput& in, bool uav,
                                             const posegraph::VectorOfConstraints* constraints,
                                             ceres::LossFunction* loss) {
  if (!constraints) return false;
  posegraph::MapOfStates& map = uav ? *in.uav_map : *in.agv_map;
  bool constraint_available = false;

  for (const auto& constraint : *constraints) {
    if (!in.isFree(uav, constraint.id_begin) && !in.isFree(uav, constraint.id_end)) continue;
    constraint_available = true;

    State& state_i = map[constraint.id_begin];
    State& state_j = map[constraint.id_end];
    ceres::CostFunction* cost = MeasurementResidual::Create(constraint.t_T_s, constraint.covariance,
                                                            state_i.roll, state_i.pitch,
                                                            state_j.roll, state_j.pitch);
    problem.AddResidualBlock(cost, loss, state_i.state.data(), state_j.state.data());
  }
  return constraint_available;
}

// Inter-robot (encounter) factors: AGV node -> UAV node through both anchors.
bool CeresBackend::addEncounterConstraints(ceres::Problem& problem, PoseGraphInput& in,
                                           const posegraph::VectorOfConstraints* constraints,
                                           ceres::LossFunction* loss) {
  if (!constraints) return false;
  bool constraint_available = false;

  for (const auto& constraint : *constraints) {
    if (!in.isFree(false, constraint.id_begin) && !in.isFree(true, constraint.id_end)) continue;
    constraint_available = true;

    State& state_i = (*in.agv_map)[constraint.id_begin];
    State& state_j = (*in.uav_map)[constraint.id_end];
    ceres::CostFunction* cost = AnchorResidual::Create(constraint.t_T_s, constraint.covariance,
                                                       state_i.roll, state_i.pitch,
                                                       state_j.roll, state_j.pitch,
                                                       in.anchor_uav->roll, in.anchor_uav->pitch,
                                                       in.anchor_agv->roll, in.anchor_agv->pitch);
    problem.AddResidualBlock(cost, loss, state_i.state.data(), state_j.state.data(),
                             in.anchor_uav->state.data(), in.anchor_agv->state.data());
  }
  return constraint_available;
}

void CeresBackend::buildProblem(ceres::Problem& problem, PoseGraphInput& in) {
  // Planar nodes keep z fixed (index 2 of [x, y, z, yaw]); the rest are
  // plain 4-DoF blocks. The problem takes ownership of the manifold.
  ceres::Manifold* planar_manifold = new ceres::SubsetManifold(4, {2});

  // Residuals beyond loss_threshold sigmas are treated as outliers.
  ceres::LossFunction* robust_loss = new ceres::CauchyLoss(options_.loss_threshold);

  // Anchor nodes: place each robot's odometry frame in the common frame.
  problem.AddParameterBlock(in.anchor_uav->state.data(), 4);
  problem.AddParameterBlock(in.anchor_agv->state.data(), 4, planar_manifold);

  // Robot nodes. Nodes outside the sliding window (and all of them until the
  // window has min_keyframes nodes) are held constant.
  auto add_nodes = [&](posegraph::MapOfStates& map, bool uav) {
    for (auto& kv : map) {
      State& state = kv.second;
      if (state.planar) problem.AddParameterBlock(state.state.data(), 4, planar_manifold);
      else problem.AddParameterBlock(state.state.data(), 4);
      if (!in.isFree(uav, kv.first)) problem.SetParameterBlockConstant(state.state.data());
    }
  };
  add_nodes(*in.agv_map, false);
  add_nodes(*in.uav_map, true);

  // Priors on the first node of each robot while it is still free. Bind by
  // reference: the prior must act on the parameter block stored in the map.
  if (in.isFree(false, 0)) {
    auto& first_node_agv = (*in.agv_map)[0];
    problem.AddResidualBlock(PriorCostFunction::Create(in.prior_agv->pose, first_node_agv.roll, first_node_agv.pitch,
                                                       in.prior_agv->covariance),
                             nullptr, first_node_agv.state.data());
  }
  if (in.isFree(true, 0)) {
    auto& first_node_uav = (*in.uav_map)[0];
    problem.AddResidualBlock(PriorCostFunction::Create(in.prior_uav->pose, first_node_uav.roll, first_node_uav.pitch,
                                                       in.prior_uav->covariance),
                             nullptr, first_node_uav.state.data());
  }

  // Intra-robot factors: odometry (proprioceptive) and radar (extraceptive).
  addMeasurementConstraints(problem, in, false, in.odom_agv, robust_loss);
  addMeasurementConstraints(problem, in, true, in.odom_uav, robust_loss);
  addMeasurementConstraints(problem, in, false, in.radar_agv, robust_loss);
  addMeasurementConstraints(problem, in, true, in.radar_uav, robust_loss);

  // Inter-robot (encounter) factors.
  const bool encounter_uwb_available = addEncounterConstraints(problem, in, in.encounters_uwb, robust_loss);
  const bool encounter_radar_available = addEncounterConstraints(problem, in, in.encounters_radar, robust_loss);

  if (!encounter_uwb_available && !encounter_radar_available) {
    // Nothing observes the anchors: hold them constant.
    problem.SetParameterBlockConstant(in.anchor_uav->state.data());
    problem.SetParameterBlockConstant(in.anchor_agv->state.data());
  } else {
    // Fix one anchor with a strong prior (removes the gauge freedom of the common frame).
    State* fixed = in.fixedAnchor();
    problem.AddResidualBlock(PriorCostFunction::Create(in.prior_anchor->pose, fixed->roll, fixed->pitch,
                                                       in.prior_anchor->covariance),
                             nullptr, fixed->state.data());
  }
}

bool CeresBackend::solve(ceres::Problem& problem, PoseGraphSolveStats& stats) {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.num_threads = options_.num_threads;
  options.use_nonmonotonic_steps = true;  // Help escape plateaus
  options.max_num_iterations = options_.max_iterations;
  options.function_tolerance = options_.function_tolerance;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  RCLCPP_INFO(logger_, "%s", summary.BriefReport().c_str());

  stats.solve_time = summary.total_time_in_seconds;
  stats.iterations = static_cast<int>(summary.iterations.size()) - 1;  // iteration 0 is the initial state
  stats.initial_cost = summary.initial_cost;
  stats.final_cost = summary.final_cost;
  return summary.termination_type == ceres::CONVERGENCE;
}

// Covariance of every optimized (non-constant) node and anchor. Nodes held
// constant keep their previous covariance. Returns false (and sets identity
// covariances) if the computation fails.
bool CeresBackend::updateCovariances(ceres::Problem& problem, PoseGraphInput& in) {
  std::vector<State*> optimized;
  for (auto& kv : *in.agv_map) optimized.push_back(&kv.second);
  for (auto& kv : *in.uav_map) optimized.push_back(&kv.second);
  optimized.push_back(in.anchor_agv);
  optimized.push_back(in.anchor_uav);
  optimized.erase(std::remove_if(optimized.begin(), optimized.end(), [&](State* s) {
    return problem.IsParameterBlockConstant(s->state.data());
  }), optimized.end());

  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  for (State* s : optimized) covariance_blocks.emplace_back(s->state.data(), s->state.data());

  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);

  if (!covariance.Compute(covariance_blocks, &problem)) {
    RCLCPP_WARN(logger_, "Failed to compute covariances.");
    for (auto& kv : *in.agv_map) kv.second.covariance = Eigen::Matrix4d::Identity();
    for (auto& kv : *in.uav_map) kv.second.covariance = Eigen::Matrix4d::Identity();
    in.anchor_agv->covariance = Eigen::Matrix4d::Identity();
    in.anchor_uav->covariance = Eigen::Matrix4d::Identity();
    return false;
  }

  for (State* s : optimized) {
    double cov_data[16];
    if (covariance.GetCovarianceBlock(s->state.data(), s->state.data(), cov_data)) {
      s->covariance = Eigen::Map<Eigen::Matrix4d>(cov_data);
    } else {
      s->covariance = Eigen::Matrix4d::Identity();
      RCLCPP_WARN(logger_, "Failed to get covariance for a pose-graph node.");
    }
  }
  return true;
}

} // namespace uwb_localization
