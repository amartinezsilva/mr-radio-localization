#include "uwb_localization/gtsam_backend.hpp"

#include <chrono>
#include <cmath>
#include <map>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "uwb_localization/gtsam_factors.hpp"

namespace uwb_localization {

using gtsam_factors::NodeSlot;
using gtsam_factors::Pose4DoFFactor;

namespace {

double secondsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// One optimizable state (robot node or anchor) and how it enters the graph.
struct Variable {
  State* state = nullptr;
  bool planar = false;
  bool used = false;  // referenced by at least one factor
};

// Builds the graph and the initial Values from a PoseGraphInput.
class GraphBuilder {
public:
  GraphBuilder(PoseGraphInput& in, double loss_threshold) : in_(in), loss_threshold_(loss_threshold) {}

  void build() {
    // The anchors are free only if some encounter factor is used (same rule as Ceres).
    anchors_free_ = usesAny(in_.encounters_uwb) || usesAny(in_.encounters_radar);

    if (in_.isFree(false, 0)) addPrior(nodeSlot(false, 0), *in_.prior_agv);
    if (in_.isFree(true, 0)) addPrior(nodeSlot(true, 0), *in_.prior_uav);

    // Odometry without robust loss (it drifts but has no outliers); radar and UWB with it.
    // Same as CeresBackend.
    addIntra(false, in_.odom_agv, false);
    addIntra(true, in_.odom_uav, false);
    addIntra(false, in_.radar_agv, true);
    addIntra(true, in_.radar_uav, true);

    addEncounters(in_.encounters_uwb);
    addEncounters(in_.encounters_radar);

    if (anchors_free_) addPrior(anchorSlot(in_.fix_uav_anchor), *in_.prior_anchor);  // fixes the gauge

    // Initial values for every variable a factor references.
    for (auto& [key, var] : variables_) {
      if (!var.used) continue;
      const Eigen::Vector4d& s = var.state->state;
      if (var.planar) initial_.insert<gtsam::Vector3>(key, gtsam::Vector3(s[0], s[1], s[3]));
      else initial_.insert<gtsam::Vector4>(key, gtsam::Vector4(s));
    }
  }

  // Copies the optimized values back into the states (z of planar nodes is untouched).
  void writeBack(const gtsam::Values& result) {
    for (auto& [key, var] : variables_) {
      if (!var.used) continue;
      Eigen::Vector4d& s = var.state->state;
      if (var.planar) {
        const gtsam::Vector3 v = result.at<gtsam::Vector3>(key);
        s[0] = v[0]; s[1] = v[1]; s[3] = v[2];
      } else {
        s = result.at<gtsam::Vector4>(key);
      }
    }
  }

  // Marginal covariance of every variable, as a 4x4 [x, y, z, yaw] matrix
  // (zero z row/column for planar nodes, as Ceres reports with SubsetManifold).
  void writeCovariances(const gtsam::Values& result) {
    gtsam::Marginals marginals(graph_, result);
    for (auto& [key, var] : variables_) {
      if (!var.used) continue;
      const gtsam::Matrix C = marginals.marginalCovariance(key);
      Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
      if (var.planar) {
        const int idx[3] = {0, 1, 3};
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j) cov(idx[i], idx[j]) = C(i, j);
      } else {
        cov = C;
      }
      var.state->covariance = cov;
    }
  }

  const gtsam::NonlinearFactorGraph& graph() const { return graph_; }
  const gtsam::Values& initial() const { return initial_; }

private:
  // A slot for robot node `id`: a variable if it is in the window, else a constant.
  NodeSlot nodeSlot(bool uav, int id) {
    State& s = (uav ? *in_.uav_map : *in_.agv_map)[id];
    if (!in_.isFree(uav, id)) return NodeSlot::constant(s.state, s.roll, s.pitch);
    return variableSlot(gtsam::Symbol(uav ? 'u' : 'a', id), s, s.planar);
  }

  // The AGV anchor is planar, the UAV anchor 4-DoF (as in CeresBackend).
  NodeSlot anchorSlot(bool uav) {
    State& s = uav ? *in_.anchor_uav : *in_.anchor_agv;
    if (!anchors_free_) return NodeSlot::constant(s.state, s.roll, s.pitch);
    return variableSlot(gtsam::Symbol(uav ? 'U' : 'A', 0), s, !uav);
  }

  NodeSlot variableSlot(gtsam::Key key, State& s, bool planar) {
    Variable& var = variables_[key];
    var.state = &s; var.planar = planar; var.used = true;
    return planar ? NodeSlot::freePlanar(key, s.state[2], s.roll, s.pitch)
                  : NodeSlot::free4(key, s.roll, s.pitch);
  }

  gtsam::SharedNoiseModel noise(const Eigen::Matrix4d& covariance, bool robust) const {
    auto gaussian = gtsam::noiseModel::Gaussian::Covariance(covariance);
    if (!robust) return gaussian;
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(loss_threshold_),
                                             gaussian);
  }

  bool usesAny(const posegraph::VectorOfConstraints* constraints) const {
    if (!constraints) return false;
    for (const auto& c : *constraints)
      if (in_.isFree(false, c.id_begin) || in_.isFree(true, c.id_end)) return true;
    return false;
  }

  void addPrior(const NodeSlot& slot, const posegraph::PriorConstraint& prior) {
    graph_.emplace_shared<Pose4DoFFactor>(Pose4DoFFactor::Type::Prior, prior.pose,
                                          std::vector<NodeSlot>{slot}, noise(prior.covariance, false));
  }

  void addIntra(bool uav, const posegraph::VectorOfConstraints* constraints, bool robust) {
    if (!constraints) return;
    for (const auto& c : *constraints) {
      if (!in_.isFree(uav, c.id_begin) && !in_.isFree(uav, c.id_end)) continue;
      graph_.emplace_shared<Pose4DoFFactor>(Pose4DoFFactor::Type::Between, c.t_T_s,
                                            std::vector<NodeSlot>{nodeSlot(uav, c.id_begin), nodeSlot(uav, c.id_end)},
                                            noise(c.covariance, robust));
    }
  }

  void addEncounters(const posegraph::VectorOfConstraints* constraints) {
    if (!constraints) return;
    for (const auto& c : *constraints) {
      if (!in_.isFree(false, c.id_begin) && !in_.isFree(true, c.id_end)) continue;
      graph_.emplace_shared<Pose4DoFFactor>(
          Pose4DoFFactor::Type::Anchor, c.t_T_s,
          std::vector<NodeSlot>{nodeSlot(false, c.id_begin), nodeSlot(true, c.id_end),
                                anchorSlot(true), anchorSlot(false)},
          noise(c.covariance, true));
    }
  }

  PoseGraphInput& in_;
  double loss_threshold_;
  bool anchors_free_ = false;
  std::map<gtsam::Key, Variable> variables_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
};

}  // namespace

bool GtsamBackend::optimize(PoseGraphInput& in, PoseGraphSolveStats& stats) {
  stats = PoseGraphSolveStats{};

  auto t0 = std::chrono::steady_clock::now();
  GraphBuilder builder(in, options_.loss_threshold);
  builder.build();
  stats.build_time = secondsSince(t0);
  stats.num_factors = static_cast<int>(builder.graph().size());
  stats.num_variables = static_cast<int>(builder.initial().size());

  // Stopping rule closest to Ceres' function_tolerance (relative cost decrease).
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = options_.max_iterations;
  params.relativeErrorTol = options_.function_tolerance;
  params.absoluteErrorTol = 0.0;

  gtsam::Values result;
  t0 = std::chrono::steady_clock::now();
  try {
    gtsam::LevenbergMarquardtOptimizer optimizer(builder.graph(), builder.initial(), params);
    stats.initial_cost = builder.graph().error(builder.initial());
    result = optimizer.optimize();
    stats.iterations = static_cast<int>(optimizer.iterations());
    stats.final_cost = optimizer.error();
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    stats.solve_time = secondsSince(t0);
    RCLCPP_WARN(logger_, "GTSAM: indeterminant linear system: %s", e.what());
    return false;
  }
  stats.solve_time = secondsSince(t0);
  stats.converged = std::isfinite(stats.final_cost) && stats.iterations < options_.max_iterations;

  RCLCPP_INFO(logger_, "GTSAM LM: %d factors, %d variables, %d iterations, cost %g -> %g, %.4f s%s",
              stats.num_factors, stats.num_variables, stats.iterations, stats.initial_cost,
              stats.final_cost, stats.solve_time, stats.converged ? "" : " (not converged)");

  if (!std::isfinite(stats.final_cost)) return false;
  builder.writeBack(result);  // as Ceres: the solution is written even without convergence
  if (!stats.converged) return false;

  t0 = std::chrono::steady_clock::now();
  try {
    builder.writeCovariances(result);
    stats.covariance_ok = true;
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "GTSAM: failed to compute covariances: %s", e.what());
    for (auto& kv : *in.agv_map) kv.second.covariance = Eigen::Matrix4d::Identity();
    for (auto& kv : *in.uav_map) kv.second.covariance = Eigen::Matrix4d::Identity();
    in.anchor_agv->covariance = Eigen::Matrix4d::Identity();
    in.anchor_uav->covariance = Eigen::Matrix4d::Identity();
  }
  stats.covariance_time = secondsSince(t0);
  return true;
}

} // namespace uwb_localization
