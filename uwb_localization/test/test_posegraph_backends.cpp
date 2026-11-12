// Checks that the GTSAM pose-graph backend solves the same model as the Ceres one:
//  1) Each GTSAM factor's error equals the Ceres functor's residual (identity
//     covariance), for every mix of 4-DoF / planar / constant nodes.
//  2) The GTSAM factors' hand-chained Jacobians match numerical derivatives.
//  3) Both backends give the same solution and covariances on a small
//     two-robot graph with a sliding window, encounters and anchors.
#include <gtest/gtest.h>

#include <random>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "uwb_localization/CostFunctions.hpp"
#include "uwb_localization/gtsam_factors.hpp"
#include "uwb_localization/posegraph_backend.hpp"

using namespace uwb_localization;
using gtsam_factors::NodeSlot;
using gtsam_factors::Pose4DoFFactor;

namespace {

std::mt19937 rng(7);
double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
Eigen::Vector4d randomState() { return {U(-10, 10), U(-10, 10), U(-2, 3), U(-M_PI, M_PI)}; }

struct TestNode {
  Eigen::Vector4d state;
  double roll, pitch;
  NodeSlot::Kind kind;
};

// Slot + the matching initial value (if free) for node k.
NodeSlot makeSlot(const TestNode& n, gtsam::Key key, gtsam::Values& values) {
  switch (n.kind) {
    case NodeSlot::Kind::Free4:
      values.insert<gtsam::Vector4>(key, gtsam::Vector4(n.state));
      return NodeSlot::free4(key, n.roll, n.pitch);
    case NodeSlot::Kind::FreePlanar:
      values.insert<gtsam::Vector3>(key, gtsam::Vector3(n.state[0], n.state[1], n.state[3]));
      return NodeSlot::freePlanar(key, n.state[2], n.roll, n.pitch);
    default:
      return NodeSlot::constant(n.state, n.roll, n.pitch);
  }
}

}  // namespace

TEST(GtsamFactors, ErrorAndJacobiansMatchCeres) {
  const NodeSlot::Kind kinds[3] = {NodeSlot::Kind::Free4, NodeSlot::Kind::FreePlanar, NodeSlot::Kind::Constant};
  const Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  auto unit = gtsam::noiseModel::Unit::Create(4);

  int checked = 0;
  for (int trial = 0; trial < 200; ++trial) {
    TestNode n[4];
    for (auto& x : n) { x.state = randomState(); x.roll = U(-0.2, 0.2); x.pitch = U(-0.2, 0.2); x.kind = kinds[rng() % 3]; }
    if (n[0].kind == NodeSlot::Kind::Constant) n[0].kind = NodeSlot::Kind::Free4;  // need >= 1 variable
    const double* s[4] = {n[0].state.data(), n[1].state.data(), n[2].state.data(), n[3].state.data()};
    Sophus::SE3d P[4];
    for (int k = 0; k < 4; ++k) P[k] = buildSE3(s[k], n[k].roll, n[k].pitch);

    for (int type = 0; type < 3; ++type) {
      const int arity = type == 0 ? 1 : type == 1 ? 2 : 4;
      // Measurement = prediction with a moderate error (rotation up to ~0.5 rad per
      // axis), so central differences don't straddle the log's jump at pi (NearPi
      // covers residuals close to pi).
      const Sophus::SE3d T_pred = type == 0 ? P[0] : type == 1 ? P[1].inverse() * P[0]
                                                                : (P[2] * P[1]).inverse() * (P[3] * P[0]);
      Eigen::Matrix<double, 6, 1> perturbation;
      perturbation << U(-1, 1), U(-1, 1), U(-1, 1), U(-0.5, 0.5), U(-0.5, 0.5), U(-0.5, 0.5);
      const Sophus::SE3d T_meas = T_pred * Sophus::SE3d::exp(perturbation);
      gtsam::Values values;
      std::vector<NodeSlot> slots;
      for (int k = 0; k < arity; ++k) slots.push_back(makeSlot(n[k], gtsam::Symbol('x', k), values));

      std::unique_ptr<ceres::CostFunction> ceres_cost;
      if (type == 0) {
        ceres_cost.reset(PriorCostFunction::Create(T_meas, n[0].roll, n[0].pitch, I));
      } else if (type == 1) {
        ceres_cost.reset(MeasurementResidual::Create(T_meas, I, n[0].roll, n[0].pitch, n[1].roll, n[1].pitch));
      } else {
        ceres_cost.reset(AnchorResidual::Create(T_meas, I, n[0].roll, n[0].pitch, n[1].roll, n[1].pitch,
                                                           n[2].roll, n[2].pitch, n[3].roll, n[3].pitch));
      }
      double r_ceres[4];
      ASSERT_TRUE(ceres_cost->Evaluate(s, r_ceres, nullptr));

      Pose4DoFFactor factor(static_cast<Pose4DoFFactor::Type>(type), T_meas, slots, unit);
      const gtsam::Vector e = factor.unwhitenedError(values);
      for (int i = 0; i < 4; ++i) EXPECT_NEAR(e[i], r_ceres[i], 1e-9) << "type " << type << " row " << i;

      // Jacobians vs central differences (GTSAM's own checker).
      EXPECT_TRUE(gtsam::internal::testFactorJacobians("Pose4DoFFactor", factor, values, 1e-6, 1e-5))
          << "type " << type;
      ++checked;
    }
  }
  EXPECT_EQ(checked, 600);
}

// With a correlated (non-diagonal) covariance, the Ceres residual must give
// the Mahalanobis distance e^T C^-1 e, and GTSAM's whitened error the same.
TEST(GtsamFactors, CorrelatedCovariance) {
  for (int trial = 0; trial < 50; ++trial) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Random();
    const Eigen::Matrix4d C = A * A.transpose() + 0.05 * Eigen::Matrix4d::Identity();
    const Eigen::Vector4d state = randomState();
    const Sophus::SE3d T_meas = buildSE3(state.data(), 0.05, -0.03) *
        Sophus::SE3d::exp((Eigen::Matrix<double, 6, 1>() << U(-1, 1), U(-1, 1), U(-1, 1), 0.1, -0.1, U(-0.5, 0.5)).finished());

    std::unique_ptr<ceres::CostFunction> c(PriorCostFunction::Create(T_meas, 0.05, -0.03, C));
    std::unique_ptr<ceres::CostFunction> c_unit(PriorCostFunction::Create(T_meas, 0.05, -0.03, Eigen::Matrix4d::Identity()));
    const double* p[1] = {state.data()};
    double r[4], e[4];
    ASSERT_TRUE(c->Evaluate(p, r, nullptr));
    ASSERT_TRUE(c_unit->Evaluate(p, e, nullptr));
    const Eigen::Vector4d ev(e[0], e[1], e[2], e[3]);
    const double mahalanobis = ev.dot(C.ldlt().solve(ev));
    EXPECT_NEAR(Eigen::Map<Eigen::Vector4d>(r).squaredNorm(), mahalanobis, 1e-9 * std::max(1.0, mahalanobis));

    gtsam::Values v;
    v.insert<gtsam::Vector4>(gtsam::Symbol('x', 0), gtsam::Vector4(state));
    Pose4DoFFactor f(Pose4DoFFactor::Type::Prior, T_meas, {NodeSlot::free4(gtsam::Symbol('x', 0), 0.05, -0.03)},
                     gtsam::noiseModel::Gaussian::Covariance(C));
    EXPECT_NEAR(f.whitenedError(v).squaredNorm(), mahalanobis, 1e-9 * std::max(1.0, mahalanobis));
  }
}

// Residual rotations close to pi (large outlier residuals reach them): the
// error must still match Ceres (Sophus log) exactly, and the Jacobians must
// still match numerical derivatives. (GTSAM 4.2's Rot3::Logmap was off by
// ~1e-4 there; 4.3 must not be.)
TEST(GtsamFactors, NearPi) {
  auto unit = gtsam::noiseModel::Unit::Create(4);
  const Eigen::Vector4d state(1.0, 2.0, 0.5, 0.0);
  for (double off : {0.04, 0.02, 0.01, 0.001}) {
    const Sophus::SE3d T_meas(Sophus::SO3d::rotZ(M_PI - off) * Sophus::SO3d::rotX(0.01),
                              Eigen::Vector3d(0.3, -0.2, 0.1));
    std::unique_ptr<ceres::CostFunction> c(PriorCostFunction::Create(T_meas, 0.02, -0.01, Eigen::Matrix4d::Identity()));
    double r[4];
    const double* p[1] = {state.data()};
    ASSERT_TRUE(c->Evaluate(p, r, nullptr));
    gtsam::Values v;
    v.insert<gtsam::Vector4>(gtsam::Symbol('x', 0), gtsam::Vector4(state));
    Pose4DoFFactor f(Pose4DoFFactor::Type::Prior, T_meas, {NodeSlot::free4(gtsam::Symbol('x', 0), 0.02, -0.01)}, unit);
    const gtsam::Vector e = f.unwhitenedError(v);
    for (int i = 0; i < 4; ++i) EXPECT_NEAR(e[i], r[i], 1e-9) << "pi - " << off << ", row " << i;
    EXPECT_TRUE(gtsam::internal::testFactorJacobians("NearPi", f, v, 1e-7, 1e-4)) << "pi - " << off;
  }
}

// Two-robot test graph: N nodes per robot (AGV planar, UAV 4-DoF), with noisy
// odometry chains, radar links 3 nodes back, and UWB encounters as in the node
// (latest node of each robot, through both anchors; correlated covariance).
struct Scenario {
  int N = 0;
  posegraph::MapOfStates agv, uav;          // initial guesses
  posegraph::VectorOfConstraints odom_agv, odom_uav, radar_agv, radar_uav, uwb, none;
  State anchor_agv, anchor_uav;
  posegraph::PriorConstraint prior_agv, prior_uav, prior_anchor;

  explicit Scenario(int n) : N(n) {
    std::vector<Sophus::SE3d> agv_true, uav_true;
    Sophus::SE3d Ta, Tu(Sophus::SO3d::rotZ(0.5), Eigen::Vector3d(1, 2, 1.5));
    for (int i = 0; i < N; ++i) {
      agv_true.push_back(Ta); uav_true.push_back(Tu);
      Ta = Ta * Sophus::SE3d(Sophus::SO3d::rotZ(0.15), Eigen::Vector3d(0.8, 0.05, 0.0));
      Tu = Tu * Sophus::SE3d(Sophus::SO3d::rotZ(-0.1), Eigen::Vector3d(0.6, 0.1, 0.05));
    }
    auto make_state = [&](const Sophus::SE3d& T, bool planar) {
      State s;
      s.state = transformSE3ToState(T) + Eigen::Vector4d(U(-0.3, 0.3), U(-0.3, 0.3), planar ? 0.0 : U(-0.1, 0.1), U(-0.05, 0.05));
      s.roll = 0.0; s.pitch = 0.0; s.planar = planar;
      s.pose = buildTransformationSE3(0.0, 0.0, s.state);
      s.covariance = Eigen::Matrix4d::Identity();
      return s;
    };
    for (int i = 0; i < N; ++i) { agv[i] = make_state(agv_true[i], true); uav[i] = make_state(uav_true[i], false); }

    auto noisy = [&](const Sophus::SE3d& T, double sp, double sy) {
      return T * Sophus::SE3d(Sophus::SO3d::rotZ(U(-sy, sy)), Eigen::Vector3d(U(-sp, sp), U(-sp, sp), U(-sp, sp) * 0.2));
    };
    auto link = [&](posegraph::VectorOfConstraints& v, int a, int b, const Sophus::SE3d& Ts, const Sophus::SE3d& Tt,
                    double sp, double sy, double var, bool correlated = false) {
      posegraph::MeasurementConstraint c;
      c.id_begin = a; c.id_end = b; c.t_T_s = noisy(Tt.inverse() * Ts, sp, sy);
      c.covariance = Eigen::Vector4d(var, var, var, var * 0.1).asDiagonal();
      if (correlated) {  // like the UWB relative-transform covariance: x-y-yaw correlations
        c.covariance(0, 1) = c.covariance(1, 0) = 0.5 * var;
        c.covariance(0, 3) = c.covariance(3, 0) = 0.1 * var;
        c.covariance(1, 3) = c.covariance(3, 1) = -0.1 * var;
      }
      v.push_back(c);
    };
    // Anchors: true AGV/UAV odometry frames in the map frame.
    const Sophus::SE3d A_agv(Sophus::SO3d::rotZ(3.0), Eigen::Vector3d(13, 27, 0.6));
    const Sophus::SE3d A_uav(Sophus::SO3d::rotZ(0.03), Eigen::Vector3d(15, 23, 0.9));
    for (int i = 1; i < N; ++i) {
      link(odom_agv, i - 1, i, agv_true[i - 1], agv_true[i], 0.05, 0.02, 0.1);
      link(odom_uav, i - 1, i, uav_true[i - 1], uav_true[i], 0.05, 0.02, 0.1);
      if (i >= 3 && i % 2 == 1) {
        link(radar_agv, i - 3, i, agv_true[i - 3], agv_true[i], 0.02, 0.01, 0.04);
        link(radar_uav, i - 3, i, uav_true[i - 3], uav_true[i], 0.02, 0.01, 0.04);
      }
    }
    for (int i = 2; i < N; ++i)
      link(uwb, i, i, A_agv * agv_true[i], A_uav * uav_true[i], 0.1, 0.03, 0.2, /*correlated=*/true);

    anchor_agv.state = transformSE3ToState(A_agv) + Eigen::Vector4d(0.5, -0.4, 0.0, 0.05);
    anchor_uav.state = transformSE3ToState(A_uav) + Eigen::Vector4d(-0.6, 0.3, 0.2, -0.05);
    anchor_agv.roll = anchor_agv.pitch = anchor_uav.roll = anchor_uav.pitch = 0.0;
    anchor_agv.covariance = anchor_uav.covariance = Eigen::Matrix4d::Identity();

    prior_agv = {agv_true[0], Eigen::Matrix4d::Identity() * 1e-6};
    prior_uav = {uav_true[0], Eigen::Matrix4d::Identity() * 1e-6};
    prior_anchor = {A_agv, Eigen::Matrix4d::Identity() * 1e-6};
  }

  // Input over (copies of) the states, with every constraint.
  PoseGraphInput input(posegraph::MapOfStates& a, posegraph::MapOfStates& u, State& aa, State& au, int max_keyframes) {
    PoseGraphInput in;
    in.agv_map = &a; in.uav_map = &u; in.anchor_agv = &aa; in.anchor_uav = &au;
    in.agv_current_id = in.uav_current_id = N - 1;
    in.max_keyframes = max_keyframes; in.min_keyframes = 3;
    in.prior_agv = &prior_agv; in.prior_uav = &prior_uav; in.prior_anchor_agv = &prior_anchor;
    in.odom_agv = &odom_agv; in.odom_uav = &odom_uav; in.radar_agv = &radar_agv; in.radar_uav = &radar_uav;
    in.encounters_uwb = &uwb; in.encounters_radar = &none;
    return in;
  }
};

// Solved to a tight tolerance, so the solvers must reach the same minimum
// (with the default 1e-6 they stop ~1e-4 apart, wherever each happens to stop).
// The floor left is Ceres' parameter_tolerance (1e-8 relative, ~2e-7 m here).
PoseGraphBackendOptions tightOptions() {
  PoseGraphBackendOptions options;
  options.function_tolerance = 1e-14;
  return options;
}

void expectSameState(const State& a, const State& b, double pos_tol, double yaw_tol, double cov_rel_tol,
                     const std::string& what) {
  EXPECT_LT((a.state.head<3>() - b.state.head<3>()).norm(), pos_tol) << what;
  EXPECT_LT(std::abs(normalizeAngle(a.state[3] - b.state[3])), yaw_tol) << what;
  EXPECT_LT((a.covariance - b.covariance).norm(), cov_rel_tol * a.covariance.norm() + 1e-12) << what;
}

TEST(PoseGraphBackends, CeresAndGtsamAgree) {
  Scenario sc(12);
  const PoseGraphBackendOptions options = tightOptions();
  // Two windows: the whole graph free, and only the last 8 nodes free (the rest constant).
  for (int max_keyframes : {100, 7}) {
    posegraph::MapOfStates agv_c = sc.agv, uav_c = sc.uav, agv_g = sc.agv, uav_g = sc.uav;
    State aa_c = sc.anchor_agv, au_c = sc.anchor_uav, aa_g = sc.anchor_agv, au_g = sc.anchor_uav;
    PoseGraphInput in_c = sc.input(agv_c, uav_c, aa_c, au_c, max_keyframes);
    PoseGraphInput in_g = sc.input(agv_g, uav_g, aa_g, au_g, max_keyframes);
    PoseGraphSolveStats stats_c, stats_g;
    ASSERT_TRUE(makePoseGraphBackend("ceres", options, rclcpp::get_logger("test"))->optimize(in_c, stats_c));
    ASSERT_TRUE(makePoseGraphBackend("gtsam", options, rclcpp::get_logger("test"))->optimize(in_g, stats_g));
    EXPECT_EQ(stats_c.num_factors, stats_g.num_factors);
    EXPECT_EQ(stats_c.num_variables, stats_g.num_variables);
    EXPECT_NEAR(stats_c.initial_cost, stats_g.initial_cost, 1e-9 * std::max(1.0, stats_c.initial_cost));
    EXPECT_NEAR(stats_c.final_cost, stats_g.final_cost, 1e-10 * std::max(1.0, stats_c.final_cost));
    EXPECT_TRUE(stats_c.covariance_ok);
    EXPECT_TRUE(stats_g.covariance_ok);

    const std::string w = " (window " + std::to_string(max_keyframes) + ")";
    for (int i = 0; i < sc.N; ++i) {
      expectSameState(agv_c[i], agv_g[i], 1e-6, 1e-7, 1e-5, "agv " + std::to_string(i) + w);
      expectSameState(uav_c[i], uav_g[i], 1e-6, 1e-7, 1e-5, "uav " + std::to_string(i) + w);
    }
    expectSameState(aa_c, aa_g, 1e-6, 1e-7, 1e-5, "anchor_agv" + w);
    expectSameState(au_c, au_g, 1e-6, 1e-7, 1e-5, "anchor_uav" + w);
    // Planar nodes keep their z in both.
    for (int i = 0; i < sc.N; ++i) EXPECT_EQ(agv_g[i].state[2], sc.agv[i].state[2]);
  }
}
