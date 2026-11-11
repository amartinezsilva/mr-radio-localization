#pragma once

// GTSAM version of the pose-graph factors in CostFunctions.hpp / posegraph.hpp.
//
// Same model as the Ceres factors: every node is 4-DoF [x, y, z, yaw] with
// roll/pitch fixed (from the IMU), and every factor's error is
//   e = [rho_x, rho_y, rho_z, phi_z]  of  log(T_meas^{-1} * T_pred),
// whitened by the measurement covariance. They differ only in T_pred:
//   Prior:   T_pred = T(node)
//   Between: T_pred = T(target)^{-1} * T(source)                       (odometry, radar)
//   Anchor:  T_pred = (A_uav * T(target))^{-1} * (A_agv * T(source))   (UWB, radar encounters)
//
// Variables (gtsam::Values):
//   - 4-DoF node:   gtsam::Vector4 [x, y, z, yaw]
//   - planar node:  gtsam::Vector3 [x, y, yaw]; z is a constant of the factor
//     (the Ceres SubsetManifold equivalent)
//   - nodes outside the sliding window are not variables at all: they enter
//     the factor as constants (the SetParameterBlockConstant equivalent).
//
// Unlike Ceres, GTSAM has no autodiff: Jacobians are chained by hand. Only
// pose() below needs a hand-derived one; compose, between and the log
// derivative come from GTSAM. Checked against numerical derivatives and the
// Ceres factors in test/test_posegraph_backends.cpp.

#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <sophus/se3.hpp>

#include "uwb_localization/utils.hpp"  // buildSE3

namespace uwb_localization {
namespace gtsam_factors {

// One node referenced by a factor.
struct NodeSlot {
  enum class Kind { Free4, FreePlanar, Constant };

  Kind kind = Kind::Constant;
  gtsam::Key key = 0;               // Free4 / FreePlanar
  Eigen::Vector4d state = Eigen::Vector4d::Zero();  // Constant: the state. FreePlanar: state[2] is the fixed z.
  double roll = 0.0, pitch = 0.0;

  static NodeSlot free4(gtsam::Key key, double roll, double pitch) {
    NodeSlot s; s.kind = Kind::Free4; s.key = key; s.roll = roll; s.pitch = pitch; return s;
  }
  static NodeSlot freePlanar(gtsam::Key key, double z, double roll, double pitch) {
    NodeSlot s; s.kind = Kind::FreePlanar; s.key = key; s.state[2] = z; s.roll = roll; s.pitch = pitch; return s;
  }
  static NodeSlot constant(const Eigen::Vector4d& state, double roll, double pitch) {
    NodeSlot s; s.kind = Kind::Constant; s.state = state; s.roll = roll; s.pitch = pitch; return s;
  }

  bool isFree() const { return kind != Kind::Constant; }
  int dim() const { return kind == Kind::Free4 ? 4 : kind == Kind::FreePlanar ? 3 : 0; }

  // Full state [x, y, z, yaw] of this slot under `values`.
  Eigen::Vector4d fullState(const gtsam::Values& values) const {
    switch (kind) {
      case Kind::Free4: return values.at<gtsam::Vector4>(key);
      case Kind::FreePlanar: {
        const gtsam::Vector3 v = values.at<gtsam::Vector3>(key);
        return Eigen::Vector4d(v[0], v[1], state[2], v[2]);
      }
      default: return state;
    }
  }

  // Pose of the node, and (for free slots) its 6 x dim() Jacobian in GTSAM's
  // Pose3 tangent space [omega; v] (right perturbation, T * Exp(xi)):
  //   d/d(x,y,z): v = R^T * e_i,  omega = 0
  //   d/d(yaw):   omega = R^T * e_z, v = 0   (R(yaw + d) = Rz(d) * R)
  gtsam::Pose3 pose(const gtsam::Values& values, gtsam::Matrix* H = nullptr) const {
    const Eigen::Vector4d s = fullState(values);
    const Sophus::SE3d T = buildSE3(s.data(), roll, pitch);
    const gtsam::Pose3 P(gtsam::Rot3(T.rotationMatrix()), gtsam::Point3(T.translation()));
    if (H && isFree()) {
      const Eigen::Matrix3d Rt = T.rotationMatrix().transpose();
      H->setZero(6, dim());
      if (kind == Kind::Free4) {
        H->block<3, 3>(3, 0) = Rt;             // x, y, z
        H->block<3, 1>(0, 3) = Rt.col(2);      // yaw
      } else {
        H->block<3, 1>(3, 0) = Rt.col(0);      // x
        H->block<3, 1>(3, 1) = Rt.col(1);      // y
        H->block<3, 1>(0, 2) = Rt.col(2);      // yaw
      }
    }
    return P;
  }
};

class Pose4DoFFactor : public gtsam::NoiseModelFactor {
public:
  enum class Type { Prior, Between, Anchor };

  // Slots, by type:
  //   Prior:   {node}
  //   Between: {source, target}
  //   Anchor:  {source (AGV node), target (UAV node), UAV anchor, AGV anchor}
  // The factor's keys are those of the free slots, in this order.
  Pose4DoFFactor(Type type, const Sophus::SE3d& T_meas, std::vector<NodeSlot> slots,
                 const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor(noise, freeKeys(slots)),
        type_(type),
        T_meas_(gtsam::Rot3(T_meas.rotationMatrix()), gtsam::Point3(T_meas.translation())),
        slots_(std::move(slots)) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new Pose4DoFFactor(*this));
  }

  gtsam::Vector unwhitenedError(const gtsam::Values& x,
                                boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
    const size_t n = slots_.size();
    std::vector<gtsam::Pose3> P(n);
    std::vector<gtsam::Matrix> J(n);                  // d pose_k / d variable_k
    for (size_t k = 0; k < n; ++k) P[k] = slots_[k].pose(x, H ? &J[k] : nullptr);

    // T_pred and d T_pred / d pose_k.
    std::vector<gtsam::Matrix66> D(n, gtsam::Matrix66::Zero());
    gtsam::Pose3 T_pred;
    switch (type_) {
      case Type::Prior:
        T_pred = P[0];
        D[0].setIdentity();
        break;
      case Type::Between:
        T_pred = P[1].between(P[0], D[1], D[0]);
        break;
      case Type::Anchor: {
        gtsam::Matrix66 Ds, Dt, Dws, Dwt;
        const gtsam::Pose3 w_T_s = P[3].compose(P[0], D[3], Ds);
        const gtsam::Pose3 w_T_t = P[2].compose(P[1], D[2], Dt);
        T_pred = w_T_t.between(w_T_s, Dwt, Dws);
        D[0] = Dws * Ds;  D[3] = Dws * D[3];
        D[1] = Dwt * Dt;  D[2] = Dwt * D[2];
        break;
      }
    }

    // e = selected rows of log(T_meas^{-1} * T_pred). GTSAM orders the
    // tangent as [omega; v], so translation is rows 3..5 and yaw is row 2.
    // The log itself is taken with Sophus, as in the Ceres factors: GTSAM
    // 4.2's Rot3::Logmap is approximate for rotations within a few degrees
    // of pi, which large (outlier) residuals do reach. Its Jacobian comes
    // from GTSAM's closed-form LogmapDerivative, which is accurate there.
    gtsam::Matrix66 H_between;
    const gtsam::Pose3 T_err = T_meas_.between(T_pred, boost::none, H_between);
    const Eigen::Matrix<double, 6, 1> xi_sophus =
        Sophus::SE3d(T_err.rotation().matrix(), T_err.translation()).log();  // [v; omega]
    gtsam::Vector6 xi;
    xi << xi_sophus.tail<3>(), xi_sophus.head<3>();
    // Pose3::LogmapDerivative(pose) would recompute the (approximate) log, so
    // assemble it from xi the same way GTSAM does: [Jw 0; -Jw Q Jw  Jw].
    const gtsam::Matrix3 Jw = gtsam::Rot3::LogmapDerivative(xi.head<3>());
    const gtsam::Matrix3 Q = gtsam::Pose3::ComputeQforExpmapDerivative(xi);
    gtsam::Matrix66 H_log;
    H_log << Jw, gtsam::Matrix3::Zero(), -Jw * Q * Jw, Jw;
    gtsam::Vector e(4);
    e << xi[3], xi[4], xi[5], xi[2];

    if (H) {
      Eigen::Matrix<double, 4, 6> S = Eigen::Matrix<double, 4, 6>::Zero();
      S(0, 3) = S(1, 4) = S(2, 5) = S(3, 2) = 1.0;
      const Eigen::Matrix<double, 4, 6> SE = S * H_log * H_between;
      size_t j = 0;
      for (size_t k = 0; k < n; ++k) {
        if (!slots_[k].isFree()) continue;
        (*H)[j++] = SE * D[k] * J[k];
      }
    }
    return e;
  }

private:
  static gtsam::KeyVector freeKeys(const std::vector<NodeSlot>& slots) {
    gtsam::KeyVector keys;
    for (const auto& s : slots) if (s.isFree()) keys.push_back(s.key);
    return keys;
  }

  Type type_;
  gtsam::Pose3 T_meas_;
  std::vector<NodeSlot> slots_;
};

} // namespace gtsam_factors
} // namespace uwb_localization
