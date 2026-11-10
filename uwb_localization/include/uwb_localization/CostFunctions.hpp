#ifndef UWB_LOCALIZATION_COSTFUNCTIONS_H_
#define UWB_LOCALIZATION_COSTFUNCTIONS_H_

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Cholesky>

#include "uwb_localization/utils.hpp"  // buildSE3

namespace uwb_localization
{
    // ---------------------------------------------------------------------
    // Shared building blocks
    // ---------------------------------------------------------------------
    //
    // Every node is 4-DoF: state = [x, y, z, yaw]. Roll and pitch are not
    // optimized; they come from the IMU and are passed to each factor as
    // constants. The pose of a state is built with buildSE3 (utils.hpp).

    // Square-root information matrix S such that ||S e||^2 = e^T cov^{-1} e,
    // with cov = L L^T (Cholesky) and S = L^{-T}. Computed once per factor.
    inline Eigen::Matrix4d sqrtInformation(const Eigen::Matrix4d& covariance) {
        Eigen::LLT<Eigen::Matrix4d> chol(covariance);
        return Eigen::Matrix4d(chol.matrixL().transpose()).inverse();
    }

    // Weighted 4-DoF error between a measured and a predicted pose:
    //   T_err = T_meas^{-1} * T_pred,  xi = log(T_err) = [rho; phi],
    //   e = [rho_x, rho_y, rho_z, phi_z],  residual = S * e.
    template <typename T>
    void weightedPoseError4D(const Sophus::SE3d& T_meas,
                             const Eigen::Matrix4d& sqrt_information,
                             const Sophus::SE3<T>& T_pred,
                             T* residual) {
        const Sophus::SE3<T> T_err = T_meas.template cast<T>().inverse() * T_pred;
        const Eigen::Matrix<T, 6, 1> xi = T_err.log();

        Eigen::Matrix<T, 4, 1> error;
        error.template head<3>() = xi.template head<3>();  // translation
        error[3] = xi[5];                                   // yaw (z of the rotation vector)

        Eigen::Map<Eigen::Matrix<T, 4, 1>> weighted(residual);
        weighted = sqrt_information.template cast<T>() * error;
    }

    // ---------------------------------------------------------------------
    // UWB range factor (relative transform estimation, global_opt_node_eliko)
    // ---------------------------------------------------------------------
    struct UWBCostFunction {
        UWBCostFunction(const Eigen::Vector3d& anchor, const Eigen::Vector3d& tag, double measured_distance,
                        double roll, double pitch, double measurement_stdev)
            : anchor_(anchor), tag_(tag), measured_distance_(measured_distance),
              roll_(roll), pitch_(pitch), measurement_stdev_inv_(1.0 / measurement_stdev) {}

        template <typename T>
        bool operator()(const T* const state, T* residual) const {
            // Transform the anchor with the relative pose, then compare the
            // predicted anchor-tag distance with the measured range.
            const Sophus::SE3<T> SE3_rel = buildSE3(state, roll_, pitch_);
            const Eigen::Matrix<T, 3, 1> anchor_transformed = SE3_rel * anchor_.cast<T>();
            const T predicted_distance = (tag_.cast<T>() - anchor_transformed).norm();

            residual[0] = (T(measured_distance_) - predicted_distance) * T(measurement_stdev_inv_);
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector3d& anchor, const Eigen::Vector3d& tag, double measured_distance,
                                           double roll, double pitch, double measurement_stdev) {
            return new ceres::AutoDiffCostFunction<UWBCostFunction, 1, 4>(
                new UWBCostFunction(anchor, tag, measured_distance, roll, pitch, measurement_stdev));
        }

        const Eigen::Vector3d anchor_;
        const Eigen::Vector3d tag_;
        const double measured_distance_;
        const double roll_;
        const double pitch_;
        const double measurement_stdev_inv_;
    };

    // ---------------------------------------------------------------------
    // Pose-graph factors. All three use the same 4-DoF pose error and differ
    // only in how the predicted pose is built from their parameter blocks.
    // ---------------------------------------------------------------------

    // Unary prior on one node: T_pred = T(state).
    struct PriorCostFunction {
        PriorCostFunction(const Sophus::SE3d& prior_T, const double& roll, const double& pitch,
                          const Eigen::Matrix4d& prior_covariance)
            : prior_T_(prior_T), roll_(roll), pitch_(pitch),
              sqrt_information_(sqrtInformation(prior_covariance)) {}

        template <typename T>
        bool operator()(const T* const state, T* residual) const {
            weightedPoseError4D(prior_T_, sqrt_information_, buildSE3(state, roll_, pitch_), residual);
            return true;
        }

        static ceres::CostFunction* Create(const Sophus::SE3d& prior_T, const double& roll, const double& pitch,
                                           const Eigen::Matrix4d& prior_covariance) {
            return new ceres::AutoDiffCostFunction<PriorCostFunction, 4, 4>(
                new PriorCostFunction(prior_T, roll, pitch, prior_covariance));
        }

    private:
        const Sophus::SE3d prior_T_;
        const double roll_, pitch_;
        const Eigen::Matrix4d sqrt_information_;
    };

    // Relative pose between two nodes of the same robot (odometry, radar):
    // T_pred = T_target^{-1} * T_source.
    struct MeasurementResidual {
        MeasurementResidual(const Sophus::SE3d& T_meas, const Eigen::Matrix4d& cov,
                            double source_roll, double source_pitch, double target_roll, double target_pitch)
            : T_meas_(T_meas), sqrt_information_(sqrtInformation(cov)),
              source_roll_(source_roll), source_pitch_(source_pitch),
              target_roll_(target_roll), target_pitch_(target_pitch) {}

        template <typename T>
        bool operator()(const T* const source_state, const T* const target_state, T* residual) const {
            const Sophus::SE3<T> T_s = buildSE3(source_state, source_roll_, source_pitch_);
            const Sophus::SE3<T> T_t = buildSE3(target_state, target_roll_, target_pitch_);
            weightedPoseError4D(T_meas_, sqrt_information_, T_t.inverse() * T_s, residual);
            return true;
        }

        static ceres::CostFunction* Create(const Sophus::SE3d& T_meas, const Eigen::Matrix4d& cov,
                                           double source_roll, double source_pitch,
                                           double target_roll, double target_pitch) {
            return new ceres::AutoDiffCostFunction<MeasurementResidual, 4, 4, 4>(
                new MeasurementResidual(T_meas, cov, source_roll, source_pitch, target_roll, target_pitch));
        }

    private:
        const Sophus::SE3d T_meas_;
        const Eigen::Matrix4d sqrt_information_;
        const double source_roll_, source_pitch_;
        const double target_roll_, target_pitch_;
    };

    // Relative pose between nodes of different robots (UWB, radar encounters),
    // each expressed in the common frame through its robot's anchor node:
    // T_pred = (anchor_uav * T_target)^{-1} * (anchor_agv * T_source).
    struct AnchorResidual {
        AnchorResidual(const Sophus::SE3d& T_meas, const Eigen::Matrix4d& cov,
                       double source_roll, double source_pitch,
                       double target_roll, double target_pitch,
                       double anchor_roll_uav, double anchor_pitch_uav,
                       double anchor_roll_agv, double anchor_pitch_agv)
            : T_meas_(T_meas), sqrt_information_(sqrtInformation(cov)),
              source_roll_(source_roll), source_pitch_(source_pitch),
              target_roll_(target_roll), target_pitch_(target_pitch),
              anchor_roll_uav_(anchor_roll_uav), anchor_pitch_uav_(anchor_pitch_uav),
              anchor_roll_agv_(anchor_roll_agv), anchor_pitch_agv_(anchor_pitch_agv) {}

        template <typename T>
        bool operator()(const T* const source_state, const T* const target_state,
                        const T* const T_anchor_uav, const T* const T_anchor_agv, T* residual) const {
            const Sophus::SE3<T> w_T_s = buildSE3(T_anchor_agv, anchor_roll_agv_, anchor_pitch_agv_) *
                                         buildSE3(source_state, source_roll_, source_pitch_);
            const Sophus::SE3<T> w_T_t = buildSE3(T_anchor_uav, anchor_roll_uav_, anchor_pitch_uav_) *
                                         buildSE3(target_state, target_roll_, target_pitch_);
            weightedPoseError4D(T_meas_, sqrt_information_, w_T_t.inverse() * w_T_s, residual);
            return true;
        }

        static ceres::CostFunction* Create(const Sophus::SE3d& T_meas, const Eigen::Matrix4d& cov,
                                           double source_roll, double source_pitch,
                                           double target_roll, double target_pitch,
                                           double anchor_roll_uav, double anchor_pitch_uav,
                                           double anchor_roll_agv, double anchor_pitch_agv) {
            return new ceres::AutoDiffCostFunction<AnchorResidual, 4, 4, 4, 4, 4>(
                new AnchorResidual(T_meas, cov, source_roll, source_pitch, target_roll, target_pitch,
                                   anchor_roll_uav, anchor_pitch_uav, anchor_roll_agv, anchor_pitch_agv));
        }

    private:
        const Sophus::SE3d T_meas_;
        const Eigen::Matrix4d sqrt_information_;
        const double source_roll_, source_pitch_;
        const double target_roll_, target_pitch_;
        const double anchor_roll_uav_, anchor_pitch_uav_;
        const double anchor_roll_agv_, anchor_pitch_agv_;
    };

}

#endif
