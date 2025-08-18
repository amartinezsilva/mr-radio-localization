#ifndef UWB_LOCALIZATION_COSTFUNCTIONS_H_
#define UWB_LOCALIZATION_COSTFUNCTIONS_H_

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace uwb_localization
{
   // Residual struct for Ceres
    struct UWBCostFunction {
    UWBCostFunction(const Eigen::Vector3d& anchor, const Eigen::Vector3d& tag, double measured_distance, double roll, double pitch, double measurement_stdev)
        : anchor_(anchor), tag_(tag), measured_distance_(measured_distance), roll_(roll), pitch_(pitch), measurement_stdev_inv_(1.0 / measurement_stdev) {}

    template <typename T>
    bool operator()(const T* const state, T* residual) const {

            // Build the 4x4 transformation matrix
            Sophus::SE3<T> SE3_rel = buildTransformationSE3(state, roll_, pitch_);

            // Transform the anchor point using the robot pose.
            Eigen::Matrix<T,3,1> anchor_vec;
            anchor_vec << T(anchor_(0)), T(anchor_(1)), T(anchor_(2));
            Eigen::Matrix<T,3,1> anchor_transformed = SE3_rel * anchor_vec;
            
            // Compute the predicted distance from the transformed anchor to the tag.
            Eigen::Matrix<T,3,1> tag_vec;
            tag_vec << T(tag_(0)), T(tag_(1)), T(tag_(2));
            T predicted_distance = (tag_vec - anchor_transformed).norm();

            // Residual as (measured - predicted)
            residual[0] = T(measured_distance_) - predicted_distance;
            residual[0] *= T(measurement_stdev_inv_);  // Scale residual

            //std::cout << "Measurement residual: " << residual[0] << std::endl;

            return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& anchor, const Eigen::Vector3d& tag, double measured_distance, double roll, double pitch, double measurement_stdev) {
        return (new ceres::AutoDiffCostFunction<UWBCostFunction, 1, 4>(
            new UWBCostFunction(anchor, tag, measured_distance, roll, pitch, measurement_stdev)));
    }

    template <typename T>
    Sophus::SE3<T> buildTransformationSE3(const T* state, double roll, double pitch) const{
        // Extract translation
        Eigen::Matrix<T, 3, 1> t;
        t << state[0], state[1], state[2];
        // Build rotation from yaw, with fixed roll and pitch
        Eigen::Matrix<T, 3, 3> R = (Eigen::AngleAxis<T>(state[3], Eigen::Matrix<T, 3, 1>::UnitZ()) *
                                    Eigen::AngleAxis<T>(T(pitch), Eigen::Matrix<T, 3, 1>::UnitY()) *
                                    Eigen::AngleAxis<T>(T(roll),  Eigen::Matrix<T, 3, 1>::UnitX())).toRotationMatrix();
        // Return the Sophus SE3 object
        return Sophus::SE3<T>(R, t);
    }

    const Eigen::Vector3d anchor_;
    const Eigen::Vector3d tag_;
    const double measured_distance_;
    const double roll_;
    const double pitch_;
    const double measurement_stdev_inv_;
    };

    struct PriorCostFunction {
        PriorCostFunction(const Sophus::SE3d& prior_T, const double &roll, const double &pitch, const Eigen::Matrix4d& prior_covariance)
            : prior_T_(prior_T), roll_(roll), pitch_(pitch), prior_covariance_(prior_covariance) {}

        template <typename T>
        bool operator()(const T* const state, T* residual) const {
            
            Sophus::SE3<T> SE3_pred = buildTransformationSE3(state, roll_, pitch_);
            Sophus::SE3<T> T_meas = prior_T_.template cast<T>();

            // Compute the error transformation: T_err = T_meas^{-1} * T_pred.
            Sophus::SE3<T> T_err = T_meas.inverse() * SE3_pred;
            
            // Compute the full 6-vector logarithm (xi = [rho; phi]),
            // where phi is the rotation vector.
            Eigen::Matrix<T,6,1> xi = T_err.log();

            // Project the 6-vector error onto the 4-DOF space:
            // Keep the three translation components and only the z component of the rotation.
            Eigen::Matrix<T,4,1> error_vec;
            error_vec.template segment<3>(0) = xi.template segment<3>(0); // translation error
            error_vec[3] = xi[5];  // use the z-component (yaw) of the rotation error

            // Scale by the square root of the inverse covariance matrix
            Eigen::LLT<Eigen::Matrix4d> chol(prior_covariance_);
            Eigen::Matrix4d sqrt_inv_covariance = Eigen::Matrix4d(chol.matrixL().transpose()).inverse();
            //Eigen::Matrix4d sqrt_covariance = Eigen::Matrix4d(chol.matrixL());

            Eigen::Matrix<T, 4, 1> weighted_residual = sqrt_inv_covariance.cast<T>() * error_vec;

            // Assign to residual
            residual[0] = weighted_residual[0];
            residual[1] = weighted_residual[1];
            residual[2] = weighted_residual[2];
            residual[3] = weighted_residual[3];

            return true;
        }

        static ceres::CostFunction* Create(const Sophus::SE3d& prior_T, const double& roll, const double &pitch, const Eigen::Matrix4d& prior_covariance) {
            return new ceres::AutoDiffCostFunction<PriorCostFunction, 4, 4>(
                new PriorCostFunction(prior_T, roll, pitch, prior_covariance));
        }

    private:

        template <typename T>
        Sophus::SE3<T> buildTransformationSE3(const T* state, double roll, double pitch) const {
            // Extract translation
            Eigen::Matrix<T, 3, 1> t;
            t << state[0], state[1], state[2];
            // Build rotation from yaw, with fixed roll and pitch
            Eigen::Matrix<T, 3, 3> R = (Eigen::AngleAxis<T>(state[3], Eigen::Matrix<T, 3, 1>::UnitZ()) *
                                        Eigen::AngleAxis<T>(T(pitch), Eigen::Matrix<T, 3, 1>::UnitY()) *
                                        Eigen::AngleAxis<T>(T(roll),  Eigen::Matrix<T, 3, 1>::UnitX())).toRotationMatrix();
            // Return the Sophus SE3 object
            return Sophus::SE3<T>(R, t);
        }

        const Sophus::SE3d prior_T_;
        const double roll_, pitch_;
        const Eigen::Matrix4d prior_covariance_;
    };


    struct MeasurementResidual {
        MeasurementResidual(const Sophus::SE3d& T_meas, const Eigen::Matrix4d& cov, double source_roll, double source_pitch, double target_roll, double target_pitch)
            : T_meas_(T_meas), cov_(cov), source_roll_(source_roll), source_pitch_(source_pitch), target_roll_(target_roll), target_pitch_(target_pitch) {}
    
        template <typename T>
        bool operator()(const T* const source_state, const T* const target_state, T* residual) const {
            // Build homogeneous transforms from the state vectors.
            Sophus::SE3<T> T_s = buildTransformationSE3(source_state, source_roll_, source_pitch_);
            Sophus::SE3<T> T_t = buildTransformationSE3(target_state, target_roll_, target_pitch_);
            
            // Compute the relative transform from AGV to UAV:
            Sophus::SE3<T> SE3_pred = T_t.inverse() * T_s;
            Sophus::SE3<T> SE3_meas = T_meas_.template cast<T>();
    
            // Compute the error transformation: T_err = T_meas^{-1} * T_pred.
            Sophus::SE3<T> T_err = SE3_meas.inverse() * SE3_pred;
            
            // Compute the full 6-vector logarithm (xi = [rho; phi]),
            // where phi is the rotation vector.
            Eigen::Matrix<T,6,1> xi = T_err.log();
    
            // Project the 6-vector error onto the 4-DOF space:
            // Keep the three translation components and only the z component of the rotation.
            Eigen::Matrix<T,4,1> error_vec;
            error_vec.template head<3>() = xi.template head<3>();
            error_vec[3] = xi[5];  // use the z-component (yaw) of the rotation error
    
            // Scale by the square root of the inverse covariance matrix
            Eigen::LLT<Eigen::Matrix4d> chol(cov_);
            Eigen::Matrix4d sqrt_inv_covariance = Eigen::Matrix4d(chol.matrixL().transpose()).inverse();
            
            Eigen::Matrix<T, 4, 1> weighted_residual = sqrt_inv_covariance.cast<T>() * error_vec;
    
            // Assign to residual
            residual[0] = weighted_residual[0];
            residual[1] = weighted_residual[1];
            residual[2] = weighted_residual[2];
            residual[3] = weighted_residual[3];
    
            return true;
        }
    
        static ceres::CostFunction* Create(const Sophus::SE3d& T_meas, const Eigen::Matrix4d& cov, double source_roll, double source_pitch, double target_roll, double target_pitch) {
            return new ceres::AutoDiffCostFunction<MeasurementResidual, 4, 4, 4>(
                new MeasurementResidual(T_meas, cov, source_roll, source_pitch, target_roll, target_pitch));
        }
    
        private:
            template <typename T>
            Sophus::SE3<T> buildTransformationSE3(const T* state, double roll, double pitch) const {
                // Extract translation
                Eigen::Matrix<T, 3, 1> t;
                t << state[0], state[1], state[2];
                // Build rotation from yaw, with fixed roll and pitch
                Eigen::Matrix<T, 3, 3> R = (Eigen::AngleAxis<T>(state[3], Eigen::Matrix<T, 3, 1>::UnitZ()) *
                                            Eigen::AngleAxis<T>(T(pitch), Eigen::Matrix<T, 3, 1>::UnitY()) *
                                            Eigen::AngleAxis<T>(T(roll),  Eigen::Matrix<T, 3, 1>::UnitX())).toRotationMatrix();
                // Return the Sophus SE3 object
                return Sophus::SE3<T>(R, t);
            }
    
        const Sophus::SE3d T_meas_;
        const Eigen::Matrix4d cov_;
        const double source_roll_, source_pitch_;
        const double target_roll_, target_pitch_;
    
    };


    struct AnchorResidual {
        AnchorResidual(const Sophus::SE3d &T_meas, const Eigen::Matrix4d &cov, 
                        double source_roll, double source_pitch, 
                        double target_roll, double target_pitch,
                        double anchor_roll_uav, double anchor_pitch_uav,
                        double anchor_roll_agv, double anchor_pitch_agv)
          : T_meas_(T_meas), cov_(cov), source_roll_(source_roll), source_pitch_(source_pitch), 
            target_roll_(target_roll), target_pitch_(target_pitch), 
            anchor_roll_uav_(anchor_roll_uav), anchor_pitch_uav_(anchor_pitch_uav),
            anchor_roll_agv_(anchor_roll_agv), anchor_pitch_agv_(anchor_pitch_agv) {}
      
        template <typename T>
        bool operator()(const T* const source_state, const T* const target_state, const T* const T_anchor_uav, const T* const T_anchor_agv, T* residual) const {
        
          // Build homogeneous transforms from the state vectors.
          Sophus::SE3<T> T_s = buildTransformationSE3(source_state, source_roll_, source_pitch_);
          Sophus::SE3<T> T_t = buildTransformationSE3(target_state, target_roll_, target_pitch_);

          // Build the extra transformation from UAV local frame into AGV frame.
          Sophus::SE3<T> anchor_T_t = buildTransformationSE3(T_anchor_uav, anchor_roll_uav_, anchor_pitch_uav_);
          Sophus::SE3<T> anchor_T_s = buildTransformationSE3(T_anchor_agv, anchor_roll_agv_, anchor_pitch_agv_);

          Sophus::SE3<T> w_T_s = anchor_T_s * T_s;
          Sophus::SE3<T> w_T_t = anchor_T_t * T_t;

          Sophus::SE3<T> T_pred = w_T_t.inverse() * w_T_s;

          Sophus::SE3<T> T_err = T_meas_.template cast<T>().inverse() * T_pred;
          Eigen::Matrix<T, 6, 1> xi = T_err.log();
      
          // Project the 6-vector error onto the 4-DOF space (translation and yaw).
          Eigen::Matrix<T, 4, 1> error_vec;
          error_vec.template head<3>() = xi.template head<3>();
          error_vec[3] = xi[5];
      
          // Scale by the square root of the inverse covariance matrix
          Eigen::LLT<Eigen::Matrix4d> chol(cov_);
          Eigen::Matrix4d sqrt_inv_cov = Eigen::Matrix4d(chol.matrixL().transpose()).inverse();
          Eigen::Matrix<T, 4, 1> weighted_residual = sqrt_inv_cov.cast<T>() * error_vec;

          // Assign to residual
          residual[0] = weighted_residual[0];
          residual[1] = weighted_residual[1];
          residual[2] = weighted_residual[2];
          residual[3] = weighted_residual[3];
      
          return true;
        }
      
        static ceres::CostFunction* Create(const Sophus::SE3d &T_meas, const Eigen::Matrix4d &cov, 
                                            double source_roll, double source_pitch, 
                                            double target_roll, double target_pitch,
                                            double anchor_roll_uav, double anchor_pitch_uav,
                                            double anchor_roll_agv, double anchor_pitch_agv) {
          return new ceres::AutoDiffCostFunction<AnchorResidual, 4, 4, 4, 4, 4>(
            new AnchorResidual(T_meas, cov, source_roll, source_pitch, target_roll, target_pitch, anchor_roll_uav, anchor_pitch_uav, anchor_roll_agv, anchor_pitch_agv));
        }
      
      private:
        // Helper function to build a 4-DOF SE3 transformation given a state and fixed roll, pitch.
        template <typename T>
        Sophus::SE3<T> buildTransformationSE3(const T* state, double roll, double pitch) const {
            // Extract translation
            Eigen::Matrix<T, 3, 1> t;
            t << state[0], state[1], state[2];
            // Build rotation from yaw, with fixed roll and pitch
            Eigen::Matrix<T, 3, 3> R = (Eigen::AngleAxis<T>(state[3], Eigen::Matrix<T, 3, 1>::UnitZ()) *
                                        Eigen::AngleAxis<T>(T(pitch), Eigen::Matrix<T, 3, 1>::UnitY()) *
                                        Eigen::AngleAxis<T>(T(roll),  Eigen::Matrix<T, 3, 1>::UnitX())).toRotationMatrix();
            // Return the Sophus SE3 object
            return Sophus::SE3<T>(R, t);
        }
      
        const Sophus::SE3d T_meas_;
        const Eigen::Matrix4d cov_;
        const double source_roll_, source_pitch_;
        const double target_roll_, target_pitch_;
        const double anchor_roll_uav_, anchor_pitch_uav_;
        const double anchor_roll_agv_, anchor_pitch_agv_;
      };
    
    struct UWBPoseGraphCostFunction {
        UWBPoseGraphCostFunction(const Eigen::Vector3d& agv_anchor_offset_bl,
                            const Eigen::Vector3d& uav_tag_offset_bl,
                            double measured_dist, double sigma,
                            double roll_agv_kf, double pitch_agv_kf,
                            double roll_uav_kf, double pitch_uav_kf,
                            double roll_Aagv,   double pitch_Aagv,
                            double roll_Auav,   double pitch_Auav)
        : a_off_(agv_anchor_offset_bl),
            t_off_(uav_tag_offset_bl),
            d_(measured_dist),
            inv_sigma_(1.0 / sigma),
            rpk_agv_(roll_agv_kf), ppk_agv_(pitch_agv_kf),
            rpk_uav_(roll_uav_kf), ppk_uav_(pitch_uav_kf),
            rpa_agv_(roll_Aagv),   ppa_agv_(pitch_Aagv),
            rpa_uav_(roll_Auav),   ppa_uav_(pitch_Auav) {}

        template <typename T>
        bool operator()(const T* const A_agv,  // [x,y,z,yaw]  AGV-odom -> World (anchor node)
                        const T* const A_uav,  // [x,y,z,yaw]  UAV-odom -> World (anchor node)
                        const T* const X_i,    // [x,y,z,yaw]  AGV base_link in AGV-odom (KF i)
                        const T* const Y_j,    // [x,y,z,yaw]  UAV base_link in UAV-odom (KF j)
                        T* residual) const {

            // Build the four SE3s (4-DoF yaw+xyz, fixed roll/pitch)
            const Sophus::SE3<T> Tw_Aagv = buildTransformationSE3_(A_agv, rpa_agv_, ppa_agv_); // World <- AGV-odom
            const Sophus::SE3<T> Tw_Auav = buildTransformationSE3_(A_uav, rpa_uav_, ppa_uav_); // World <- UAV-odom
            const Sophus::SE3<T> To_Xi   = buildTransformationSE3_(X_i,   rpk_agv_, ppk_agv_); // AGV-odom <- AGV base_link
            const Sophus::SE3<T> To_Yj   = buildTransformationSE3_(Y_j,   rpk_uav_, ppk_uav_); // UAV-odom <- UAV base_link

            // Fixed hardware offsets in base_link -> lift to T
            const Eigen::Matrix<T,3,1> pa_bl(T(a_off_.x()), T(a_off_.y()), T(a_off_.z()));
            const Eigen::Matrix<T,3,1> pt_bl(T(t_off_.x()), T(t_off_.y()), T(t_off_.z()));

            // Compose: World points of anchor/tag hardware
            // World <- AGV-odom <- base_link * offset
            const Eigen::Matrix<T,3,1> pw_anchor = (Tw_Aagv * To_Xi) * pa_bl;
            // World <- UAV-odom <- base_link * offset
            const Eigen::Matrix<T,3,1> pw_tag    = (Tw_Auav * To_Yj) * pt_bl;

            // Range residual
            const Eigen::Matrix<T,3,1> diff = pw_anchor - pw_tag;
            const T pred = ceres::sqrt(diff.squaredNorm());
            residual[0] = (pred - T(d_)) * T(inv_sigma_);
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector3d& agv_anchor_offset_bl,
                                            const Eigen::Vector3d& uav_tag_offset_bl,
                                            double d, double sigma,
                                            double roll_agv_kf, double pitch_agv_kf,
                                            double roll_uav_kf, double pitch_uav_kf,
                                            double roll_Aagv,   double pitch_Aagv,
                                            double roll_Auav,   double pitch_Auav) {
            return (new ceres::AutoDiffCostFunction<UWBPoseGraphCostFunction, 1, 4,4,4,4>(
            new UWBPoseGraphCostFunction(agv_anchor_offset_bl, uav_tag_offset_bl, d, sigma,
                                    roll_agv_kf, pitch_agv_kf,
                                    roll_uav_kf, pitch_uav_kf,
                                    roll_Aagv,   pitch_Aagv,
                                    roll_Auav,   pitch_Auav)));
        }

        private:

            template <typename T>
            Sophus::SE3<T> buildTransformationSE3_(const T* state, double roll, double pitch) const {
                // Your helper (kept verbatim, just inlined here)
                Eigen::Matrix<T,3,1> t; t << state[0], state[1], state[2];
                Eigen::Matrix<T,3,3> R =
                    (Eigen::AngleAxis<T>(state[3], Eigen::Matrix<T,3,1>::UnitZ()) *
                    Eigen::AngleAxis<T>(T(pitch),  Eigen::Matrix<T,3,1>::UnitY()) *
                    Eigen::AngleAxis<T>(T(roll),   Eigen::Matrix<T,3,1>::UnitX())).toRotationMatrix();
                return Sophus::SE3<T>(R, t);
            }

            // Data
            Eigen::Vector3d a_off_, t_off_;
            double d_, inv_sigma_;
            // Fixed roll/pitch per block
            double rpk_agv_, ppk_agv_, rpk_uav_, ppk_uav_;
            double rpa_agv_, ppa_agv_, rpa_uav_, ppa_uav_;
        };

}


struct ResidualLogger : public ceres::IterationCallback {
  explicit ResidualLogger(ceres::Problem& problem) : problem_(problem) {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    std::vector<double> residuals;
    problem_.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, &residuals, nullptr, nullptr);
    RCLCPP_INFO(rclcpp::get_logger("uwb_opt"), "Residuals: %zu values", residuals.size());
    for (size_t i = 0; i < residuals.size(); i++) {
      RCLCPP_INFO(rclcpp::get_logger("uwb_opt"), "  res[%zu] = %.6f", i, residuals[i]);
    }
    return ceres::SOLVER_CONTINUE;
  }
  ceres::Problem& problem_;
};

#endif