#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGRelativePose : public ceres::SizedCostFunction<6, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;

  FactorPRVAGRelativePose(void) {};

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);

    //
    // parse measurement:
    // 
		const Eigen::Vector3d     &pos_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d         ori_ij = Sophus::SO3d::exp(log_ori_ij);

    //
    // TODO: get square root of information matrix:
    //
    Eigen::Matrix<double,6,6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6 ,6>>(I_).matrixL().transpose() ;

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double,6,1>> residuals_eigen(residuals);
    residuals_eigen.block<3,1>(INDEX_P, 0) = ori_i.inverse() * (pos_j - pos_i) - pos_ij;
    residuals_eigen.block<3,1>(INDEX_R, 0) = (ori_i.inverse() * ori_j * ori_ij.inverse()).log();
    
    //
    // TODO: compute jacobians:
    //
    if ( jacobians ) {
      // compute shared intermediate results:
      const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
      const Eigen::Matrix3d J_r_inv = JacobianRInv(residuals_eigen.block(INDEX_R, 0 ,3 , 1));

      if ( jacobians[0] ) {
        // implement computing:
        Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> jacobians_i (jacobians[0]);
        jacobians_i.setZero();

        jacobians_i.block<3,3>(INDEX_P, INDEX_P) = - R_i_inv;
        jacobians_i.block<3,3>(INDEX_P, INDEX_R) =  Sophus::SO3d::hat(ori_i.inverse() * (pos_j - pos_i)).matrix();
        jacobians_i.block<3,3>(INDEX_R, INDEX_R) = - J_r_inv * (ori_ij * ori_j.inverse() * ori_i).matrix();
        jacobians_i = sqrt_info * jacobians_i ; 
      }

      if ( jacobians[1] ) {
        // implement computing:
        Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> jacobians_j (jacobians[1]);
        jacobians_j.setZero();

        jacobians_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;
        jacobians_j.block<3, 3>(INDEX_R,INDEX_R)  = J_r_inv * ori_ij.matrix();

        jacobians_j = sqrt_info * jacobians_j ;
      }
    }

    //
    // TODO: correct residual by square root of information matrix:
    //
    residuals_eigen = sqrt_info * residuals_eigen;

    return true;
  }

private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
      Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if ( theta > 1e-5 ) {
          // Eigen::Vector3d k = w.normalized();
          // Eigen::Matrix3d K = Sophus::SO3d::hat(k);
          
          // J_r_inv = J_r_inv 
          //           + 0.5 * K
          //           + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
      
          Eigen::Vector3d a = w.normalized();
          Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
          double theta_half = 0.5 * theta;
          double cot_theta = 1.0 / tan(theta_half);

          J_r_inv = theta_half * cot_theta * J_r_inv
                  + (1.0 - theta_half * cot_theta) * a * a.transpose()
                  + theta_half * a_hat;      
      }

      return J_r_inv;
  }
  
  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
