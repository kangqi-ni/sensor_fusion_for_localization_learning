// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#ifndef LIDAR_LOCALIZATION_MODELS_ALOAM_FACTOR_HPP
#define LIDAR_LOCALIZATION_MODELS_ALOAM_FACTOR_HPP

//
// TODO: implement analytic Jacobians for LOAM residuals in this file
// 

#include <ceres/local_parameterization.h>
#include <ceres/sized_cost_function.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>

//
// TODO: Sophus is ready to use if you have a good undestanding of Lie algebra.
// 
#include <sophus/so3.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


Eigen::Matrix<double,3,3> skew(const Eigen::Vector3d &mat_in);

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};

struct LidarDistanceFactor
{
	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_) 
						: curr_point(curr_point_), closed_point(closed_point_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};

struct LidarEdgeFactorAnalytic: public ceres::SizedCostFunction<1, 4, 3> 
{
	LidarEdgeFactorAnalytic(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
	{
		Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);

		Eigen::Vector3d lp = q_last_curr * curr_point + t_last_curr;
		Eigen::Vector3d nu = (lp - last_point_b).cross(lp - last_point_a);
		Eigen::Vector3d de = last_point_a - last_point_b;

		residuals[0] = nu.norm() / de.norm();   

		if (jacobians != nullptr) {
			if (jacobians[0] != nullptr) {
				// Rotation
				Eigen::Matrix<double,3,3> jacobian_nu_lp = skew(de) / de.norm();
				Eigen::Matrix<double,1,3> jacobian_error_nu = nu.transpose() / nu.norm();
				Eigen::Matrix<double,3,3> jacobian_lp_so3 = - skew(q_last_curr * curr_point);
				
				Eigen::Map<Eigen::Matrix<double,1,4>> jacobian_error_so3 (jacobians[0]);
				jacobian_error_so3.setZero();
				jacobian_error_so3.block<1,3>(0,0) = jacobian_error_nu * jacobian_nu_lp * jacobian_lp_so3;

 				// Translation
				Eigen::Map<Eigen::Matrix<double,1,3>> jacobian_error_xyz (jacobians[1]);
				jacobian_error_xyz.setZero();
				jacobian_error_xyz = jacobian_error_nu * jacobian_nu_lp * Eigen::Matrix<double,3,3>::Identity();
			}
		}

		return true;
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

struct LidarPlaneFactorAnalytic: public ceres::SizedCostFunction<1, 4, 3> 
{
	LidarPlaneFactorAnalytic(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm = (last_point_l - last_point_j).cross(last_point_m - last_point_j);
		ljm.normalize();
	}

	virtual bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const 
	{
		Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);

		Eigen::Vector3d lp = q_last_curr * curr_point + t_last_curr;

		double d = (lp - last_point_j).dot(ljm);
		residuals[0] = abs(d);
		
		if (jacobians != nullptr) {
			if (jacobians[0] != nullptr) {
				// Rotation
				double jacobian_error_d = d / abs(d);
				Eigen::Matrix<double,3,3> jacobian_lp_so3 = - skew(q_last_curr * curr_point);
				
				Eigen::Map<Eigen::Matrix<double,1,4>> jacobian_error_so3(jacobians[0]);
				jacobian_error_so3.setZero();
				jacobian_error_so3.block<1,3>(0,0) = jacobian_error_d * ljm.transpose() * jacobian_lp_so3;

				// Translation
				Eigen::Map<Eigen::Matrix<double,1,3>> jacobian_error_t(jacobians[1]);
				jacobian_error_t.setZero();
				jacobian_error_t = jacobian_error_d * ljm.transpose() * Eigen::Matrix<double,3,3>::Identity();
			}
		}
		
		return true;
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm;
	double s;
};

class PoseSO3Parametrization: public ceres::LocalParameterization
{
public:
	PoseSO3Parametrization() {}

	virtual ~PoseSO3Parametrization() {}

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const 
	{
		// Quaternion to be updated
        Eigen::Map<const Eigen::Quaterniond> quat(x);     
		// Delta value so3  
        Eigen::Map<const Eigen::Vector3d> delta_so3(delta);  
		// Delta value quaternion
		Eigen::Quaterniond delta_quat = Sophus::SO3d::exp(delta_so3).unit_quaternion();
		// Update the quaternion
		Eigen::Map<Eigen::Quaterniond> quat_plus(x_plus_delta);    //   更新后的四元数
		quat_plus = (delta_quat * quat).normalized();    

		return true;
	}

	virtual bool ComputeJacobian(const double *x, double *jacobian) const 
	{
		// Compute jacobian of quaternion wrt so3
		Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
        (j.topRows(3)).setIdentity();
        (j.bottomRows(1)).setZero();
		return true;
	}
	
    virtual int GlobalSize() const {return  4;} // dimension of quaternion
    virtual int LocalSize() const {return  3;} // dimension of so3
};

#endif // LIDAR_LOCALIZATION_MODELS_ALOAM_FACTOR_HPP