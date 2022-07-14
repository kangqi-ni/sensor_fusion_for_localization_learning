#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_svd_registration.hpp"

namespace lidar_localization {

ICPSVDRegistration::ICPSVDRegistration(
    const YAML::Node& node
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    // parse params:
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPSVDRegistration::ICPSVDRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPSVDRegistration::SetRegistrationParam(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) {
    // set params:
    max_corr_dist_ = max_corr_dist;
    trans_eps_ = trans_eps;
    euc_fitness_eps_ = euc_fitness_eps;
    max_iter_ = max_iter;

    LOG(INFO) << "ICP SVD params:" << std::endl
              << "max_corr_dist: " << max_corr_dist_ << ", "
              << "trans_eps: " << trans_eps_ << ", "
              << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
              << "max_iter: " << max_iter_ 
              << std::endl << std::endl;

    return true;
}

bool ICPSVDRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

bool ICPSVDRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    // init estimation:
    transformation_.setIdentity();
    
    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    int curr_iter = 0;
    while (curr_iter < max_iter_) {
        // TODO: apply current estimation:
        CloudData::CLOUD_PTR curr_input_source(new CloudData::CLOUD());
        pcl::transformPointCloud(*transformed_input_source, *curr_input_source, transformation_);

        // TODO: get correspondence:
        std::vector<Eigen::Vector3f> xs, ys;
        int num_correspondences = GetCorrespondence(curr_input_source, xs, ys);

        // TODO: do not have enough correspondence -- break:
        if (num_correspondences < 5) {
            break;
        }

        // TODO: update current transform:
        Eigen::Matrix4f transform_update;
        GetTransform(xs, ys, transform_update);

        // TODO: whether the transformation update is significant:
        bool is_significant = IsSignificant(transform_update, trans_eps_);

        if (!is_significant) {
            break;
        }

        // TODO: update transformation:
        transformation_ = transform_update * transformation_;

        ++curr_iter;
    }

    // set output:
    result_pose = transformation_ * predict_pose;

    // normalize the quaternion
    Eigen::Quaternionf q(result_pose.block<3,3>(0,0));
    q.normalize();
    Eigen::Vector3f t = result_pose.block<3,1>(0,3);
    result_pose.setIdentity();
    result_pose.block<3,3>(0,0) = q.toRotationMatrix();
    result_pose.block<3,1>(0,3) = t;

    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

size_t ICPSVDRegistration::GetCorrespondence(
    const CloudData::CLOUD_PTR &input_source, 
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys
) {
    // Notice: treat xs as target points and ys as source points

    const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

    size_t num_corr = 0;

    // TODO: set up point correspondence
    for (const CloudData::POINT &p: *input_source) { 
        // Find the nearest neighboring point     
        std::vector<int> indices;
        std::vector<float> k_sqr_distances;  
        int num_neighbors = input_target_kdtree_->nearestKSearch(p, 1, indices, k_sqr_distances);

        if (num_neighbors == 0) {
            continue;
        }
        if (k_sqr_distances[0] > MAX_CORR_DIST_SQR) {
            continue;
        }

        // Get the corresponding point 
        CloudData::POINT p_prime = input_target_->points[indices[0]];

        // Add the target point
        xs.push_back(Eigen::Vector3f(p_prime.x, p_prime.y, p_prime.z));   

        // Add the source point
        ys.push_back(Eigen::Vector3f(p.x, p.y, p.z));

        num_corr++;   
    }
    
    return num_corr;
}

void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();

    // TODO: find centroids of mu_x and mu_y:
    Eigen::Vector3f mu_x = Eigen::Vector3f::Zero(); 
    for (const Eigen::Vector3f &x: xs) {
        mu_x += x;
    }
    mu_x /= N;
    Eigen::Vector3f mu_y = Eigen::Vector3f::Zero();
    for (const Eigen::Vector3f &y:ys) {
        mu_y += y;
    }
    mu_y /= N;

    // TODO: build H:
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        Eigen::Vector3f y_prime = ys[i] - mu_y;
        Eigen::Vector3f x_prime = xs[i] - mu_x;
        H += y_prime * x_prime.transpose();
    }

    // TODO: solve R:
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0) {
        Eigen::Matrix3f V = svd.matrixV();
        V(0,2) *= -1;
        V(1,2) *= -1;
        V(2,2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    // TODO: solve t:
    Eigen::Vector3f t = mu_x - R * mu_y;

    // TODO: set output:
    transformation_ = Eigen::Matrix4f::Identity();
    transformation_.block<3,3>(0,0) = R;
    transformation_.block<3,1>(0,3) = t;
}

bool ICPSVDRegistration::IsSignificant(
    const Eigen::Matrix4f &transformation,
    const float trans_eps
) {
    // a. translation magnitude -- norm:
    float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
    // b. rotation magnitude -- angle:
    float rotation_magnitude = fabs(
        acos(
            (transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f
        )
    );

    return (
        (translation_magnitude > trans_eps) || 
        (rotation_magnitude > trans_eps)
    );
}

float ICPSVDRegistration::GetFitnessScore() {
    return 0.0f;
}

} // namespace lidar_localization