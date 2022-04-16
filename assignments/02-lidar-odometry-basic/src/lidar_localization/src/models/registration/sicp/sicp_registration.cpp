#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "glog/logging.h"

#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

#include "lidar_localization/models/registration/sicp/ICP.h"

namespace lidar_localization {

SICPRegistration::SICPRegistration(
    const YAML::Node& node
) {
    // parse params:
    SICP::Parameters params = SICP::Parameters();

    // params_.p = node["p"].as<float>();
    // params_.mu = node["mu"].as<float>();
    // params_.alpha = node["alpha"].as<float>();
    // params_.max_mu = node["max_mu"].as<float>();
    // params_.max_icp = node["max_icp"].as<int>();
    // params_.max_outer = node["max_outer"].as<int>();
    // params_.max_inner = node["max_inner"].as<int>();
    // params_.stop = node["stop"].as<float>();

    LOG(INFO) << "SICP Parameters:" << std::endl
              << "p: " << params.p << ", "
              << "mu: " << params.mu << ", "
              << "alpha: " << params.alpha << ", "
              << "max_mu: " << params.max_mu << ", "
              << "max_icp: " << params.max_icp << ", "
              << "max_outer: " << params.max_outer << ","
              << "max_inner: " << params.max_inner << ", "
              << "stop: " << params.stop 
              << std::endl << std::endl;
}

bool SICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;

    return true;
}

bool SICPRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    //
    // TODO: second option -- adapt existing implementation
    //
    // TODO: format inputs for SICP:


    Eigen::Matrix3Xd X (3, transformed_input_source->size()); // source, transformed
    Eigen::Matrix3Xd Y (3, input_target_->size()); // target

    for (size_t i = 0; i < transformed_input_source->size(); ++i) {
        X(0,i) = transformed_input_source->points[i].x;
        X(1,i) = transformed_input_source->points[i].y;
        X(2,i) = transformed_input_source->points[i].z;
    }
    for (size_t i = 0; i < input_target_->size(); ++i) {
        Y(0,i) = input_target_->points[i].x;
        Y(1,i) = input_target_->points[i].y;
        Y(2,i) = input_target_->points[i].z;
    }
    
    // TODO: SICP registration:
    Eigen::Affine3d transform = SICP::point_to_point(X, Y);

    transformation_ << transform(0,0), transform(0,1), transform(0,2), transform(0,3),  
                       transform(1,0), transform(1,1), transform(1,2), transform(1,3),
                       transform(2,0), transform(2,1), transform(2,2), transform(2,3),
                       transform(3,0), transform(3,1), transform(3,2), transform(3,3);

    // set output:
    result_pose = transformation_ * predict_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

} // namespace lidar_localization