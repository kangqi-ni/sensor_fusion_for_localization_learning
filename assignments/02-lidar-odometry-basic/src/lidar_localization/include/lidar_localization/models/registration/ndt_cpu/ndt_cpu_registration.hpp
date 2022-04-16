#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"

#include "lidar_localization/models/registration/ndt_cpu/NormalDistributionsTransform.h"

namespace lidar_localization {
class NDTCPURegistration: public RegistrationInterface {
  public:
    NDTCPURegistration(const YAML::Node& node);
    NDTCPURegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    std::shared_ptr<cpu::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>> ndt_cpu_ptr_;
};
}

#endif 