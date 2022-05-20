#ifndef LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_
#define LIDAR_LOCALIZATION_MATCHING_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"

namespace lidar_localization {
class Matching {
  public:
    Matching();

    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);

    bool SetGNSSPose(const Eigen::Matrix4f& init_pose);
    bool SetScanContextPose(const CloudData& init_scan);

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetInited(void);

    Eigen::Matrix4f GetInitPose(void);
    void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
    CloudData::CLOUD_PTR& GetLocalMap();
    CloudData::CLOUD_PTR& GetCurrentScan();
    bool HasInited();
    bool HasNewGlobalMap();
    bool HasNewLocalMap();

  private:
    bool InitWithConfig();
    bool InitDataPath(const YAML::Node& config_node);
    bool InitScanContextManager(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool InitBoxFilter(const YAML::Node& config_node);

    bool InitGlobalMap();
    bool ResetLocalMap(float x, float y, float z);

  private:
    std::string scan_context_path_ = "";
    std::string map_path_ = "";

    std::string loop_closure_method_ = "";

    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> box_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};
}

#endif