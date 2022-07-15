#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {

class PoseData {
  public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();

  public:
    Eigen::Quaternionf GetQuaternion();
};

}

#endif