#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class LoopPose {
  public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif