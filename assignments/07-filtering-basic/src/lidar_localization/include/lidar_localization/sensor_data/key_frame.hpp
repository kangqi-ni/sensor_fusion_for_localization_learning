#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class KeyFrame {
  public:
    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;
};
}

#endif