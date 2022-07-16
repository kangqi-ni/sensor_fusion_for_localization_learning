#include "lidar_localization/sensor_data/loop_pose.hpp"

namespace lidar_localization {
Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}