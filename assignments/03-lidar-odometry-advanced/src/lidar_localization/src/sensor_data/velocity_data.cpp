#include "lidar_localization/sensor_data/velocity_data.hpp"

#include "glog/logging.h"

namespace lidar_localization {
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time) {
    while (UnsyncedData.size() >= 2UL) {
        const auto& prev_data = UnsyncedData.front();
        const auto& next_data = UnsyncedData.at(1);

        // LOG(WARNING) << "Sync ..." << sync_time - prev_data.time << ", " << next_data.time - sync_time << std::endl;
        // get current data:  
        if (prev_data.time > sync_time) {
            return false;
        }

        // get next data:
        if (next_data.time < sync_time) {
            UnsyncedData.pop_front();
        } else {
            break;
        }
    }

    if (UnsyncedData.size() < 2UL) {
        return false;
    }

    const auto& prev_data = UnsyncedData.front();
    const auto& next_data = UnsyncedData.at(1);
    VelocityData synced_data;

    double front_scale = (next_data.time - sync_time) / (next_data.time - prev_data.time);
    double back_scale = (sync_time - prev_data.time) / (next_data.time - prev_data.time);
    synced_data.time = sync_time;
    synced_data.linear_velocity.x = prev_data.linear_velocity.x * front_scale + next_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y = prev_data.linear_velocity.y * front_scale + next_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z = prev_data.linear_velocity.z * front_scale + next_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x = prev_data.angular_velocity.x * front_scale + next_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = prev_data.angular_velocity.y * front_scale + next_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = prev_data.angular_velocity.z * front_scale + next_data.angular_velocity.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();

    Eigen::Matrix3d R = matrix.block<3,3>(0,0);
    Eigen::Vector3d t = matrix.block<3,1>(0,3);

    // get angular & linear velocities in IMU frame:
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);

    // a. first, add velocity component generated by rotation:
    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * t(2) - w(2) * t(1);
    delta_v(1) = w(2) * t(0) - w(0) * t(2);
    delta_v(2) = w(0) * t(1) - w(1) * t(0);
    v += delta_v;

    // b. transform velocities in IMU frame to lidar frame:
    w = R.transpose() * w;
    v = R.transpose() * v;

    // finally:
    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}

}