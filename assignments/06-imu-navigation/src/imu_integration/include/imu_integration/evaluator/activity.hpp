#ifndef IMU_INTEGRATION_EVALUATOR_HPP_
#define IMU_INTEGRATION_EVALUATOR_HPP_

#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "imu_integration/config/config.hpp"

#include "imu_integration/subscriber/imu_subscriber.hpp"
#include "imu_integration/subscriber/odom_subscriber.hpp"


namespace imu_integration {

namespace evaluator {

class Activity {
public:
    Activity(void);

private:
    bool CreateFiles(const std::string &dir_path);

    void EvaluationCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void GTCallBack(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle private_nh_;

    std::string dir_path;

    ros::Subscriber sub_eva;
    ros::Subscriber sub_gt;

    std::deque<OdomData> eva_buff_;
    std::deque<OdomData> gt_buff_;

    OdomConfig config_;

    std::ofstream gt_writter_;
    std::ofstream eva_writter_; 
};

} // evaluator

} // imu_integration

#endif // IMU_INTEGRATION_EVALUATOR_HPP_