#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include "imu_integration/evaluator/activity.hpp"

int main(int argc, char** argv) {
    std::string node_name{"imu_integration_evaluator_node"};
    ros::init(argc, argv, node_name);
    
    imu_integration::evaluator::Activity activity;
    
    // 100 Hz:
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    } 

    return EXIT_SUCCESS;
}