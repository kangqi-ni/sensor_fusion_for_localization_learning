# Multi-Sensor Fusion for Localization & Mapping: Graph Optimization 

## IMU Preintegration

The derivation of Jacobians can be found in imu_preintegration_jacobian.pdf (https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/imu_preintegration_jacobian.pdf)

![map_imu_preintegration](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/map_imu_preintegration.png)

![traj_imu_preintegration](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/traj_imu_preintegration.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/ape_imu_preintegration.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/imu_preintegration_raw.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/imu_preintegration_map.png">

## No IMU Preintegration

![map_imu_preintegration](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/map_no_imu_preintegration.png)

![traj_imu_preintegration](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/traj_no_imu_preintegration.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/ape_no_imu_preintegration.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/no_imu_preintegration_raw.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/docs/no_imu_preintegration_map.png">

The experiments show that imu preintegration does not improve the trajectory acccuracy overall. The possible reasons can be the kitti dataset itself. However, it is still obvious that the optimized trajectory is much more accurate than laser odometry from the rviz visualization. 

## IMU and Wheel Encoder Fusion Preintegration

The full derivation of residuals, Jacobians, and covariances can be found in imu_and_wheel_encoder_preintegration.pdf (https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/09-graph-optimization/imu_and_wheel_encoder_preintegration.pdf)
