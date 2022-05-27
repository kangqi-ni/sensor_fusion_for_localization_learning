# Multi-Sensor Fusion for Localization and Mapping: IMU Navigation

## Example Midpoint

![example_midpoint](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_midpoint.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_midpoint_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_midpoint_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_midpoint_raw.png">

## Example Euler

![example_euler](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_euler.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_euler_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_euler_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/example_euler_raw.png">

For the example data, the midpoint method performs better than the euler method.

## Static Motion Midpoint

![static_motion_midpoint](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_midpoint.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_midpoint_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_midpoint_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_midpoint_raw.png">

## Static Motion Euler

![static_motion_euler](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_euler.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_euler_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_euler_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/static_motion_euler_raw.png">

For static motion, both methods perform about the same.

## Const Motion Midpoint

![const_motion_midpoint](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_midpoint.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_midpoint_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_midpoint_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_midpoint_raw.png">

## Const Motion Euler

![const_motion_euler](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_euler.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_euler_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_euler_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_motion_euler_raw.png">

For constant motion, both methods perform about the same.

## Const Acceleration Midpoint

![const_acc_midpoint](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_midpoint.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_midpoint_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_midpoint_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_midpoint_raw.png">

## Const Acceleration Euler

![const_acc_midpoint](https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_euler.png)

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_euler_ape.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_euler_map.png">

<img src="https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/06-imu-navigation/docs/const_acc_euler_raw.png">

For const acceleration, the euler method performs only slightly better.

From the experiments, both the midpoint and euler methods perform about the same when acceleration is relatively constant. The midpoint method performs better when the motion is more unpredictable.
