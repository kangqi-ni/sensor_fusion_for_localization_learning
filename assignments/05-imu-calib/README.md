# Multi-Sensor Fusion for Localization and Mapping: IMU Calibration

The derivation of the accelerometer error jacobian can be found in accelerometer_error_jacobian.pdf (https://github.com/kangqi-ni/sensor_fusion_for_localization_learning/blob/master/assignments/05-imu-calib/accelerometer_error_jacobian.pdf)


Calibration Result of the Example:

Accelerometer:
'''
          1          -0           0
-0.00354989           1          -0
-0.00890444  -0.0213032           1

0.00241267          0          0
         0 0.00242659          0
         0          0 0.00241232

33124.2
33275.2
32364.4
'''

Gyroscope:
'''
         1 0.00927517 0.00990014
0.00507442          1 -0.0322229
 0.0162201 -0.0239393          1

0.000209338           0           0
          0 0.000209834           0
          0           0 0.000209664

32777.1
32459.8
32511.8
'''
