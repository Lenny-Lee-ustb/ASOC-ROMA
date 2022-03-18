#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/roma/asoc-roma/devel/setup.bash
rosbag record -o playground_A_20_15_7_20_10 /Tmotor_Info /cmd_vel /GPS_odom /imu_data_leg_0 /imu_rpy0 /suspension_cmd /leg_angle_high /leg_angle_low /Roll_high /Roll_low
# rosbag record -o artificial_AP_5_0_0_0_0 /Tmotor_Info /cmd_vel /GPS_odom /imu_data_leg_0 /imu_rpy0 /suspension_cmd /leg_angle_high /leg_angle_low /Roll_high /Roll_low

# 地形--悬架配置--速度_p_lateral--d_lateral--p_yaw--d_yaw
# 主动悬架的配置是统一的，只用调节一次