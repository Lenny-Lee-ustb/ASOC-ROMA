#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/roma/asoc-roma/devel/setup.bash
# rosbag record -o playground_A_20_15_7_20_10 /Tmotor_Info /cmd_vel /GPS_odom /imu_data_leg_0 /imu_rpy0 /suspension_cmd /leg_angle_high /leg_angle_low /Roll_high /Roll_low
# rosbag record -o artificial_AP_5_0_0_0_0 /Tmotor_Info /cmd_vel /GPS_odom /imu_data_leg_0 /imu_rpy0 /suspension_cmd /leg_angle_high /leg_angle_low /Roll_high /Roll_low
# rosbag record -o indoor_test1 /Tmotor_Info /imu_data_leg_0 /imu_rpy0 /suspension_cmd /leg_angle_high /leg_angle_low
rosbag record -o 0802_no15_in_nocurve_30_0.7_p_30_20_27_25_noslow /odometry/filtered /MultiAngle /MultiTurn /MultiAngleSum /Tmotor_Info /cmd_vel /GPS_odom /suspension_cmd /imu_data_leg_1 /imu_rpy1 /imu_data_leg_0 /imu_rpy0 /leg_angle_high /leg_angle_low /Roll_high /Roll_low /velocity_high /velocity_low /I_high /I_low


# 时间--实验编号--室内、室外--地图类型--速度--前瞻点--悬挂模式--p_lateral--d_lateral--p_yaw--d_yaw
# 地形--悬架配置--速度_p_lateral--p_lateral--p_yaw--d_yaw
# 主动悬架的配置是统一的，只用调节一次
# ls -l /dev |grep ttyUSB
# sudo chmod 777 /dev/ttyUSB2
# roslaunch wt931 Wt906_0.launch 