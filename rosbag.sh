#!/bin/bash
source /opt/ros/noetic/setup.bash
{
gnome-terminal -t "roabag" -x bash -c "rosbag record /MultiAngleSum /Tmotor_Info /cmd_vel /GPS_odom /suspension_cmd /imu_data_leg1 /imu_rpy1 /imu_data_leg0 /imu_rpy0 /leg_angle_high /leg_angle_low /Roll_high /Roll_low /velocity_high /velocity_low /I_high /I_low"
}&

# gnome-terminal -t "roabag" -x bash -c "rosbag record /MultiAngleSum /Tmotor_Info /cmd_vel /GPS_odom /suspension_cmd /imu_data_leg_1 /imu_rpy1 /imu_data_leg_0 /imu_rpy0 /leg_angle_high /leg_angle_low /Roll_high /Roll_low /velocity_high /velocity_low /I_high /I_low"


sleep 0.1
echo "rosbag starting success!"
wait
exit 0
