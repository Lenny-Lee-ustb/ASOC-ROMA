#!/bin/bash
source /opt/ros/noetic/setup.bash
{
gnome-terminal -t "roabag" -x bash -c "rosbag record /MultiAngleSum /Tmotor_Info /cmd_vel /odometry/filtered /suspension_cmd /leg_angle_high /leg_angle_low /velocity_high /velocity_low"
}&

sleep 0.1
echo "rosbag starting success!"
wait
exit 0
