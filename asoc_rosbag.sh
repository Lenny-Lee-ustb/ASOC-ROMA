#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/roma/asoc-roma/devel/setup.bash
rosbag record -o outdoor /MultiAngleSum /Tmotor_Info /cmd_vel /GPS_odom /suspension_cmd /leg_angle_high /leg_angle_low /velocity_high /velocity_low

