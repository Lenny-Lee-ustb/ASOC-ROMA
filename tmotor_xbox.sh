#!/bin/bash
source /opt/ros/noetic/setup.bash
cd ~/asoc-roma

echo "1234" | sudo ip link set can0 type can bitrate 1000000  
echo "1234" | sudo ip link set up can0

source devel/setup.bash
roslaunch tmotor_leg tmotor_xbox.launch;