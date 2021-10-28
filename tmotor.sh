#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "1234" | sudo ip link set can2 type can bitrate 1000000  
echo "1234" | sudo ip link set up can2

source devel/setup.bash
roslaunch tmotor_leg tmotor_control.launch;

sleep 0.1
echo "tmotor starting success!"
wait
exit 0