#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "1234" | sudo ip link set down can2
echo "1234" | sudo ip link set can2 up type can bitrate 1000000  

source /home/roma/asoc-roma/devel/setup.bash
# roslaunch tmotor_leg tmotor2_uppercontroller1.launch 
roslaunch tmotor_leg tmotor_controller3.launch

# {
# gnome-terminal -t "start_tmotor_controller" -x bash -c "cd ~/asoc-roma;source devel/setup.bash;roslaunch tmotor_leg tmotor_auto_indoor.launch;exec bash"
# }&

# sleep 0.1
# echo "tmotor starting success!"
# wait
# exit 0