#!/bin/bash
source /opt/ros/noetic/setup.bash

echo "1234" | sudo ip link set down can0 
echo "1234" | sudo ip link set down can1
echo "1234" | sudo ip link set can0 up type can bitrate 1000000
echo "1234" | sudo ip link set can1 up type can bitrate 1000000

source /home/roma/asoc-roma/devel/setup.bash
roslaunch asoc_lower_controller asoc_lower_platform.launch

# {
# gnome-terminal -t "start_lower_platform" -x bash -c "cd ~/asoc-roma;source devel/setup.bash;roslaunch asoc_lower_controller asoc_lower_platform.launch;exec bash"
# }&

# sleep 0.1
# echo "asoc_lower_controller starting success!"
# wait
# exit 0
