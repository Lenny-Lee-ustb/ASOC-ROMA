#!/bin/bash

echo 1234 |sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can1
echo 1234 | sudo -s
source devel/setup.bash
roslaunch SpimTest asoc_encoder.launch &
sleep 2
echo "encoder startgin success!"

roslaunch asoc_lower_controller asoc_lower_controller.launch &
sleep 0.1
echo "asoc_lower_controller starting success!"
wait
exit 0
