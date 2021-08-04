#!/bin/bash
source /opt/ros/melodic/setup.bash

#gnome-terminal -x bash -c "roscore" & 
#sleep 2

#echo "0515haichuan" | sudo -S sudo su
#cd ~/catkin_encoder
#source devel/setup.bash
#rosrun SpimTest demo1
#{
#gnome-terminal -t "start_encoder" -x bash -c "cd ~/catkin_encoder;echo 0515haichuan | sudo -#S;source devel/setup.bash;rosrun SpimTest demo1;exec bash"
#}&
#rosrun SpimTest demo1 &
#sleep 2
#echo "encoder starting success!"
echo "1234" | sudo ip link set can0 type can bitrate 1000000
echo "1234" | sudo ip link set up can0
echo "1234" | sudo ip link set can1 type can bitrate 1000000
echo "1234" | sudo ip link set up can1
{
gnome-terminal -t "start_lower_controller" -x bash -c "cd ~/asoc_roma;source devel/setup.bash;roslaunch asoc_lower_controller asoc_lower_controller.launch;exec bash"
}&
#roslaunch asoc_lower_controller asoc_lower_controller.launch &
sleep 0.1
echo "asoc_lower_controller starting success!"
wait
exit 0
