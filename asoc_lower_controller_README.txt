asoc_lower_controller.sh needs authority to run:
	sudo chmod a+x asoc_lower_controller.sh

SpimTest demo1 needs root authority to run:
	sudo su

asoc_lower_controller has 2 CANs, and they need to be initialized as follow:
	sudo ip link set can0 type can bitrate 1000000
	sudo ip link set up can0
	sudo ip link set can1 type can bitrate 1000000
	sudo ip link set up can1
FYI: the first USB you plug in will be CAN0, which shall be used as CAN_low for 4 low motors, and the second USB you plug in will be CAN1, which shall be used as CAN_high for 4 high motors.

steps to run the program:
1. sudo -s
2. rosrun SpimTest demo1
3.rosparam get ori0 (see if you get the latest ori0, if so, go to the next step, if not, repeat until you get it)
4.~/catkin_ws/asoc_lower_controller.sh start
