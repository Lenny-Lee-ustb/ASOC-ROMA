## 使能串口
sudo ip link set can0 type can bitrate 1000000  
sudo ip link set up can0  
## 运行demo(给电机发送一个正弦位置信号，对电机进行位置环控制)
roscore  
rosrun tmotortest rostest1  

## 关于设置电机id：打开串口调试助手，波特率设置为921600, 按ESC，会有交互界面（发送s进入调式模式，输入iX+Enter，发送。x为电机id）

运行：
串口使能；
打开joy node(插上xbox手柄usb)：
rosrun joy joy_node
工作空间下：
catkin_make
source devel/setup.bash
rosrun tmotor_leg tmotortest3
xbox操作请自行探索

