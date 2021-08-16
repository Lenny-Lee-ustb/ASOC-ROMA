simple test：  
## 创建简单工作空间

mkdir ~/catkin_ws;cd catkin_ws  
mkdir src  
cd src  
把tmotortest文件夹放到src目录下  
## 编译package

cd ~/catkin_ws  
catkin_make  
source devel/setup.bash  

## 使能串口
sudo ip link set can0 type can bitrate 1000000  
sudo ip link set up can0  
## 运行demo(给电机发送一个正弦位置信号，对电机进行位置环控制)
roscore  
rosrun tmotortest rostest1  

## 关于设置电机id：打开串口调试助手，波特率设置为9*****那个（不是9600，五位数还是六位数，发送s进入调式模式，输入iX+Enter，发送。x为电机id

目前最新版为tmotortest3,包括自动调零，电弹簧模式，四条腿控制，单条腿控制
运行：
串口使能；
打开joy node(插上xbox手柄usb)：
rosrun joy joy_node
工作空间下：
catkin_make
source devel/setup.bash
rosrun tmotor_leg tmotortest3
xbox操作请自行探索

