# asoc_lower_controller 

文件结构：\
|--> docs \
| roll角编码器CAN通讯的参考 \
| C620电调说明书，带通信协议\
|--> src \
| /include/asoc_lower_platform.hpp 主控的头文件，将部分变量和初始化放置于此\
| asoc_lower_platform.cpp 下位机主控，输入为车身状态量，输出为各个轮的速度（电流控制）\

使用方式：\
cd ~/asoc-roma \
sudo ./asoc_lower_platform.sh

注意事项：\
请一定记得CAN口的接入顺序！\
提前打开枢轴的编码器脚本，否则各轮子的初始角度设置会有问题。\
各腿的编号方式：头方向为low_0, 尾是low_1。车头朝前时，车的左腿为high_0, 右腿为high_1。
