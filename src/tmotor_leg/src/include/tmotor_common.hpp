#include "ros/ros.h"

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

//参考AK80-9电机手册
#define P_MIN -12.5f //-12.5f —— 12.5f rad，电机位置
#define P_MAX 12.5f
#define V_MIN -76.0f //-30.0f——30.0f rad/s，电机速度
#define V_MAX 76.0f
#define T_MIN -12.0f //-18.0f——18.0f N*m，电机扭矩
#define T_MAX 12.0f

#define KP_MIN 0.0f //position control index
#define KP_MAX 500.0f
#define KD_MIN 0.0f // velocity control index
#define KD_MAX 5.0f

#define PI 3.14159265358979

int rxCounter = 0;
int txCounter = 0;
int Stop_flag = 0;

// Basic config for T-motor
struct Tmotor
{
	int id;

	float pos_zero;	   //弹簧零点（相对零点），一开始不赋值
	float pos_abszero; //绝对零点。电机零点

	float pos_now; //当前位置
	float pos_des; //目标（前馈）位置
	float vel_now; //当前速度
	float vel_des; //目标速度
	float vel_pd;
	float t_now;   //当前力矩（电流？）
	float t_des;   //前馈力矩（电流？）
	float kp;
	float kd;

	int zeroPointSet = 0; //设零点为1，not设零点为0
	int flag = 1;
	//旧版本： 0:调绝对零点 ；1：快速调相对零点；2：慢速调相对零点；3：电弹簧模式（近）4：电弹簧模式（远); 5:手柄控制
	//新版本：（上电位置为绝对零点）1：快速调相对零点；2：慢速调相对零点；3：电弹簧模式（近）4：电弹簧模式（远); 5:手柄控制
};

// Converts a float to an unsigned int
int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	x = fmax(fminf(x, x_max), x_min);
	return (int)((x - offset) * ((float)((1 << bits) - 1) / span));
}

// converts unsigned int to float
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// check communication and open T-motor
void canCheck(can_frame &frame, int s, int id)
{
	int nbytes;
	frame.can_dlc = 8;
	frame.can_id = 0x000 + id;
	for (int i = 0; i < 8; i++)
	{
		frame.data[i] = 0xff;
	}
	frame.data[7] = 0xfd;// close
	nbytes = write(s, &frame, sizeof(struct can_frame));
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	frame.data[7] = 0xfc;// start
	nbytes = write(s, &frame, sizeof(struct can_frame));
    //enter Tmotor control mode
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	ROS_INFO("Wrote %d bytes", nbytes);
	
	if (nbytes == -1)
	{
		ROS_INFO("Send error in canCheck process");
	}
	else
	{
		ROS_INFO("D[%d] pass check!", id);
	}
}

// check communication and open T-motor
void canCheckZeroSet(can_frame &frame, int s, int id)
{
	int nbytes;
	frame.can_dlc = 8;
	frame.can_id = 0x000 + id;
	for (int i = 0; i < 8; i++)
	{
		frame.data[i] = 0xff;
	}
	frame.data[7] = 0xfd;// close
	nbytes = write(s, &frame, sizeof(struct can_frame));
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	
	frame.data[7] = 0xfc;// start
	nbytes = write(s, &frame, sizeof(struct can_frame));
    //enter Tmotor control mode
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	

	frame.data[7] = 0xfe;//电机零点
	nbytes = write(s, &frame, sizeof(struct can_frame));
	//set Tmotor zero point	
	std::this_thread::sleep_for(std::chrono::milliseconds(20));

	ROS_INFO("Wrote %d bytes", nbytes);
	// std::this_thread::sleep_for(std::chrono::milliseconds(20));

	if (nbytes == -1)
	{
		ROS_INFO("Send error in canCheck process");
	}
	else
	{
		ROS_INFO("D[%d] pass check!", id);
	}
}

// A thread to deal with Ctrl-C command
void signalCallback(int signum)
{
	double startt, endt;
	startt = clock();
	Stop_flag = 1;
	ros::Duration(1.0).sleep();
	endt = clock();
	ROS_INFO("WARN:got signal [2], shutdown!");
	exit(1);
}
