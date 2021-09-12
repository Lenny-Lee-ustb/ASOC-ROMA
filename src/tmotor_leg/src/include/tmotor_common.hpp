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

#define P_MIN -12.5f //-12.5f —— 12.5f rad
#define P_MAX 12.5f
#define V_MIN -38.2f //-30.0f——30.0f rad/s
#define V_MAX 38.2f
#define T_MIN -18.0f //-18.0f——18.0f N*m
#define T_MAX 18.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

int rxCounter = 0;
int txCounter = 0;
int Stop_flag = 0;

// Basic config for T-motor
struct Tmotor
{
	int id;

	float pos_zero = 1;	   //相对零点
	float pos_abszero = 0; //绝对零点

	float pos_now;		//当前位置
	float pos_des;	//目标（前馈）位置
	float vel_now;		//当前速度
	float vel_des; //目标速度
	float t_now;		//当前力矩（电流？）
	float t_des;	//前馈力矩（电流？）
	float kp;
	float kd;

	int zeroPointSet = 0; //设过零点为1，未设过零点为0
	int flag = 1;
	//旧版本： 0:调绝对零点 ；1：快速调相对零点；2：慢速调相对零点；3：电弹簧模式（近）4：电弹簧模式（远); 5:手柄控制
	//新版本：（上电位置为绝对零点）1：快速调相对零点；2：慢速调相对零点；3：电弹簧模式（近）4：电弹簧模式（远); 5:手柄控制
};


// Converts a float to an unsigned int, given range and number of bits
int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}


// converts unsigned int to float, given range and number of bits
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
	frame.data[7] = 0xfc;
	nbytes = write(s, &frame, sizeof(struct can_frame));
    //enter Tmotor control mode
    sleep(0.1);
	frame.data[7] = 0xfe;
	nbytes = write(s, &frame, sizeof(struct can_frame));
	//set Tmotor zero point	
	// printf("Wrote %d bytes\n", nbytes);
	sleep(0.1);
	if (nbytes == -1)
	{
		printf("send error\n");
	}
}

// A thread to deal with Ctrl-C command
void signalCallback(int signum)
{	
	double startt,endt;
	startt = clock();
	Stop_flag = 1;
	ros::Duration(1.0).sleep();
	endt = clock();
	ROS_INFO("shutdown!!!!!!!");
	exit(1);
}
