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

#define P_MIN -12.5f //-95.5f-95.5f rad
#define P_MAX 12.5f
#define V_MIN -38.2f //-30.0f-30.0f rad/s
#define V_MAX 38.2f
#define T_MIN -18.0f //-18.0f-18.0f N*m
#define T_MAX 18.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

ros::Publisher Tmotor_pos;

ros::Subscriber joy_sub;

int rxCounter = 0;
int txCounter = 0;

std_msgs::Float32MultiArray tmotor_pos_msgs;

struct Tmotor
{
	int id;

	float pos_zero = 0;
	float pos_abszero = 0;

	float pos_now;
	float pos_des = 0;
	float vel_now;
	float vel_des = 1;
	float t_now;
	float t_des = 0;
	float kp = 0;
	float kd = 2;

	int zeroPointSet = 0; //设过零点为1，未设过零点为0
	int flag = 0;
};
Tmotor tmotor[4];

void flagTest(int id);

void signalCallback(int signum)
{
	exit(1);
}

void buttonCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float fminf(float a, float b)
{
	if (a > b)
	{
		return b;
	}
	else
	{
		return a;
	}
}

float fmaxf(float a, float b)
{
	if (a > b)
	{
		return a;
	}
	else
	{
		return b;
	}
}

void rxThread(int s)
{
	int i;
	struct can_frame frame;
	int nbytes;
	for (i = 0;; i++)
	{
		ros::spinOnce();
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0)
		{
			perror("Read");
			break;
		}
		uint16_t motorID, pos, vel, t;
		float f_pos, f_vel, f_t;
		int ID;

		motorID = frame.data[0];
		ID = int(motorID - 0x00) - 1;
		pos = ((uint16_t)frame.data[1] << 8) | frame.data[2];
		vel = ((uint16_t)frame.data[3] << 4) | (frame.data[4] >> 4);
		t = ((uint16_t)(frame.data[4] & 0xf) << 8) | frame.data[5];
		rxCounter++;

		f_pos = uint_to_float(pos, P_MIN, P_MAX, 16);
		f_vel = uint_to_float(vel, V_MIN, V_MAX, 12);
		f_t = uint_to_float(t, T_MIN, T_MAX, 12);

		tmotor[ID].pos_now = f_pos;
		tmotor[ID].vel_now = f_vel;
		tmotor[ID].t_now = f_t;

		flagTest(ID);

		// if ((rxCounter > 0) && (rxCounter % 10 == 0))
		// {
		// 	printf("------pos_now is %f, vel_now  is %f, torque_now is %f , pos_zero is %f,flag is %d-----\n", tmotor.pos_now, tmotor.vel_now, tmotor.t_now, tmotor.pos_zero, tmotor.flag);
		// 	printf("\n");
		// }
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}

void flagTest(int id)
{
	if (tmotor[id].flag == 0)
	{
		if (tmotor[id].t_now > 1.5)
		{
			//开始调零
			tmotor[id].pos_abszero = tmotor[id].pos_now;
			tmotor[id].pos_zero = tmotor[id].pos_des = tmotor[id].pos_abszero - 6;
			tmotor[id].flag = 1;
		}
	}

	if (tmotor[id].flag == 1)
	{
		if (abs(tmotor[id].pos_zero - tmotor[id].pos_now) < 0.6)
		{
			tmotor[id].flag = 2;
		}
	}

	if (tmotor[id].flag == 2)
	{
		if (abs(tmotor[id].pos_zero - tmotor[id].pos_now) < 0.1)
		{
			tmotor[id].flag = 3;
		}
	}

	if ((tmotor[id].flag == 4) && (abs(tmotor[id].pos_now - tmotor[id].pos_zero) < 0.1))
	{
		tmotor[id].flag = 3;
	}
	else if ((tmotor[id].flag == 3) && (abs(tmotor[id].pos_now - tmotor[id].pos_zero) >= 0.1))
	{
		tmotor[id].flag = 4;
	}
}

void motorParaSet(int id)
{
	switch (tmotor[id].flag)
	{
	case 0:
		tmotor[id].t_des = 0;
		tmotor[id].vel_des = 1;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 2;
		break;
	case 1:
		tmotor[id].t_des = 0;
		tmotor[id].vel_des = -1;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 2;
		break;
	case 2:
		tmotor[id].t_des = 0;
		tmotor[id].vel_des = -0.2;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 2;
		break;
	case 3:
		tmotor[id].t_des = 0;
		tmotor[id].vel_des = 0;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 0;
		break;
	case 4:
		if (tmotor[id].pos_now - tmotor[id].pos_zero > 0)
		{
			tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * 0.1 - 0.3;
		}
		else
		{
			tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * 0.1 + 0.3;
		}
		tmotor[id].pos_des = 0;
		tmotor[id].vel_des = 0;
		tmotor[id].kd = 0;
		tmotor[id].kp = 0;
		break;

	default:
		break;
	}
}

void printTmotorInfo(int id)
{
	ROS_INFO("tmotor ID is  %d         flag is %d \n", tmotor[id].id, tmotor[id].flag);
}

void velTest(struct can_frame &frame, int id)
{
	float f_p, f_v, f_kp, f_kd, f_t;
	uint16_t p, v, kp, kd, t;
	motorParaSet(id);
	f_p = tmotor[id].pos_des;
	f_v = tmotor[id].vel_des;
	f_t = tmotor[id].t_des;
	f_kp = tmotor[id].kp;
	f_kd = tmotor[id].kd;

	// if ((txCounter > 0) && (txCounter % 10 == 0))
	// {
	// 	printf("------pos_des is %f, vel_des is %f, torque_des is %f  ------\n", f_p, f_v, f_t);
	// }

	p = float_to_uint(f_p, P_MIN, P_MAX, 16);
	v = float_to_uint(f_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(f_t, T_MIN, T_MAX, 12);
	frame.data[0] = p >> 8;
	frame.data[1] = p & 0xFF;
	frame.data[2] = v >> 4;
	frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
	frame.data[4] = kp & 0xFF;
	frame.data[5] = kd >> 4;
	frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
	frame.data[7] = t & 0xff;
}

void txThread(int s)
{
	struct can_frame frame;
	frame.can_id = 0x01;
	frame.can_dlc = 8;
	for (int j = 0; j < 8; j++)
	{
		frame.data[j] = 0xff;
	}
	frame.data[7] = 0xfc;
	//进入电机控制模式

	int nbytes;

	for (int i = 0;; i++)
	{
		tmotor_pos_msgs.data.resize(4);
		for (int id = 0; id < 4; id++)
		{
			frame.can_id = 0x00 + id + 1;
			velTest(frame, id);
			nbytes = write(s, &frame, sizeof(struct can_frame));
			if (nbytes == -1)
			{
				printf("send error\n");
				printf("please check battary!!\n");
				exit(1);
			}
			txCounter++;
			//printf("tx is %d;",txCounter);
			printTmotorInfo(id);

			tmotor_pos_msgs.data[id] = tmotor[id].pos_now;

			std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
		}
		Tmotor_pos.publish(tmotor_pos_msgs);
	}
}

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
	// printf("Wrote %d bytes\n", nbytes);
	if (nbytes == -1)
	{
		printf("send error\n");
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "rosTest1");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	signal(SIGINT, signalCallback);

	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;
	const char *ifname = "can0";

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	//printf("%s at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Error in socket bind");
		return -2;
	}
	//can连接完毕

	for (int i = 0; i < 4; i++)
	{
		tmotor[i].id = i;
	}
	//设置对应tmotor的id

	struct can_frame frame;
	for (int id = 1; id < 5; id++)
	{
		canCheck(frame, s, id);
	}
	//检查can通讯连接

	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, buttonCallback);
	//订阅节点
	Tmotor_pos = n.advertise<std_msgs::Float32MultiArray>("Tmotor_pos", 100);

	std::thread canTx(txThread, s);
	sleep(0.1);
	std::thread canRx(rxThread, s);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
