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

#define P_MIN -95.5f //-95.5f-95.5f rad
#define P_MAX 95.5f
#define V_MIN -30.0f //-30.0f-30.0f rad/s
#define V_MAX 30.0f
#define T_MIN -18.0f //-18.0f-18.0f N*m
#define T_MAX 18.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

double timeCounter;
int rxCounter = 0;
int txCounter = 0;

struct Tmotor
{
	float pos_now;
	float pos_des = 0;
	float pos_zero = 0;
	float vel_now;
	float vel_des = 1;
	float t_now;
	float t_des = 0;
	float kp = 0;
	float kd = 2;

	/* flag 用于表示电机的状态
	flag=0: 开始调绝对零点
	flag=1: 开始调相对零点
	flag=2: 电弹簧模式(正常)
    flag=3: 电弹簧模式(接近相对零点)
	flag=5: 开始低速调相对零点
	*/
	int flag = 0;
} tmotor;

void flagTest();
void paraSet();

void signalCallback(int signum)
{
	exit(1);
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
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0)
		{
			perror("Read");
			break;
		}

		uint16_t motorID, pos, vel, t;
		float f_pos, f_vel, f_t;

		motorID = frame.data[0];
		pos = ((uint16_t)frame.data[1] << 8) | frame.data[2];
		vel = ((uint16_t)frame.data[3] << 4) | (frame.data[4] >> 4);
		t = ((uint16_t)(frame.data[4] & 0xf) << 8) | frame.data[5];
		rxCounter++;

		f_pos = uint_to_float(pos, P_MIN, P_MAX, 16);
		f_vel = uint_to_float(vel, V_MIN, V_MAX, 12);
		f_t = uint_to_float(t, T_MIN, T_MAX, 12);

		//更新tmotor数据并打印
		tmotor.pos_now = f_pos;
		tmotor.vel_now = f_vel;
		tmotor.t_now = f_t;

		flagTest();

		if ((rxCounter > 0) && (rxCounter % 10 == 0))
		{
			printf("------pos_now is %f, vel_now  is %f, torque_now is %f , pos_zero is %f,flag is %d-----\n", tmotor.pos_now, tmotor.vel_now, tmotor.t_now, tmotor.pos_zero, tmotor.flag);
			printf("\n");
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}

void flagTest()
{
	if (tmotor.flag == 0)
	{
		if (tmotor.t_now > 1)
		{
			//开始调零
			tmotor.pos_des = tmotor.pos_now - 10;
			tmotor.pos_zero = tmotor.pos_des;
			tmotor.flag = 1;
		}
	}

	if (tmotor.flag == 1)
	{
		if (abs(tmotor.pos_zero - tmotor.pos_now) < 0.5)
		{
			tmotor.flag = 5;
		}
	}

	//低速调零结束
	if (tmotor.flag == 5)
	{
		if (abs(tmotor.pos_zero - tmotor.pos_now) < 0.01)
		{
			tmotor.flag = 2;
		}
	}

	if ((tmotor.flag == 2) && (abs(tmotor.pos_now - tmotor.pos_zero) < 0.1))
	{
		tmotor.flag = 3;
	}
	else if ((tmotor.flag == 3) && (abs(tmotor.pos_now - tmotor.pos_zero) >= 0.1))
	{
		tmotor.flag = 2;
	}
}

void paraSet()
{
	if (tmotor.flag == 1)
	{
		tmotor.pos_des = 0;
		tmotor.vel_des = -0.5;
		tmotor.t_des = 0;
		tmotor.kd = 2;
		tmotor.kp = 0;
	}
	else if (tmotor.flag == 2)
	{
		if ((tmotor.pos_now - tmotor.pos_zero) > 0)
		{
			tmotor.t_des = -(tmotor.pos_now - tmotor.pos_zero) * 0.06 - 0.3;
		}
		else
		{
			tmotor.t_des = -(tmotor.pos_now - tmotor.pos_zero) * 0.06 + 0.3;
		}

		tmotor.pos_des = 0;
		tmotor.vel_des = 0;
		tmotor.kd = 0;
		tmotor.kp = 0;
	}
	else if (tmotor.flag == 5)
	{
		tmotor.t_des = 0;
		tmotor.pos_des = tmotor.pos_zero;
		tmotor.vel_des = 0;
		tmotor.kd = 0;
		tmotor.kp = 20;
	}
	else if (tmotor.flag == 3)
	{
		tmotor.t_des = 0;
		tmotor.pos_des = 0;
		tmotor.vel_des = 0;
		tmotor.kd = 0;
		tmotor.kp = 0;
	}
	else
	{ //flag=0;
		tmotor.t_des = 0;
		tmotor.vel_des = 1;
		tmotor.pos_des = 0;
		tmotor.kd = 2;
		tmotor.kp = 0;
	}
}

void velTest(struct can_frame &frame)
{
	paraSet();
	float f_p, f_v, f_kp, f_kd, f_t;
	f_p = tmotor.pos_des;
	f_v = tmotor.vel_des;
	f_t = tmotor.t_des;
	f_kp = tmotor.kp;
	f_kd = tmotor.kd;

	if ((txCounter > 0) && (txCounter % 10 == 0))
	{
		printf("------pos_des is %f, vel_des is %f, torque_des is %f  ------\n", f_p, f_v, f_t);
	}

	uint16_t p, v, kp, kd, t;
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
	frame.can_id = 0x2;
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
		velTest(frame);
		nbytes = write(s, &frame, sizeof(struct can_frame));
		if (nbytes == -1)
		{
			printf("send error\n");
			printf("please check battary!!\n");
			exit(1);
		}
		txCounter++;

		std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
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

	int nbytes;
	struct can_frame frame;
	frame.can_dlc = 8;
	frame.can_id = 0x002;
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
	std::thread canTx(txThread, s);
	sleep(0.1);
	std::thread canRx(rxThread, s);

	while (1)
	{
		sleep(1);
	}

	return 0;
}
