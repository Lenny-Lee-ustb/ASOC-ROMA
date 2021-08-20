#include "include/tmotor_common.hpp"



void positionTest(struct can_frame &frame);
void velTest(struct can_frame &frame);
void StopTest(struct can_frame &frame);


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
		// printf("data is %x %x %x %x %x %x %x %x\n",frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
		// printf("id is %d;pos is %d;vel is %d;t is %d;\n", motorID, pos, vel, t);
		printf("pos_now is %.2f;vel_now is %.1f;torque_now is %.2f\n", f_pos, f_vel, f_t);
		printf("\n");
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}

void txThread(int s)
{
	struct can_frame frame;
	frame.can_id = 0x1; //0x2
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
		// positionTest(frame);
		velTest(frame);
		// StopTest(frame);
		if (Stop_flag == 1)
		{
			StopTest(frame);
		}
		
		nbytes = write(s, &frame, sizeof(struct can_frame));
		
		if (nbytes == -1)
		{
			printf("send error\n");
			printf("please check battary!!\n");
			exit(1);
		}
		txCounter++;

		std::this_thread::sleep_for(std::chrono::nanoseconds(10000000)); // freq 1000hz
	}
}


void StopTest(struct can_frame &frame)
{
	float f_p, f_v, f_kp, f_kd, f_t;
	f_v = 0;
	f_t = 0;
	f_kp = 0;
	f_kd = 0;
	f_p = 0;
	f_p = fminf(fmaxf(P_MIN, f_p), P_MAX);
	f_v = fminf(fmaxf(V_MIN, f_v), V_MAX);
	f_kp = fminf(fmaxf(KP_MIN, f_kp), KP_MAX);
	f_kd = fminf(fmaxf(KD_MIN, f_kd), KD_MAX);
	f_t = fminf(fmaxf(T_MIN, f_t), T_MAX);
	printf("------pos_des is %f, vel_des is %f, torque_des is %f  ------\n", f_p, f_v, f_t);

	uint16_t p, v, kp, kd, t;
	p = float_to_uint(f_p, P_MIN, P_MAX, 16);
	v = float_to_uint(f_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(f_t, T_MIN, T_MAX, 12);
	//printf("--------p is %d, v is %d, t is %d ------- \n", p,v,t);
	frame.data[0] = p >> 8;
	frame.data[1] = p & 0xFF;
	frame.data[2] = v >> 4;
	frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
	frame.data[4] = kp & 0xFF;
	frame.data[5] = kd >> 4;
	frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
	frame.data[7] = t & 0xff;
}


void positionTest(struct can_frame &frame)
{
	float f_p, f_v, f_kp, f_kd, f_t;
	f_v = 0;
	f_t = 0;
	f_kp = 1;
	f_kd = 0;
	//f_p = 12.5f;
	f_p = 1;
	f_p = fminf(fmaxf(P_MIN, f_p), P_MAX);
	f_v = fminf(fmaxf(V_MIN, f_v), V_MAX);
	f_kp = fminf(fmaxf(KP_MIN, f_kp), KP_MAX);
	f_kd = fminf(fmaxf(KD_MIN, f_kd), KD_MAX);
	f_t = fminf(fmaxf(T_MIN, f_t), T_MAX);
	printf("pos_des is %.2f, vel_des is %.1f, torque_des is %.2f\n", f_p, f_v, f_t);

	uint16_t p, v, kp, kd, t;
	p = float_to_uint(f_p, P_MIN, P_MAX, 16);
	v = float_to_uint(f_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(f_t, T_MIN, T_MAX, 12);
	//printf("--------p is %d, v is %d, t is %d ------- \n", p,v,t);
	frame.data[0] = p >> 8;
	frame.data[1] = p & 0xFF;
	frame.data[2] = v >> 4;
	frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
	frame.data[4] = kp & 0xFF;
	frame.data[5] = kd >> 4;
	frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
	frame.data[7] = t & 0xff;
}


void velTest(struct can_frame &frame)
{
	float f_p, f_v, f_kp, f_kd, f_t;
	f_v = 1;
	f_t = 0;
	//t=5 T=2.87  kt=0.574
	//t=10 T=5.42   kt=0.542
	//t=2 T=1.05        转矩模式  kt=0.525

	//纯速度模式 0.56

	f_p = 0;
	f_kd = 2;
	f_kp = 0;
	printf("pos_des is %.2f, vel_des is %.1f, torque_des is %.2f\n", f_p, f_v, f_t);

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


int main(int argc, char **argv)
{

	ros::init(argc, argv, "rosTest1");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	

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
	frame.can_id = 0x001; //0x002
	for (int i = 0; i < 8; i++)
	{
		frame.data[i] = 0xff;
	}
	frame.data[7] = 0xfc;

	nbytes = write(s, &frame, sizeof(struct can_frame));

	if (nbytes == -1)
	{
		printf("send error\n");
	}

	signal(SIGINT, signalCallback);

	std::thread canTx(txThread, s);
	sleep(0.1);
	std::thread canRx(rxThread, s);
    ROS_INFO("***");
	while (1)
	{
		sleep(1);
	}

	return 0;
}
