#include "include/tmotor_common.hpp"
//AK80_9单个电机测试

ros::Subscriber joy_sub;

//判断是否开启手柄模式 xbox_mode_on>0: 开启；<0关闭
int xbox_mode_on = -1;
int xbox_power = 0;
int xbox_power_last = 0;

double K_S = 9.0;
double D_S = 0.3;
double zero_length = 2.0;

Tmotor tmotor;

//监测电机状态
void flagTest()
{
	if ((tmotor.flag == 2) && (abs(tmotor.pos_now - tmotor.pos_zero) < 0.15))
	{
		tmotor.flag = 1;
	}
	else if ((tmotor.flag == 1) && (abs(tmotor.pos_now - tmotor.pos_zero) >= 0.15))
	{
		tmotor.flag = 2;
	}
	else
	{
		tmotor.flag = 1;
	}
}

//joy按键回调函数
void buttonCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
	//只有四个电机都调零完毕才能手柄控制，flag5为手柄控制模式
	if (tmotor.zeroPointSet == 1)
	{
		xbox_power = joy->buttons[7];
		float move_up = -(joy->axes[2]) + 1;
		float move_down = -(joy->axes[5]) + 1;

		if (xbox_power > xbox_power_last)
		{
			xbox_mode_on = -xbox_mode_on;
		}
		xbox_power_last = xbox_power;

		if (move_up != 0)
		{
			tmotor.flag = 5;
			tmotor.vel_des = 4 * move_up;
			tmotor.pos_des = 0;
			tmotor.t_des = 0;
			tmotor.kd = 2;
			tmotor.kp = 0;
		}

		if (move_down != 0)
		{
			tmotor.flag = 5;
			tmotor.vel_des = -(4 * move_down);
			tmotor.pos_des = 0;
			tmotor.t_des = 0;
			tmotor.kd = 2;
			tmotor.kp = 0;
		}

		if ((move_up == 0) && (move_down == 0))
		{
			if (xbox_mode_on > 0)
			{
				tmotor.pos_des = tmotor.pos_now;
				tmotor.vel_des = 0;
				tmotor.t_des = 0;
				tmotor.kp = 20;
				tmotor.kd = 0;
			}
			else
			{
				flagTest();
			}
		}
	}
}

//收报函数
void rxThread(int s)
{
	int i;
	struct can_frame frame;
	int nbytes;
	sleep(0.4);
	for (i = 0;; i++)
	{
		ros::spinOnce();
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0)
		{
			perror("Read Error!");
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

		//参考AK80-6电机手册，整型转浮点型
		f_pos = uint_to_float(pos, P_MIN, P_MAX, 16);
		f_vel = uint_to_float(vel, V_MIN, V_MAX, 12);
		f_t = uint_to_float(t, T_MIN, T_MAX, 12);

		tmotor.pos_now = f_pos;
		tmotor.vel_now = f_vel;
		tmotor.t_now = f_t;

		if (rxCounter == 4)
		{
			tmotor.pos_zero = tmotor.pos_now;
			tmotor.zeroPointSet = 1;
		}

		rxCounter++;
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}

//设置tmotor各个flag对应的电机参数
void motorParaSet()
{
	switch (tmotor.flag)
	{
	case 1:
		tmotor.t_des = 10;
		tmotor.vel_des = 10;
		// tmotor.pos_des = tmotor.pos_zero;
		tmotor.pos_des = 0;
		tmotor.kp = 0;
		tmotor.kd = 3;
		break;

	case 2:
		//F=kx+电机自身阻尼补偿+速度阻尼
		// if (tmotor.pos_now - tmotor.pos_zero > 0)
		// {
		// 	if (tmotor.vel_now > 0)
		// 	{
		// 		tmotor.t_des = -(tmotor.pos_now - tmotor.pos_zero) * K_S - 0.6 - D_S * abs(tmotor.vel_now);
		// 	}
		// 	else
		// 	{
		// 		tmotor.t_des = -(tmotor.pos_now - tmotor.pos_zero) * K_S - 0.6 + D_S * abs(tmotor.vel_now);
		// 	}
		// }
		// else
		// {
		// 	if (tmotor.vel_now > 0)
		// 	{
		// 		tmotor.t_des = -(tmotor.pos_now - tmotor.pos_zero) * K_S + 0.6 - D_S * abs(tmotor.vel_now);
		// 	}
		// 	else
		// 	{
		// 		tmotor.t_des = -(tmotor.pos_now - tmotor.pos_zero) * K_S + 0.6 + D_S * abs(tmotor.vel_now);
		// 	}
		// }
		tmotor.t_des = 10;
		tmotor.pos_des = 0;
		tmotor.vel_des = 10;
		tmotor.kd = 3;
		tmotor.kp = 0;
		break;

	default:
		break;
	}
}

//init CAN_Frame member data
void frameDataSet(struct can_frame &frame)
{
	float f_p, f_v, f_kp, f_kd, f_t;
	uint16_t p, v, kp, kd, t;

	if (tmotor.flag != 5)
	{
		flagTest();
		motorParaSet();
	}

	f_p = tmotor.pos_des;
	f_v = tmotor.vel_des;
	f_t = tmotor.t_des;
	f_kp = tmotor.kp;
	f_kd = tmotor.kd;

	//限位保護
	f_t = fmax(fminf(tmotor.t_des, T_MAX), T_MIN);
	f_p = fmax(fminf(tmotor.pos_des, P_MAX), P_MIN);
	f_v = fmax(fminf(tmotor.vel_des, V_MAX), V_MIN);

	//参考AK80-9电机使用手册，将各参数的浮点型转化为整型后保存在data中
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

//打印信息
void printTmotorInfo()
{
	ROS_INFO("\nflag[%d] \npos_now is [%.2f]\npos_des is [%.2f] \nvel_des is [%.2f] \nvel_now is [%.2f]\n t_des is [%.2f]\n t_now is [%.2f] \npos_zero is [%.2f] \nxbox_mode: %d\nstop_flag:%d\n------------\n",
			 tmotor.flag,
			 tmotor.pos_now,
			 tmotor.pos_des,
			 tmotor.vel_des,
			 tmotor.vel_now,
			 tmotor.t_des,
			 tmotor.t_now,
			 tmotor.pos_zero,
			 xbox_mode_on,
			 Stop_flag);
}

//发报函数
void txThread(int s)
{
	struct can_frame frame;
	frame.can_id = 0x01;
	frame.can_dlc = 8;

	int nbytes;

	sleep(0.6);
	for (int i = 0;; i++)
	{
		frame.can_id = 0x01;
		frameDataSet(frame);
		if (Stop_flag == 1)
		{
			for (int j = 0; j < 8; j++)
			{
				frame.data[j] = 0xff;
			}
			frame.data[7] = 0xfd; // exit T-motor control mode!
		}

		nbytes = write(s, &frame, sizeof(struct can_frame));
		if (nbytes == -1)
		{
			printf("send error\n");
			exit(1);
		}
		txCounter++;
		printTmotorInfo();
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "tmotor2_singletest1");
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

	struct can_frame frame;
	// for (int id = 1; id < 5; id++)
	// {
	// 	canCheck(frame, s, id);
	// 	ROS_INFO("D[%d] pass check!", id);
	// }
	//检查can通讯连接

	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, buttonCallback);
	//Tmotor_Info = n.advertise<geometry_msgs::PolygonStamped>("Tmotor_Info", 100);
	//发布及订阅节点

	std::thread canTx(txThread, s);
	std::thread canRx(rxThread, s);
	//开启收报/发报线程

	while (ros::ok())
	{
		ros::spinOnce();
		//必须开，否则无法启用回调函数
		loop_rate.sleep();
	}

	return 0;
}
