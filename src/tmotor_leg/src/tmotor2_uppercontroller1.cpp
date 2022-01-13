#include "include/tmotor_common.hpp"
//AK80_9测试1，仅手柄控制

ros::Publisher Tmotor_Info;
ros::Subscriber Control_sub;

//电弹簧模式参数
double K_S = 9.0;
double D_S = 0.3;
double zero_length = 2.0;

//电机位置信息，速度信息，力矩信息（带时间戳）
geometry_msgs::PolygonStamped tmotor_info_msgs;

Tmotor tmotor[4];

//监测电机状态
void flagTest(int id)
{
	//电弹簧模式工作状态，接近零点启用flag1，远离零点启用flag2（常规电弹簧模式），flag3的目的是避免零点附近的电机震荡
	if ((tmotor[id].flag == 2) && (abs(tmotor[id].pos_now - tmotor[id].pos_zero) < 0.15))
	{
		tmotor[id].flag = 1;
	}
	else if ((tmotor[id].flag == 1) && (abs(tmotor[id].pos_now - tmotor[id].pos_zero) >= 0.15))
	{
		tmotor[id].flag = 2;
	}
}

// 由上位机修正电机零点
void ControlCallback(const geometry_msgs::PolygonStamped &ctrl_cmd)
{
	for (int id = 0; id < 4; id++)
	{
		tmotor[id].pos_zero = ctrl_cmd.polygon.points[id].x;
	}
}

//收报函数
void rxThread(int s)
{
	int i;
	struct can_frame frame;
	int nbytes;
	sleep(1);
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

		tmotor[ID].pos_now = f_pos;
		tmotor[ID].vel_now = f_vel;
		tmotor[ID].t_now = f_t;

		if (rxCounter < 4)
		{
			tmotor[rxCounter].pos_zero = tmotor[rxCounter].pos_now;
			tmotor[rxCounter].zeroPointSet = 1;
		}

		rxCounter++;
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}

//设置tmotor各个flag对应的电机参数
void motorParaSet(int id)
{
	switch (tmotor[id].flag)
	{
	case 1:
		tmotor[id].t_des = 0.2;
		tmotor[id].vel_des = 0;
		tmotor[id].pos_des = tmotor[id].pos_zero;
		tmotor[id].kp = 3;
		tmotor[id].kd = 0;
		break;

	case 2:
		//F=kx+电机自身阻尼补偿+速度阻尼
		if (tmotor[id].pos_now - tmotor[id].pos_zero > 0)
		{
			if (tmotor[id].vel_now > 0)
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * K_S - 0.6 - D_S * abs(tmotor[id].vel_now);
			}
			else
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * K_S - 0.6 + D_S * abs(tmotor[id].vel_now);
			}
		}
		else
		{
			if (tmotor[id].vel_now > 0)
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * K_S + 0.6 - D_S * abs(tmotor[id].vel_now);
			}
			else
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * K_S + 0.6 + D_S * abs(tmotor[id].vel_now);
			}
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

//init CAN_Frame member data
void frameDataSet(struct can_frame &frame, int id)
{
	float f_p, f_v, f_kp, f_kd, f_t;
	uint16_t p, v, kp, kd, t;

	if (tmotor[id].flag != 5)
	{
		flagTest(id);
		motorParaSet(id);
	}

	f_p = tmotor[id].pos_des;
	f_v = tmotor[id].vel_des;
	f_t = tmotor[id].t_des;
	f_kp = tmotor[id].kp;
	f_kd = tmotor[id].kd;

	//粗暴地限流
	if (f_t > 10.0)
	{
		f_t = 10.0;
	}
	else if (f_t < -10.0)
	{
		f_t = -10.0;
	}

	//限位保護
	f_t = fmax(fminf(tmotor[id].t_des, T_MAX), T_MIN);
	f_p = fmax(fminf(tmotor[id].pos_des, P_MAX), P_MIN);
	f_v = fmax(fminf(tmotor[id].vel_des, V_MAX), V_MIN);

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
void printTmotorInfo(int id)
{
	ROS_INFO("\nflag[%d,%d,%d,%d] \npos_now is [%.2f,%.2f,%.2f,%.2f]\npos_des is [%.2f,%.2f,%.2f,%.2f] \nvel_des is [%.2f,%.2f,%.2f,%.2f] \nt_now is [%.2f,%.2f,%.2f,%.2f] \npos_zero is [%.2f,%.2f,%.2f,%.2f] \nstop_flag:%d\n------------\n",
			 tmotor[0].flag, tmotor[1].flag, tmotor[2].flag, tmotor[3].flag,
			 tmotor[0].pos_now, tmotor[1].pos_now, tmotor[2].pos_now, tmotor[3].pos_now,
			 tmotor[0].pos_des, tmotor[1].pos_des, tmotor[2].pos_des, tmotor[3].pos_des,
			 tmotor[0].vel_des, tmotor[1].vel_des, tmotor[2].vel_des, tmotor[3].vel_des,
			 tmotor[0].t_now, tmotor[1].t_now, tmotor[2].t_now, tmotor[3].t_now,
			 tmotor[0].pos_zero, tmotor[1].pos_zero, tmotor[2].pos_zero, tmotor[3].pos_zero,
			 Stop_flag);
}

//发报函数
void txThread(int s)
{
	struct can_frame frame;
	frame.can_id = 0x01;
	frame.can_dlc = 8;

	int nbytes;

	sleep(1);
	for (int i = 0;; i++)
	{
		tmotor_info_msgs.polygon.points.resize(4);
		for (int id = 0; id < 4; id++)
		{
			frame.can_id = 0x00 + id + 1;
			frameDataSet(frame, id);
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
			printTmotorInfo(id);

			tmotor_info_msgs.polygon.points[id].x = tmotor[id].pos_now;
			tmotor_info_msgs.polygon.points[id].y = tmotor[id].vel_now;
			tmotor_info_msgs.polygon.points[id].z = tmotor[id].t_now;
			//每个点x为tmotor当前位置，y为tmotor当前速度，z为tmotor当前电流

			std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
		}
		tmotor_info_msgs.header.stamp = ros::Time::now();
		Tmotor_Info.publish(tmotor_info_msgs);
		//标记时间戳并发布msg
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "tmotor2_xboxtest1");
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
		ROS_INFO("D[%d] pass check!", id);
	}
	//检查can通讯连接

	Control_sub = n.subscribe("suspension_cmd", 2, ControlCallback);
	Tmotor_Info = n.advertise<geometry_msgs::PolygonStamped>("Tmotor_Info", 100);
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
