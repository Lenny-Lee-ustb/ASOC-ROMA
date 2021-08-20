#include "include/tmotor_common.hpp"

ros::Publisher Tmotor_pos;
ros::Subscriber joy_sub;

//判断是否开启手柄模式 xbox_mode_on>0: 开启；<0关闭
int xbox_mode_on = -1;

std_msgs::Float32MultiArray tmotor_pos_msgs;

Tmotor tmotor[4];

//joy按键回调函数
void buttonCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

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

		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
	}
}


void motorParaSet(int id)
{
	switch (tmotor[id].flag)
	{
	case 0:
		tmotor[id].t_des = 0;
		tmotor[id].vel_des = -1;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 2;
		break;
	case 1:
		tmotor[id].t_des = 5;
		tmotor[id].vel_des = 0.5;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 5;
		break;
	case 2:
		tmotor[id].t_des = 5;
		tmotor[id].vel_des = 0.1;
		tmotor[id].pos_des = tmotor[id].pos_zero;
		tmotor[id].kp = 0;
		tmotor[id].kd = 5;
		break;
	case 3:
		tmotor[id].t_des = 0;
		tmotor[id].vel_des = 0;
		tmotor[id].pos_des = tmotor[id].pos_zero;
		tmotor[id].kp = 5;
		tmotor[id].kd = 0;
		break;
	case 4:

		//F=kx+电机自身阻尼补偿+速度阻尼
		if (tmotor[id].pos_now - tmotor[id].pos_zero > 0)
		{
			if (tmotor[id].vel_now > 0)
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * 4 - 0.3 - 0.3 * abs(tmotor[id].vel_now);
			}
			else
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * 4 - 0.3 + 0.3 * abs(tmotor[id].vel_now);
			}
		}
		else
		{
			if (tmotor[id].vel_now > 0)
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * 4 + 0.3 - 0.3 * abs(tmotor[id].vel_now);
			}
			else
			{
				tmotor[id].t_des = -(tmotor[id].pos_now - tmotor[id].pos_zero) * 4 + 0.3 + 0.3 * abs(tmotor[id].vel_now);
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



//打印信息
void printTmotorInfo(int id)
{
	ROS_INFO("\n-----\nID[%d]\nflag[%d] \nvel_des is %.2f\npos_des is %.2f \nt_now is %.2f \nZeropointset is %d \nxbox_mode: %d\nstop_flag:%d\n",
	 tmotor[id].id, 
	 tmotor[id].flag,
	 tmotor[id].vel_des, 
	 tmotor[id].pos_des, 
	 tmotor[id].t_now, 
	 tmotor[id].zeroPointSet, 
	 xbox_mode_on,
	 Stop_flag
	 );
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
			frameDataSet(frame, id);
			if (Stop_flag == 1)
			{
				tmotor[id].t_des = 0;
				tmotor[id].vel_des = 0;
				tmotor[id].pos_des = 0;
				tmotor[id].kp = 0;
				tmotor[id].kd = 0;
				frameDataSet(frame, id);
			}
			
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

	ros::init(argc, argv, "tmotorTest2");
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
	Tmotor_pos = n.advertise<std_msgs::Float32MultiArray>("Tmotor_pos", 100);
	//发布及订阅节点

	std::thread canTx(txThread, s);
	sleep(0.1);
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
