#include "include/tmotor_common.hpp"
//Integrate the IMU to control posture
// case 1 : pos_des works
// case 2 : vel_des works, the posture is refered to IMU
// run this cpp by launch file!

ros::Publisher Tmotor_Info;
ros::Subscriber Imu_sub, Joy_sub;

//电弹簧模式参数
double K_S =0.0;
double D_S =0.0;


double P_pit, D_pit, P_rol, D_rol;
int imuOnOff= -1;
float button_A;
float button_A_last=0;



// orientation
double roll, pitch;
double roll_ref=0.0;
double pitch_ref=0.0;
double deltaRoll, deltaPitch;
double lastRoll, lastPitch;

std_msgs::Float32MultiArray imuPose;
geometry_msgs::PolygonStamped tmotor_info_msgs;
sensor_msgs::Joy button;

Tmotor tmotor[4];

void joyCB(const sensor_msgs::Joy::ConstPtr& joy){
	button = *joy;
	button_A = button.buttons[0];
	if(button_A > button_A_last){
		imuOnOff = -imuOnOff;
	}
	button_A_last = button_A;

}

void imuCB(const std_msgs::Float32MultiArray::ConstPtr &sensorMsg)
{
    imuPose.data.resize(3);
    imuPose = *sensorMsg;
	if(fabs(imuPose.data[0])< 20.0 && fabs(imuPose.data[1])< 20.0){
		roll = imuPose.data[0]/180.0*PI;
		pitch = imuPose.data[1]/180.0*PI;
	}
	deltaRoll = roll_ref - roll;
    deltaPitch = pitch_ref - pitch;

    tmotor[2].vel_pd = P_pit * pitch + D_pit * (pitch - lastPitch);
    tmotor[0].vel_pd = -(P_rol * roll + D_rol * (roll - lastRoll));
    tmotor[1].vel_pd = -(P_pit * pitch + D_pit * (pitch - lastPitch));
    tmotor[3].vel_pd = P_rol * roll + D_rol * (roll - lastRoll);

    lastPitch = deltaPitch;
    lastRoll = deltaRoll;
}


//监测电机状态
void flagTest(int id)
{
	// 电弹簧模式工作状态，接近零点启用flag1，远离零点启用flag2（常规电弹簧模式），flag3的目的是避免零点附近的电机震荡
	// 0.15rad近似8.6度
	if ((tmotor[id].flag == 2) && (imuOnOff == -1))
	{
		tmotor[id].flag = 1;
	}
	else if ((tmotor[id].flag == 1) && (imuOnOff == 1))
	{
		tmotor[id].flag = 2;
	}
}

//收报函数
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
		
		
		// if (rxCounter < 40)
		// {
		// 	tmotor[ID].pos_zero = tmotor[ID].pos_now;
		// 	tmotor[ID].zeroPointSet = 1;
		// 	ROS_INFO("Balance ponit set!! pos:%.2f",tmotor[ID].pos_now);
		// }

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

		tmotor[id].t_des = 0;
		tmotor[id].pos_des = tmotor[id].pos_zero;
		tmotor[id].vel_des = 0;
		tmotor[id].kd = 0;
		tmotor[id].kp = K_S;
		break;

	case 2:
		tmotor[id].t_des = 0.1;
		tmotor[id].vel_des = tmotor[id].vel_pd;
		tmotor[id].pos_des = 0;
		tmotor[id].kp = 0;
		tmotor[id].kd = 3;
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

    flagTest(id);
    motorParaSet(id);

	f_p = tmotor[id].pos_des;
	f_v = tmotor[id].vel_des;
	f_t = tmotor[id].t_des;
	f_kp = tmotor[id].kp;
	f_kd = tmotor[id].kd;

	//粗暴地限流，持续扭矩6nm，峰值扭矩12nm
	if (f_t > 8.0)
	{
		f_t = 8.0;
	}
	else if (f_t < -8.0)
	{
		f_t = -8.0;
	}

	//位置限幅只对纯位置模式起小作用，因为在纯位置模式下模拟弹簧，电机依旧可以远离目标位置。
	//所以限幅应该比一开始设想的3.5rad还小，3.0
	if (f_p > 3.0)
	{
		f_p = 3.0;
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
	if(imuOnOff==1){
		ROS_INFO("Roll: %.3f, Pitch: %.3f", roll, pitch);
		}

	ROS_INFO("\nflag[%d,%d,%d,%d] \npos_now is [%.2f,%.2f,%.2f,%.2f]\npos_des is [%.2f,%.2f,%.2f,%.2f] \nvel_des is [%.3f,%.3f,%.3f,%.3f] \nt_now is [%.2f,%.2f,%.2f,%.2f] \npos_zero is [%.2f,%.2f,%.2f,%.2f] \nstop_flag:%d\n------------\n",
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

	// sleep 
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
			if(txCounter%100==0){
				printTmotorInfo(id);
			}

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

	ros::init(argc, argv, "tmotor2_uppercontroller1");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	signal(SIGINT, signalCallback);
	bool InitZero;
	double pos_zero_temp;

    Imu_sub = n.subscribe("/imu_rpy0", 1, imuCB);
	Joy_sub = n.subscribe("/joy", 1, joyCB);
	Tmotor_Info = n.advertise<geometry_msgs::PolygonStamped>("Tmotor_Info", 100);
	//发布及订阅节点
	
	n.param("InitZero",InitZero,false);
	// roslauch文件给定pos_zero
	n.param("pos_zero_temp",pos_zero_temp,0.0);
	n.param("K_S",K_S,0.0);
	n.param("D_S",D_S,0.0);
    n.param("P_pit", P_pit, 0.1);
    n.param("D_pit", D_pit, 0.0);
    n.param("P_rol", P_rol, 0.1);
    n.param("D_rol", D_rol, 0.0);
	ROS_INFO("--------------");
	ROS_INFO("K_S: %.2f, D_S: %.2f",K_S,D_S);
    ROS_INFO("P_pit: %.2f, D_pit: %.2f, P_rol: %.2f, D_rol: %.2f", P_pit, D_pit, P_rol, D_rol);
	ROS_INFO("--------------");
	for (int i = 0; i < 4; i++)
	{
	tmotor[i].pos_zero = pos_zero_temp;
	}
	ROS_INFO("pos_zero is given by roslaunch !!!!!");
	ROS_INFO("pos_zero_temp is [%.2f]",pos_zero_temp);	

	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;
	const char *ifname = "can2";

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
		if(InitZero == false){
			canCheck(frame, s, id);
		}
		else if(InitZero == true){
			canCheckZeroSet(frame, s, id);
			ROS_INFO("Zero point is SET !!!!!");
		}
	}
	//进入电机控制模式，电机初始化位置设为电机零点（绝对零点），并检查can通讯连接



	std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
