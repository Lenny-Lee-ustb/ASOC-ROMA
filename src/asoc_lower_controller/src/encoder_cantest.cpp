#include "include/asoc_lower_platform.hpp"

using namespace std;

ros::Publisher encoder_low;
ros::Publisher encoder_high;
int flag;

Motor motor_low[4];

void rxThread_low(int s)
{
	int ID; // 7 bytes in a loop [0x07] [0x01](ID) [0x04] [0x88](low)[0x0D](high) [0x00] [0x00] 
	int i;
	float angle;
	struct can_frame frame_low;
	int nbytes_low;
	
    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes_low = read(s, &frame_low, sizeof(struct can_frame));
		if (nbytes_low < 0)
		{
			ROS_ERROR("Read Error");
			break;
		};
		// ROS_INFO("%d",int(frame_low.can_id));
		if(int(frame_low.can_id) < 0x200){
			ID = int(frame_low.data[1]-0x10); //encoder ID 0x11~0x14
			angle = float(int(frame_low.data[4] << 8)+frame_low.data[3])/4096.0*120;
			ROS_INFO("%d %.2f %ld",ID,angle,sizeof(frame_low));
		}
		else{
			ID = int(frame_low.can_id-0x200)-1; // motor ID 0x200~0x204
			motor_low[ID].angle = (frame_low.data[0] << 8)+ frame_low.data[1];
			motor_low[ID].realAngle = motor_low[ID].angle*360/8191;
			motor_low[ID].velocity = (frame_low.data[2] <<8) + frame_low.data[3];
			motor_low[ID].I = (frame_low.data[4] <<8) + frame_low.data[5];
			motor_low[ID].temperature = frame_low.data[6];
			ROS_INFO("%.2d %.2f",ID,motor_low[ID].velocity);
		}


		std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
    }

}

void txThread_low(int s)
{
    struct can_frame frame_low;
    
	frame_low.can_id = 0x001;
	frame_low.can_dlc = 4;
	frame_low.data[0] = 0x04; 
	frame_low.data[1] = 0x01; 
	frame_low.data[2] = 0x04; 
	frame_low.data[3] = 0xAA; 

	int nbytes_low;
        
	nbytes_low = write(s, &frame_low, sizeof(struct can_frame));
        
	if (nbytes_low == -1 ) {
		printf("send error\n");
	}

	std::this_thread::sleep_for(std::chrono::nanoseconds(2000000));
	ROS_INFO("send it!");
}


int main(int argc, char** argv) {
	flag = 0;

	ros::init(argc,argv,"encoder_can");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	signal(SIGINT, signalCallback);

	int s_low;
	struct sockaddr_can addr_low;
	struct ifreq ifr_low;

	if ((s_low = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr_low.ifr_name, "can0");
	ioctl(s_low, SIOCGIFINDEX, &ifr_low);

	addr_low.can_family = AF_CAN;
	addr_low.can_ifindex = ifr_low.ifr_ifindex;

	printf("%s at index %d\n", ifr_low.ifr_name, ifr_low.ifr_ifindex);

	if (bind(s_low, (struct sockaddr *)&addr_low, sizeof(addr_low)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	std::thread canRx_low(rxThread_low, s_low);
	sleep(0.5);
	std::thread canTx_low(txThread_low, s_low);

	while (ros::ok())
    {
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}