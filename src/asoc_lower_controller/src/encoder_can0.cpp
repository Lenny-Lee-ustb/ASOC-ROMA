#include "include/asoc_lower_platform.hpp"

using namespace std;

int flag;
int rxCounter_low;

void rxThread_low(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame_low_t;
	int nbytes_low;

    encoder.data.resize(1);

    rxCounter_low= 0;
	
    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes_low = read(s, &frame_low_t, sizeof(struct can_frame));
		if (nbytes_low < 0)
		{
			perror("Read");
			break;
		}

		ID = int(frame_low_t.data[1]);

		rxCounter_low++;

        motor_low[ID].angle = (frame_low.data[0] << 8)+ frame_low.data[1];
		motor_low[ID].realAngle = motor_low[ID].angle*360/8191;
		motor_low[ID].velocity = (frame_low.data[2] <<8) + frame_low.data[3];
		motor_low[ID].I = (frame_low.data[4] <<8) + frame_low.data[5];
		motor_low[ID].temperature = frame_low.data[6];

        if (i>4){
            motor_low[ID].angleDifference = motor_low[ID].angle - motor_low[ID].angleLast;
        }

        if(motor_low[ID].angleDifference<-4000)
        {
            motor_low[ID].NumOfTurns++;
        }
        if(motor_low[ID].angleDifference>4000)
        {
            motor_low[ID].NumOfTurns--;
        }

        motor_low[ID].position = 8192*motor_low[ID].NumOfTurns+motor_low[ID].angle;
        motor_low[ID].angleLast = motor_low[ID].angle;

        velocityMessage_low.data[ID] = motor_low[ID].velocity;
		velocityMessage_low.data[4] = frame_vt;
		IMessage_low.data[ID] = motor_low[ID].I;

		if(i%4==0)
		{
			ROS_INFO("leg_angle is %f, %f, %f, %f\r\n",motor_low[0].leg_angle,motor_low[2].leg_angle,motor_high[0].leg_angle,motor_high[2].leg_angle);
			ROS_INFO("ori is %f, %f, %f, %f\r\n",motor_low[0].ori_encoder,motor_low[2].ori_encoder,motor_high[0].ori_encoder,motor_high[2].ori_encoder);
            ROS_INFO("target leg are %f, %f, %f, %f\r\n",motor_low[0].target_leg,motor_low[2].target_leg,motor_high[0].target_leg,motor_high[2].target_leg);
			ROS_INFO("VT %f\r\n",frame_vt);
            ROS_INFO("VN %f\r\n",frame_vn);
            ROS_INFO("W is %f\r\n", frame_w);
            ROS_INFO("target V is %f, %f, %f, %f, %f, %f, %f, %f \r\n",motor_low[0].targetVelocity,motor_low[1].targetVelocity,motor_low[2].targetVelocity,motor_low[3].targetVelocity,motor_high[0].targetVelocity,motor_high[1].targetVelocity,motor_high[2].targetVelocity,motor_high[3].targetVelocity);

            velocityPub_low.publish(velocityMessage_low);
			IPub_low.publish(IMessage_low);

            // std::ofstream fout;
			// fout.open("speed.txt",ios::app);
			// for(int i=0; i<4; i++){
			// 	fout<<motor_low[i].leg_angle<<"\t";
			// }
			// for(int i=0; i<4; i++){
			// 	fout<<motor_high[i].leg_angle<<"\t";
			// }
			// fout<<"\r\n";
			// fout.close();

		}
        if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				motor_low[j].targetPosition = motor_low[j].position;
			}
			
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
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

	// std::thread canRx_low(rxThread_low, s_low);
	sleep(0.5);
	std::thread canTx_low(txThread_low, s_low);

	while (ros::ok())
    {
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}