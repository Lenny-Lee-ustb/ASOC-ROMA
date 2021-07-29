#include "ros/ros.h"

#include <unistd.h>

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>


uint16_t KP,KI;

int rxCounter;

ros::Publisher positionPub;
ros::Publisher velocityPub;
ros::Publisher IPub;
ros::Publisher sendIPub;

ros::Subscriber PISub;
ros::Subscriber PSub;
ros::Subscriber targetPositionSub;
ros::Subscriber targetVelocitySub;
ros::Subscriber encoder_sub;

struct Motor{
	int16_t angle,velocity,I;
	uint8_t temperature;
	int16_t realAngle;
    int32_t position;
    int16_t NumOfTurns;

	int32_t targetPosition;
	int16_t targetVelocity;

	int16_t angleDifference;
	int16_t angleLast;

	int16_t sendI;

	int32_t positionDifference;
	int32_t positionDifferenceSum;

	int16_t velocityDifference;
	int32_t velocityDifferenceSum;

	float Ki;
	float Kp;
	
	float K;
    float K_encoder;
    float encoder_angle;
    float target_encoder;
    float encoder_difference;

};
Motor motor[2];

void PI_Callback(std_msgs::Float32MultiArray PIMessage)
{
	int ID = 0;
    motor[ID].Kp = PIMessage.data[0];
    motor[ID].Ki = PIMessage.data[1];
}

void P_Callback(std_msgs::Float32 PMessage)
{
	int ID = 0;
	motor[ID].K = PMessage.data;
}

void targetPosition_Callback(std_msgs::Float32 targetPositionMessage){
	int ID = 0;
	motor[ID].targetPosition = targetPositionMessage.data;

}

void targetVelocity_Callback(std_msgs::Float32 targetVelocityMessage){
	int ID = 0;
	motor[ID].targetVelocity = targetVelocityMessage.data;
}

void encoder_callback(std_msgs::Float32MultiArray encoder_msg){
    motor[0].encoder_angle = encoder_msg.data[2];
	motor[1].encoder_angle = encoder_msg.data[2];
}


void printfSingleMotor(Motor motor){
	printf("angle is %d,realAngle is %d,position is %d,velocity is %d,I is %d\r\n",
	motor.angle,motor.realAngle,motor.position,motor.velocity,motor.I);

}

void printfMultiMotorAngle(void){
	printf("angle is %d,%d;\r\n",motor[0].angle,motor[1].angle);
}

void printfMultiMotorPosition(void){
	printf("position is %d,%d;\r\n",motor[0].position,motor[1].position);
}

void printfMultiMotorVelocity(void){
	printf("velocity is %d,%d;\r\n",motor[0].velocity,motor[1].velocity);
}

void printfMultiMotorI(void){
	printf("I is %d,%d;\r\n",motor[0].I,motor[1].I);
}

std_msgs::Int32MultiArray positionMessage,velocityMessage,IMessage,sendIMessage;

void rxThread(int s)
{
	int ID;
	int i;
	struct can_frame frame;
	int nbytes;
	positionMessage.data.resize(2);
	velocityMessage.data.resize(2);
	IMessage.data.resize(2);

    rxCounter= 0;
	for (i = 0;i<2;i++)
	{
		motor[i].angleDifference = 0;
		motor[i].NumOfTurns = 0;
	}

    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0)
		{
			perror("Read");
			break;
		}

		ID = int(frame.can_id-0x200)-1;

		rxCounter++;

        motor[ID].angle = (frame.data[0] << 8)+ frame.data[1];
		motor[ID].realAngle = motor[ID].angle*360/8191;
		motor[ID].velocity = (frame.data[2] <<8) + frame.data[3];
		motor[ID].I = (frame.data[4] <<8) + frame.data[5];
		motor[ID].temperature = frame.data[6];

        if (i>2){
            motor[ID].angleDifference = motor[ID].angle - motor[ID].angleLast;
        }

        if(motor[ID].angleDifference<-4000)
        {
            motor[ID].NumOfTurns++;
        }
        if(motor[ID].angleDifference>4000)
        {
            motor[ID].NumOfTurns--;
        }

        motor[ID].position = 8192*motor[ID].NumOfTurns+motor[ID].angle;
        motor[ID].angleLast = motor[ID].angle;


		positionMessage.data[ID] = motor[ID].position;
		velocityMessage.data[ID] = motor[ID].velocity;
		IMessage.data[ID] = motor[ID].I;

		if(i%2==0)
		{
			printfMultiMotorPosition();
			printfMultiMotorVelocity();
			positionPub.publish(positionMessage);
			velocityPub.publish(velocityMessage);
			IPub.publish(IMessage);

		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}

void controlP_calSendI_PI(int ID){
	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	motor[ID].positionDifferenceSum = motor[ID].positionDifference + motor[ID].positionDifferenceSum;

	motor[ID].sendI = motor[ID].Kp*motor[ID].positionDifference + motor[ID].Ki*motor[ID].positionDifferenceSum;
}

void controlV_calSendI_PI(int ID){
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	if(motor[ID].velocityDifference > 1000) motor[ID].velocityDifference = 1000;
	if(motor[ID].velocityDifference < -1000) motor[ID].velocityDifference = -1000;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void controlP_calSendI_PPI(int ID){
	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	if(motor[ID].positionDifference > 4000) motor[ID].positionDifference = 4000;
	if(motor[ID].positionDifference < -4000) motor[ID].positionDifference = -4000;

	motor[ID].targetVelocity = motor[ID].K/100*(motor[ID].positionDifference);
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void control_encoder_speed(int ID){
	int limit = 16667;
	if(ID==0){
		motor[ID].encoder_difference = motor[ID].target_encoder - motor[ID].encoder_angle;
		if(motor[ID].encoder_difference > 120) motor[ID].encoder_difference = 120;
		if(motor[ID].encoder_difference < -120) motor[ID].encoder_difference = -120;
		//motor_low[ID].angleDifferenceSum = motor_low[ID].angleDifference + motor_low[ID].angleDifferenceSum;

		motor[ID].targetVelocity = motor[ID].K_encoder/100*(motor[ID].encoder_difference);
		motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
		motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

		if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
		if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

		motor[ID].sendI = 50*motor[ID].Kp*motor[ID].velocityDifference + 19*motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
	}
	else if(ID==1){
		motor[ID].encoder_difference = motor[ID].encoder_angle - motor[ID].target_encoder;
		if(motor[ID].encoder_difference > 120) motor[ID].encoder_difference = 120;
		if(motor[ID].encoder_difference < -120) motor[ID].encoder_difference = -120;
		//motor_low[ID].angleDifferenceSum = motor_low[ID].angleDifference + motor_low[ID].angleDifferenceSum;

		motor[ID].targetVelocity = motor[ID].K_encoder/100*(motor[ID].encoder_difference);
		motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
		motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

		if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
		if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

		motor[ID].sendI = 50*motor[ID].Kp*motor[ID].velocityDifference + 19*motor[ID].Ki/10*motor[ID].velocityDifferenceSum;

	}
}

void txThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	int j;
	sendIMessage.data.resize(2);
	for (j = 0; j < 2; j++)
	{
		motor[j].sendI = 0;
		frame.data[2*j] = motor[j].sendI << 8;
		frame.data[2*j+1] = motor[j].sendI << 0;
	}

	int ID = 0;
	int nbytes;
	motor[ID].targetPosition = 0;
	motor[ID].targetVelocity = 0;
	motor[ID].Kp = 13;
	motor[ID].Ki = 6;
	motor[ID].K = 0;
    motor[ID].K_encoder = 15;
    for (int i = 0;; i++)
	{
		ros::spinOnce();
		motor[0].sendI = 500;
		motor[1].sendI = -500;


		for (j = 0;j<2;j++)
		{
			if (motor[j].sendI >15000) {
				motor[j].sendI = 15000;
			}
			if (motor[j].sendI <-15000) {
				motor[j].sendI = -15000;
			}
			frame.data[2*j] = motor[j].sendI>>8;
			frame.data[2*j+1] = motor[j].sendI>>0;
			sendIMessage.data[j] = motor[j].sendI;

		}
		sendIPub.publish(sendIMessage);
        nbytes = write(s, &frame, sizeof(struct can_frame));
        if (nbytes == -1) {
			printf("send error\n");
        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }

}



int main(int argc, char** argv) {

	ros::init(argc,argv,"canTest2");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	for(int i = 0;i<2;i++)
	{
		motor[i].Ki = 0;
		motor[i].Ki = 0;

		motor[i].targetPosition = 0;
		motor[i].targetVelocity = 0;
	}

    positionPub = n.advertise<std_msgs::Int32MultiArray>("position",100);
    velocityPub = n.advertise<std_msgs::Int32MultiArray>("velocity",100);
    IPub = n.advertise<std_msgs::Int32MultiArray>("I",100);
	sendIPub = n.advertise<std_msgs::Int32MultiArray>("sendI",100);


	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	std::thread canRx(rxThread, s);
	sleep(0.5);
	std::thread canTx(txThread, s);
    PISub = n.subscribe("PI", 10, PI_Callback);
	PSub = n.subscribe("P",10,P_Callback);
	targetVelocitySub = n.subscribe("targetVelocity", 100, targetVelocity_Callback);
	targetPositionSub = n.subscribe("targetPosition", 100, targetPosition_Callback);
    encoder_sub = n.subscribe("SingleAngle",10,encoder_callback);

	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}