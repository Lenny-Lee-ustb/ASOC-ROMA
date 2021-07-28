
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

int flag;

int rxCounter;

ros::Publisher positionPub;
ros::Publisher velocityPub;
ros::Publisher IPub;
ros::Publisher sendIPub;
ros::Publisher targetPositionPub;


ros::Subscriber PISub;
ros::Subscriber PSub;
ros::Subscriber targetPositionSub;
ros::Subscriber targetVelocitySub;
ros::Subscriber encoder_sub;
ros::Subscriber turn_sub;
ros::Subscriber encoder_turn_sub;
ros::Subscriber encoder_angle_sub;
ros::Subscriber encoder_dir;

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
    int16_t velocityDifference_last;
    int32_t velocity_diff;
	int32_t velocityDifferenceSum;

    int turns;
	float encoder_turns;
	float direction;



	float Ki;
	float Kp;
	
	float K;
    float K_encoder;
    float Kp_encoder;
    float Ki_encoder;
    float Kd_encoder;
    float turn_mod;
	float turn_real;
	int encoder_angle;         //real-time angles from the encoder
	int encoder_angle_with_turn;     //real-time angle = turns*360+encoder_angle
    int encoder_angle_last;
    int encoder_D;
    int target_encoder;          //the angle of the encoder when the wheels are set to the desired position
	int target_leg;        //the angle of the leg when it is in target position
	
	int ori_encoder;         //the angle of the encoder when it is turned on and the wheels are set to the forward position
    int encoder_difference;   
	int encoder_difference_leg;         
    int encoder_differenceSum;
	int leg_angle;              //angle of the leg. leg_angle shall be zero when the wheels are set to the forward position, and the maximum shall be 360.
	int leg_angle_with_turn;                      

};
Motor motor[4];

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

void dir_callback(std_msgs::Float32MultiArray DirMsg){
	motor[0].direction = DirMsg.data[1];
	motor[1].direction = DirMsg.data[1];
}

void encoder_callback(std_msgs::Float32MultiArray MultiAngleMsg, std_msgs::Float32MultiArray MultiTurnMsg){
    float turn_mod = fmod(MultiTurnMsg.data[1],3.0);
	motor[0].encoder_angle = MultiAngleMsg.data[1];
	motor[1].encoder_angle = MultiAngleMsg.data[1];
	motor[0].turns = floor(fabs(MultiTurnMsg.data[1]));
	motor[1].turns = floor(fabs(MultiTurnMsg.data[1]));
	if(0<=turn_mod<1 && motor[0].encoder_angle-motor[0].ori_encoder>0){
		motor[0].leg_angle = (motor[0].encoder_angle - motor[0].ori_encoder)/3;
	}
	else if(0<=turn_mod<1 && motor[0].encoder_angle-motor[0].ori_encoder<=0){
		motor[0].leg_angle = (360 - motor[0].ori_encoder + motor[0].encoder_angle)/3;
	}
	else if(1<=turn_mod<2 && motor[0].encoder_angle-motor[0].ori_encoder>0){
		motor[0].leg_angle = (360 + motor[0].encoder_angle-motor[0].ori_encoder)/3;
	}
	else if(1<=turn_mod<2 && motor[0].encoder_angle-motor[0].ori_encoder<=0){
		motor[0].leg_angle = (360 + 360 - motor[0].ori_encoder + motor[0].encoder_angle)/3;
	}
	else if(2<=turn_mod<3 && motor[0].encoder_angle-motor[0].ori_encoder>0){
		motor[0].leg_angle = (360+360+motor[0].encoder_angle-motor[0].ori_encoder)/3;
	}
	else if(2<=turn_mod<3 && motor[0].encoder_angle-motor[0].ori_encoder<=0){
		motor[0].leg_angle = (360+360+360-motor[0].ori_encoder+motor[0].encoder_angle)/3;
	}
	else if(-1<turn_mod<=0 && motor[0].encoder_angle-motor[0].ori_encoder<=0){
		motor[0].leg_angle = 360 - (motor[0].ori_encoder-motor[0].encoder_angle)/3;
	}
	else if(-1<turn_mod<=0 && motor[0].encoder_angle-motor[0].ori_encoder>0){
		motor[0].leg_angle = 360 - (motor[0].ori_encoder+360-motor[0].encoder_angle)/3;
	}
	else if(-2<turn_mod<=-1 && motor[0].encoder_angle-motor[0].ori_encoder<=0){
		motor[0].leg_angle = 360 - (360+motor[0].ori_encoder-motor[0].encoder_angle)/3;
	}
	else if(-2<turn_mod<=-1 && motor[0].encoder_angle-motor[0].ori_encoder>0){
		motor[0].leg_angle = 360 - (360+motor[0].ori_encoder+360-motor[0].encoder_angle)/3;
	}
	else if(-3<turn_mod<=-2 && motor[0].encoder_angle-motor[0].ori_encoder<=0){
		motor[0].leg_angle = 360-(360+360+motor[0].ori_encoder-motor[0].encoder_angle)/3;
	}
	else if(-3<turn_mod<=-2 && motor[0].encoder_angle-motor[0].ori_encoder>0){
		motor[0].leg_angle = 360-(360+360+motor[0].ori_encoder+360-motor[0].encoder_angle)/3;
	}
	printf("leg angle is %f\r\n", motor[0].leg_angle);
}

void encoder_turn_callback(std_msgs::Float32MultiArray MultiTurnMsg){
	motor[0].encoder_turns = (int)round(MultiTurnMsg.data[1]);
	motor[1].encoder_turns = (int)round(MultiTurnMsg.data[1]);
	motor[0].turn_mod = fmod(MultiTurnMsg.data[1],3.0);
	motor[1].turn_mod = fmod(MultiTurnMsg.data[1],3.0);
	if(MultiTurnMsg.data[1]>=0){
		motor[0].turns = floor(fabs(MultiTurnMsg.data[1]));
		motor[1].turns = floor(fabs(MultiTurnMsg.data[1]));
	}
	else{
		motor[0].turns = -floor(fabs(MultiTurnMsg.data[1]));
		motor[1].turns = -floor(fabs(MultiTurnMsg.data[1]));
	}
	
}

void encoder_angle_callback(std_msgs::Float32MultiArray MultiAngleMsg){
	motor[0].encoder_angle = (int)round(MultiAngleMsg.data[1]);
	motor[1].encoder_angle = (int)round(MultiAngleMsg.data[1]);
	for(int i=0; i<2; i++){
		if(motor[i].direction>0 && motor[i].encoder_angle>=motor[i].ori_encoder){
			motor[i].encoder_angle_with_turn = motor[i].turns * 360 + motor[i].encoder_angle - motor[i].ori_encoder;
			
		}
		else if(motor[i].direction>0 && motor[i].encoder_angle<motor[i].ori_encoder){
			motor[i].encoder_angle_with_turn = motor[i].turns * 360 + motor[i].encoder_angle + 360 - motor[i].ori_encoder;
			
		}
		else if(motor[i].direction<0 && motor[i].encoder_angle>=motor[i].ori_encoder){
			motor[i].encoder_angle_with_turn = motor[i].turns * 360 - motor[i].ori_encoder - 360 + motor[i].encoder_angle;
			
		}
		else if(motor[i].direction<0 && motor[i].encoder_angle<motor[i].ori_encoder){
			motor[i].encoder_angle_with_turn = motor[i].turns * 360 - motor[i].ori_encoder + motor[i].encoder_angle;
			
		}
		motor[i].leg_angle_with_turn = motor[i].encoder_angle_with_turn/3;
		motor[i].leg_angle = motor[i].leg_angle_with_turn%360;
	}
}




void printfSingleMotor(Motor motor){
	printf("angle is %d,realAngle is %d,position is %d,velocity is %d,I is %d\r\n",
	motor.angle,motor.realAngle,motor.position,motor.velocity,motor.I);

}

void printfMultiMotorAngle(void){
	printf("angle is %d,%d,%d,%d;\r\n",motor[0].angle,motor[1].angle,motor[2].angle,motor[3].angle);
}

void printfMultiMotorPosition(void){
	printf("position is %d,%d,%d,%d;\r\n",motor[0].position,motor[1].position,motor[2].position,motor[3].position);
}

void printfMultiMotorVelocity(void){
	printf("velocity is %d,%d,%d,%d;\r\n",motor[0].velocity,motor[1].velocity,motor[2].velocity,motor[3].velocity);
}

void printfMultiMotorI(void){
	printf("I is %d,%d,%d,%d;\r\n",motor[0].I,motor[1].I,motor[2].I,motor[3].I);
}

std_msgs::Int32MultiArray positionMessage,velocityMessage,IMessage,sendIMessage,targetPositionMessage;


void rxThread(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame;
	int nbytes;
	positionMessage.data.resize(4);
	velocityMessage.data.resize(4);
	IMessage.data.resize(4);
	targetPositionMessage.data.resize(4);

    rxCounter= 0;
	for (j = 0;j<4;j++)
	{
		motor[j].angleDifference = 0;
		motor[j].NumOfTurns = 0;
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

        if (i>4){
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

		if(i%4==0)
		{
			printfMultiMotorPosition();
			printfMultiMotorVelocity();
			printfMultiMotorI();
			printf("tp is %d,%d\r\n",motor[0].targetPosition,motor[1].targetPosition,motor[2].targetPosition,motor[3].targetPosition);
			printf("k kp ki are %f,%frun,%f\r\n",motor[0].K,motor[0].Kp,motor[0].Ki);
            printf("encoder %d,%d\r\n",motor[0].encoder_angle,motor[0].encoder_difference);
            printf("target V is %d\r\n",motor[0].targetVelocity);
			printf("leg_angle is %d, %d\r\n",motor[0].leg_angle,motor[0].leg_angle_with_turn);
			printf("leg_tunr is %d\r\n",motor[0].turns);
			printf("ori is %d\r\n",motor[0].ori_encoder);
			printf("direction is %f\r\n",motor[0].direction);
			
			
			positionPub.publish(positionMessage);
			velocityPub.publish(velocityMessage);
			IPub.publish(IMessage);

		}
		if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				targetPositionMessage.data[j] = motor[j].position;
				motor[j].targetPosition = motor[j].position;
			}
			
		}
		if(i>16) targetPositionPub.publish(targetPositionMessage);
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}

void controlP_calSendI_PI(int ID){
	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	motor[ID].positionDifferenceSum = motor[ID].positionDifference + motor[ID].positionDifferenceSum;

	motor[ID].sendI = motor[ID].Kp*motor[ID].positionDifference + motor[ID].Ki*motor[ID].positionDifferenceSum;
}

void controlV_calSendI_PI(int ID){
	int limit = 16667;
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	if(motor[ID].velocityDifference > 1000) motor[ID].velocityDifference = 1000;
	if(motor[ID].velocityDifference < -1000) motor[ID].velocityDifference = -1000;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
	if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void controlP_calSendI_PPI(int ID){
	int limit = 16667;

	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	if(motor[ID].positionDifference > 4000) motor[ID].positionDifference = 4000;
	if(motor[ID].positionDifference < -4000) motor[ID].positionDifference = -4000;

	motor[ID].targetVelocity = motor[ID].K/100*(motor[ID].positionDifference);
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
	if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + 19*motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void control_encoder_speed(int ID){
	int limit = 16667;
	motor[ID].encoder_difference = motor[ID].target_leg - motor[ID].leg_angle;
	if(motor[ID].encoder_difference > 360) motor[ID].encoder_difference = 300;
	if(motor[ID].encoder_difference < -300) motor[ID].encoder_difference = -300;
	//motor_low[ID].angleDifferenceSum = motor_low[ID].angleDifference + motor_low[ID].angleDifferenceSum;

	motor[ID].targetVelocity = motor[ID].K_encoder*(motor[ID].encoder_difference);
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	//motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;
    motor[ID].velocity_diff = motor[ID].velocityDifference - motor[ID].velocityDifference_last;
    
    if(motor[ID].velocity_diff>500) motor[ID].velocity_diff = 500;
	if(motor[ID].velocity_diff<-500) motor[ID].velocity_diff = -500;


	// if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
	// if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

	motor[ID].sendI = motor[ID].Kp_encoder*motor[ID].velocityDifference + 19*motor[ID].Kd_encoder/10*motor[ID].velocity_diff;
    motor[ID].velocityDifference_last = motor[ID].velocityDifference;
}

void control_encoder_speed_leg(int ID){
	int limit = 16667;


	
	motor[ID].encoder_difference_leg = motor[ID].target_leg - motor[ID].leg_angle;
	if(motor[ID].encoder_difference_leg > 360) motor[ID].encoder_difference_leg = 360;
	if(motor[ID].encoder_difference_leg < -360) motor[ID].encoder_difference_leg = -360;
	//motor_low[ID].angleDifferenceSum = motor_low[ID].angleDifference + motor_low[ID].angleDifferenceSum;

	motor[ID].targetVelocity = motor[ID].K_encoder*(motor[ID].encoder_difference_leg);
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	//motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;
    motor[ID].velocity_diff = motor[ID].velocityDifference - motor[ID].velocityDifference_last;
    
    if(motor[ID].velocity_diff>500) motor[ID].velocity_diff = 500;
	if(motor[ID].velocity_diff<-500) motor[ID].velocity_diff = -500;


	// if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
	// if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

	motor[ID].sendI = motor[ID].Kp_encoder*motor[ID].velocityDifference + 19*motor[ID].Kd_encoder/10*motor[ID].velocity_diff;
    motor[ID].velocityDifference_last = motor[ID].velocityDifference;
}

void control_encoder(int ID){
    int limit = 16667;
    motor[ID].encoder_difference = (motor[ID].target_encoder - motor[ID].encoder_angle)/3;
	motor[ID].encoder_differenceSum = motor[ID].encoder_difference + motor[ID].encoder_differenceSum;

    motor[ID].encoder_angle_last = motor[ID].encoder_angle;

	motor[ID].sendI = motor[ID].Kp_encoder*motor[ID].encoder_difference + motor[ID].Ki_encoder*motor[ID].encoder_differenceSum;
    if(motor[ID].sendI>3000)  motor[ID].sendI=3000;
    if(motor[ID].sendI<-3000) motor[ID].sendI=-3000;
}
void txThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	int j;
	sendIMessage.data.resize(4);
	for (j = 0; j < 4; j++)
	{
		motor[j].sendI = 0;
		frame.data[2*j] = motor[j].sendI << 8;
		frame.data[2*j+1] = motor[j].sendI << 0;
	}

	int nbytes;

    for (int i = 0;; i++)
	{
		ros::spinOnce();
		if (flag ==1)
		{
			// controlP_calSendI_PPI(0);
			// controlP_calSendI_PPI(1);
			// controlP_calSendI_PPI(2);
			// controlP_calSendI_PPI(3);
			// controlV_calSendI_PI(0);
			// controlV_calSendI_PI(1);
			// controlV_calSendI_PI(2);
			// controlV_calSendI_PI(3);
			// controlP_calSendI_PI(0);
			// controlP_calSendI_PI(1);
			// controlP_calSendI_PI(2);
			// controlP_calSendI_PI(3);
            // control_encoder(0);
            // control_encoder(1);
            control_encoder_speed_leg(0);
            control_encoder_speed_leg(1);
		}
		
		for (j = 0;j<4;j++)
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
	flag = 0;

	ros::init(argc,argv,"canTest3");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	//std_msgs::Float32MultiArray_

	for(int i = 0;i<4;i++)
	{
		
		
		// if(i==0 || i==2)
		// {
		// 	motor[i].targetVelocity = 700;
		// }
		// else
		// {
		// 	motor[i].targetVelocity = -700;
		// }
		
		// if(i==0 || i==2)
		// {
		// 	motor[i].targetPosition = 7000;
		// }
		// else
		// {
		// 	motor[i].targetPosition = -7000;
		// }
		motor[i].Kp = 13;
		motor[i].Ki = 6;
		motor[i].K = 15;
        motor[i].K_encoder = 10;
        motor[i].target_encoder = 175;
		motor[i].targetPosition = 0;
		motor[i].ori_encoder = 175;
        motor[i].Kp_encoder = 15;
        motor[i].Ki_encoder = 5;
        motor[i].Kd_encoder = 10;
		motor[i].target_leg = 0;

	}

    positionPub = n.advertise<std_msgs::Int32MultiArray>("position",100);
    velocityPub = n.advertise<std_msgs::Int32MultiArray>("velocity",100);
    IPub = n.advertise<std_msgs::Int32MultiArray>("I",100);
	sendIPub = n.advertise<std_msgs::Int32MultiArray>("sendI",100);
	targetPositionPub = n.advertise<std_msgs::Int32MultiArray>("targetPositionFromPosition",100);


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
	targetVelocitySub = n.subscribe("targetVelocity", 10, targetVelocity_Callback);
	targetPositionSub = n.subscribe("targetPosition", 10, targetPosition_Callback);
    //encoder_sub = n.subscribe<std_msgs::Float32MultiArray>("SingleAngle",10,boost::bind(&encoder_callback, _1, _2));
	encoder_turn_sub = n.subscribe("MultiTurn",10,encoder_turn_callback);
	encoder_angle_sub = n.subscribe("MultiAngle",10,encoder_angle_callback);
	encoder_dir = n.subscribe("Dir",10,dir_callback);

	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
