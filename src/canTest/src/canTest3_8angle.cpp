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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <iostream>
#include <stdio.h>

uint16_t KP,KI;

int flag;

int rxCounter_low;
int rxCounter_high;

int judge_forward=0;         //high[1],high[2] angle 1
int judge_backward=0;        //high[3],high[4]   angle 0
int judge_left=0;            //low[1],low[2]    angle 2
int judge_right=0;             //low[3],low[4]   angle 3
int co =0;
int judge_angle1 = 0;
int judge_angle2 = 0;

ros::Publisher positionPub_low;
ros::Publisher positionPub_high;
ros::Publisher velocityPub_low;
ros::Publisher velocityPub_high;
ros::Publisher IPub_low;
ros::Publisher IPub_high;
ros::Publisher sendIPub_low;
ros::Publisher sendIPub_high;
ros::Publisher targetPositionPub_low;
ros::Publisher targetPositionPub_high;
ros::Publisher vel_pub_low;
ros::Publisher vel_pub_high;

ros::Subscriber PISub_low;
ros::Subscriber PISub_high;
ros::Subscriber PSub_low;
ros::Subscriber PSub_high;
ros::Subscriber targetPositionSub_low;
ros::Subscriber targetPositionSub_high;
ros::Subscriber targetVelocitySub_low;
ros::Subscriber targetVelocitySub_high;
ros::Subscriber joy_sub;
ros::Subscriber encoder_turn_sub;
ros::Subscriber encoder_angle_sub;
ros::Subscriber encoder_dir;
ros::Subscriber encoder_angle_sum;

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
    float Kd;
	
	float K;
    float K_encoder;
    float Kp_encoder;
    float Ki_encoder;
    float Kd_encoder;
    float turn_mod;
	float turn_real;
	float num_360;
	float encoder_angle;         //real-time angles from the encoder
	float encoder_angle_with_turn;     //real-time angle = turns*360+encoder_angle
    float encoder_angle_last;
    float encoder_D;
    float target_encoder;          //the angle of the encoder when the wheels are set to the desired position
	float target_leg;        //the angle of the leg when it is in target position
	
	float ori_encoder;         //the angle of the encoder when it is turned on and the wheels are set to the forward position
    float encoder_difference;   
	float encoder_difference_leg;         
    float encoder_differenceSum;
	float leg_angle;              //angle of the leg. leg_angle shall be zero when the wheels are set to the forward position, and the maximum shall be 360.
	float leg_angle_with_turn;                      

};
Motor motor_low[4];
Motor motor_high[4];
using namespace std;
void PI_Callback_low(std_msgs::Float32MultiArray PIMessage_low)
{
	int ID = 0;
    motor_low[ID].Kp = PIMessage_low.data[0];
    motor_low[ID].Ki = PIMessage_low.data[1];
}

void PI_Callback_high(std_msgs::Float32MultiArray PIMessage_high)
{
	int ID = 0;
    motor_high[ID].Kp = PIMessage_high.data[0];
    motor_high[ID].Ki = PIMessage_high.data[1];
}

void P_Callback_low(std_msgs::Float32 PMessage_low)
{
	int ID = 0;
	motor_low[ID].K = PMessage_low.data;
}

void P_Callback_high(std_msgs::Float32 PMessage_high)
{
	int ID = 0;
	motor_high[ID].K = PMessage_high.data;
}

void targetPosition_Callback_low(std_msgs::Float32 targetPositionMessage_low){
	int ID = 0;
	motor_low[ID].targetPosition = targetPositionMessage_low.data;

}

void targetPosition_Callback_high(std_msgs::Float32 targetPositionMessage_high){
	int ID = 0;
	motor_high[ID].targetPosition = targetPositionMessage_high.data;

}

void targetVelocity_Callback_low(std_msgs::Float32 targetVelocityMessage_low){
	int ID = 0;
	motor_low[ID].targetVelocity = targetVelocityMessage_low.data;
}

void targetVelocity_Callback_high(std_msgs::Float32 targetVelocityMessage_high){
	int ID = 0;
	motor_high[ID].targetVelocity = targetVelocityMessage_high.data;
}

void dir_callback(std_msgs::Float32MultiArray DirMsg){
	motor_low[0].direction = DirMsg.data[2];
	motor_low[1].direction = DirMsg.data[2];
    motor_low[2].direction = DirMsg.data[3];
	motor_low[3].direction = DirMsg.data[3];
    motor_high[0].direction = DirMsg.data[1];
	motor_high[1].direction = DirMsg.data[1];
    motor_high[2].direction = DirMsg.data[0];
	motor_high[3].direction = DirMsg.data[0];
	// for(int i = 0; i < 4; i++){
	// 	if(motor_low[i].turns < 0){
	// 		motor_low[i].direction = -1;
	// 	}
	// 	else if(motor_low[i].turns > 0){
	// 		motor_low[i].direction = 1;
	// 	}
	// 	else if(motor_low[i].turns == 0){
	// 		if(motor_low[i].encoder_angle >= motor_low[i].ori_encoder){
	// 			motor_low[i].direction = 1;
	// 		}
	// 		else{
	// 			motor_low[i].direction = -1;
	// 		}
	// 	}

	// 	if(motor_high[i].turns < 0){
	// 		motor_high[i].direction = -1;
	// 	}
	// 	else if(motor_high[i].turns > 0){
	// 		motor_high[i].direction = 1;
	// 	}
	// 	else if(motor_high[i].turns == 0){
	// 		if(motor_high[i].encoder_angle >= motor_high[i].ori_encoder){
	// 			motor_high[i].direction = 1;
	// 		}
	// 		else{
	// 			motor_high[i].direction = -1;
	// 		}
	// 	}
	// }
}

void encoder_angle_sum_callback(std_msgs::Float32MultiArray MultiAngleSumMsg){
	motor_low[0].encoder_angle_with_turn = MultiAngleSumMsg.data[2];
	motor_low[1].encoder_angle_with_turn = MultiAngleSumMsg.data[2];
	motor_low[2].encoder_angle_with_turn = MultiAngleSumMsg.data[3];
	motor_low[3].encoder_angle_with_turn = MultiAngleSumMsg.data[3];
	motor_high[0].encoder_angle_with_turn = MultiAngleSumMsg.data[1];
	motor_high[1].encoder_angle_with_turn = MultiAngleSumMsg.data[1];
	motor_high[2].encoder_angle_with_turn = MultiAngleSumMsg.data[0];
	motor_high[3].encoder_angle_with_turn = MultiAngleSumMsg.data[0];

	for(int i=0; i<4; i++){
		motor_low[i].leg_angle_with_turn = (motor_low[i].encoder_angle_with_turn-motor_low[i].ori_encoder)/3;
		if(motor_low[i].leg_angle_with_turn >= 0){
			motor_low[i].leg_angle = motor_low[i].leg_angle_with_turn - floor(motor_low[i].leg_angle_with_turn/360)*360;
		}
		else if(motor_low[i].leg_angle_with_turn < 0){
			motor_low[i].leg_angle = 360+motor_low[i].leg_angle_with_turn - ceil(motor_low[i].leg_angle_with_turn/360)*360;
		}

		motor_high[i].leg_angle_with_turn = (motor_high[i].encoder_angle_with_turn-motor_high[i].ori_encoder) /3;
		if(motor_high[i].leg_angle_with_turn >= 0){
			motor_high[i].leg_angle = motor_high[i].leg_angle_with_turn - floor(motor_high[i].leg_angle_with_turn/360)*360;
		}
		else if(motor_high[i].leg_angle_with_turn < 0){
			motor_high[i].leg_angle = 360+motor_high[i].leg_angle_with_turn - ceil(motor_high[i].leg_angle_with_turn/360)*360;
		}
	}
}
void encoder_turn_callback(std_msgs::Float32MultiArray MultiTurnMsg){
	
	if(MultiTurnMsg.data[2]>=0){
		motor_low[0].turns = floor(fabs(MultiTurnMsg.data[2]));
		motor_low[1].turns = floor(fabs(MultiTurnMsg.data[2]));
	}
	else{
	    motor_low[0].turns = -floor(fabs(MultiTurnMsg.data[2]));
		motor_low[1].turns = -floor(fabs(MultiTurnMsg.data[2]));
	}
    if(MultiTurnMsg.data[3]>=0){
		motor_low[2].turns = floor(fabs(MultiTurnMsg.data[3]));
		motor_low[3].turns = floor(fabs(MultiTurnMsg.data[3]));
	}
	else{
	    motor_low[2].turns = -floor(fabs(MultiTurnMsg.data[3]));
		motor_low[3].turns = -floor(fabs(MultiTurnMsg.data[3]));
	}
    if(MultiTurnMsg.data[1]>=0){
		motor_high[0].turns = floor(fabs(MultiTurnMsg.data[1]));
		motor_high[1].turns = floor(fabs(MultiTurnMsg.data[1]));
	}
	else{
	    motor_high[0].turns = -floor(fabs(MultiTurnMsg.data[1]));
		motor_high[1].turns = -floor(fabs(MultiTurnMsg.data[1]));
	}
    if(MultiTurnMsg.data[0]>=0){
		motor_high[2].turns = floor(fabs(MultiTurnMsg.data[0]));
		motor_high[3].turns = floor(fabs(MultiTurnMsg.data[0]));
	}
	else{
	    motor_high[2].turns = -floor(fabs(MultiTurnMsg.data[0]));
		motor_high[3].turns = -floor(fabs(MultiTurnMsg.data[0]));
	}
	
}

void encoder_angle_callback(std_msgs::Float32MultiArray MultiAngleMsg){
	// motor_low[0].encoder_angle = MultiAngleMsg.data[2];
	// motor_low[1].encoder_angle = MultiAngleMsg.data[2];
    // motor_low[2].encoder_angle = MultiAngleMsg.data[3];
	// motor_low[3].encoder_angle = MultiAngleMsg.data[3];
    // motor_high[0].encoder_angle = MultiAngleMsg.data[1];
	// motor_high[1].encoder_angle = MultiAngleMsg.data[1];
    // motor_high[2].encoder_angle = MultiAngleMsg.data[0];
	// motor_high[3].encoder_angle = MultiAngleMsg.data[0];
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// for(int i = 0; i < 4; i++){
	// 	if(motor_low[i].turns < 0){
	// 		motor_low[i].direction = -1;
	// 	}
	// 	else if(motor_low[i].turns > 0){
	// 		motor_low[i].direction = 1;
	// 	}
	// 	else if(motor_low[i].turns == 0){
	// 		if(motor_low[i].encoder_angle >= motor_low[i].ori_encoder){
	// 			motor_low[i].direction = 1;
	// 		}
	// 		else{
	// 			motor_low[i].direction = -1;
	// 		}
	// 	}

	// 	if(motor_high[i].turns < 0){
	// 		motor_high[i].direction = -1;
	// 	}
	// 	else if(motor_high[i].turns > 0){
	// 		motor_high[i].direction = 1;
	// 	}
	// 	else if(motor_high[i].turns == 0){
	// 		if(motor_high[i].encoder_angle >= motor_high[i].ori_encoder){
	// 			motor_high[i].direction = 1;
	// 		}
	// 		else{
	// 			motor_high[i].direction = -1;
	// 		}
	// 	}
	// }

	for(int i=0; i<4; i++){
		if(motor_low[i].direction>0 && motor_low[i].encoder_angle>=motor_low[i].ori_encoder){
			motor_low[i].encoder_angle_with_turn = motor_low[i].turns * 360 + motor_low[i].encoder_angle - motor_low[i].ori_encoder;
			
		}
		else if(motor_low[i].direction>0 && motor_low[i].encoder_angle<motor_low[i].ori_encoder){
			motor_low[i].encoder_angle_with_turn = motor_low[i].turns * 360 + motor_low[i].encoder_angle + 360 - motor_low[i].ori_encoder;
			
		}
		else if(motor_low[i].direction<0 && motor_low[i].encoder_angle>=motor_low[i].ori_encoder){
			motor_low[i].encoder_angle_with_turn = motor_low[i].turns * 360 - motor_low[i].ori_encoder - 360 + motor_low[i].encoder_angle;
			
		}
		else if(motor_low[i].direction<0 && motor_low[i].encoder_angle<motor_low[i].ori_encoder){
			motor_low[i].encoder_angle_with_turn = motor_low[i].turns * 360 - motor_low[i].ori_encoder + motor_low[i].encoder_angle;
			
		}
        if(motor_high[i].direction>0 && motor_high[i].encoder_angle>=motor_high[i].ori_encoder){
			motor_high[i].encoder_angle_with_turn = motor_high[i].turns * 360 + motor_high[i].encoder_angle - motor_high[i].ori_encoder;
			
		}
		else if(motor_high[i].direction>0 && motor_high[i].encoder_angle<motor_high[i].ori_encoder){
			motor_high[i].encoder_angle_with_turn = motor_high[i].turns * 360 + motor_high[i].encoder_angle + 360 - motor_high[i].ori_encoder;
			
		}
		else if(motor_high[i].direction<0 && motor_high[i].encoder_angle>=motor_high[i].ori_encoder){
			motor_high[i].encoder_angle_with_turn = motor_high[i].turns * 360 - motor_high[i].ori_encoder - 360 + motor_high[i].encoder_angle;
			
		}
		else if(motor_high[i].direction<0 && motor_high[i].encoder_angle<motor_high[i].ori_encoder){
			motor_high[i].encoder_angle_with_turn = motor_high[i].turns * 360 - motor_high[i].ori_encoder + motor_high[i].encoder_angle;
			
		}
		
		motor_low[i].leg_angle_with_turn = motor_low[i].encoder_angle_with_turn/3;
		if(motor_low[i].leg_angle_with_turn >= 0){
			motor_low[i].leg_angle = motor_low[i].leg_angle_with_turn - floor(motor_low[i].leg_angle_with_turn/360)*360;
		}
		else if(motor_low[i].leg_angle_with_turn < 0){
			motor_low[i].leg_angle = 360+ motor_low[i].leg_angle_with_turn - ceil(motor_low[i].leg_angle_with_turn/360)*360;
		}
		
		//motor_low[i].leg_angle = fmod(motor_low[i].leg_angle_with_turn,360);
		//motor_low[i].leg_angle = motor_low[i].leg_angle_with_turn%360;
        motor_high[i].leg_angle_with_turn = motor_high[i].encoder_angle_with_turn/3;
		if(motor_high[i].leg_angle_with_turn >= 0){
			motor_high[i].leg_angle = motor_high[i].leg_angle_with_turn - floor(motor_high[i].leg_angle_with_turn/360)*360;
		}
		else if(motor_high[i].leg_angle_with_turn < 0){
			motor_high[i].leg_angle =  360+motor_high[i].leg_angle_with_turn - ceil(motor_high[i].leg_angle_with_turn/360)*360;
		}
		//motor_high[i].leg_angle = fmod(motor_high[i].leg_angle_with_turn,360);
		//motor_high[i].leg_angle = motor_high[i].leg_angle_with_turn%360;
	}
}




void printfSingleMotor(Motor motor){
	printf("angle is %d,realAngle is %d,position is %d,velocity is %d,I is %d\r\n",
	motor.angle,motor.realAngle,motor.position,motor.velocity,motor.I);

}

void printfMultiMotorAngle_low(void){
	printf("low_angle is %d,%d,%d,%d;\r\n",motor_low[0].angle,motor_low[1].angle,motor_low[2].angle,motor_low[3].angle);
}

void printfMultiMotorAngle_high(void){
	printf("high_angle is %d,%d,%d,%d;\r\n",motor_high[0].angle,motor_high[1].angle,motor_high[2].angle,motor_high[4].angle);
}

void printfMultiMotorPosition_low(void){
	printf("position is %d,%d,%d,%d;\r\n",motor_low[0].position,motor_low[1].position,motor_low[2].position,motor_low[3].position);
}

void printfMultiMotorPosition_high(void){
	printf("position is %d,%d,%d,%d;\r\n",motor_high[0].position,motor_high[1].position,motor_high[2].position,motor_high[3].position);
}

void printfMultiMotorVelocity_low(void){
	printf("velocity is %d,%d,%d,%d;\r\n",motor_low[0].velocity,motor_low[1].velocity,motor_low[2].velocity,motor_low[3].velocity);
}

void printfMultiMotorVelocity_high(void){
	printf("velocity is %d,%d,%d,%d;\r\n",motor_high[0].velocity,motor_high[1].velocity,motor_high[2].velocity,motor_high[3].velocity);
}

void printfMultiMotorI_low(void){
	printf("I is %d,%d,%d,%d;\r\n",motor_low[0].I,motor_low[1].I,motor_low[2].I,motor_low[3].I);
}

void printfMultiMotorI_high(void){
	printf("I is %d,%d,%d,%d;\r\n",motor_high[0].I,motor_high[1].I,motor_high[2].I,motor_high[3].I);
}

void buttonCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    judge_forward = joy->buttons[3];
	judge_backward = joy->buttons[0];
	judge_left = joy->buttons[2];
	judge_right = joy->buttons[1];
	judge_angle1 = joy->axes[6];
	judge_angle2 = joy->axes[7];

    for (int i=0; i<4; i++){
		// if(judge_forward==1){
		// 	if(i%2==0)
		// 	{
		// 		motor_low[i].targetVelocity = 700;
        //     	motor_high[i].targetVelocity = 700;
		// 	}
		// 	else
		// 	{
		// 		motor_low[i].targetVelocity = -700;
        //     	motor_high[i].targetVelocity = -700;
		// 	}
		// }
		// else if(judge_backward==1){
		// 	if(i%2==0)
		// 	{
		// 		motor_low[i].targetVelocity = -700;
        //     	motor_high[i].targetVelocity = -700;
		// 	}
		// 	else
		// 	{
		// 		motor_low[i].targetVelocity = 700;
        //     	motor_high[i].targetVelocity = 700;
		// 	}
		// }
		// else if(judge_left==1){
		// 	if(i%2==0)
		// 	{
		// 		motor_low[i].targetVelocity = 700;
        //     	motor_high[i].targetVelocity = 700;
		// 	}
		// 	else
		// 	{
		// 		motor_low[i].targetVelocity = 700;
        //     	motor_high[i].targetVelocity = 700;
		// 	}
		// }
		// else if(judge_right==1){
		// 	if(i%2==0)
		// 	{
		// 		motor_low[i].targetVelocity = -700;
        //     	motor_high[i].targetVelocity = -700;
		// 	}
		// 	else
		// 	{
		// 		motor_low[i].targetVelocity = -700;
        //     	motor_high[i].targetVelocity = -700;
		// 	}
		// }
		// else{
		// 	if(i%2==0)
		// 	{
		// 		motor_low[i].targetVelocity = -700;
        //     	motor_high[i].targetVelocity = -700;
		// 	}
		// 	else
		// 	{
		// 		motor_low[i].targetVelocity = -700;
        //     	motor_high[i].targetVelocity = -700;
		// 	}
		// }
		if(judge_angle1 == 1){
			motor_low[i].target_leg = 90;
			motor_high[i].target_leg = 90;
		}
		if(judge_angle1 == -1){
			motor_low[i].target_leg = 30;
			motor_high[i].target_leg = 30;
		}
		if(judge_angle2 == 1){
			motor_low[i].target_leg = 270;
			motor_high[i].target_leg = 270;
		}
		if(judge_angle2 == -1){
			motor_low[i].target_leg = 180;
			motor_high[i].target_leg = 180;
		}
	}
}

std_msgs::Int32MultiArray positionMessage_low,positionMessage_high,velocityMessage_low,velocityMessage_high,IMessage_low,IMessage_high,sendIMessage_low,sendIMessage_high,targetPositionMessage_low,targetPositionMessage_high,targetVelocityMessage_low,targetVelocityMessage_high;


void rxThread_low(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame_low;
	int nbytes_low;
	positionMessage_low.data.resize(4);
	velocityMessage_low.data.resize(4);
	IMessage_low.data.resize(4);
	targetPositionMessage_low.data.resize(4);

    rxCounter_low= 0;
	for (j = 0;j<4;j++)
	{
		motor_low[j].angleDifference = 0;
		motor_low[j].NumOfTurns = 0;
	}

    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes_low = read(s, &frame_low, sizeof(struct can_frame));
		if (nbytes_low < 0)
		{
			perror("Read");
			break;
		}

		ID = int(frame_low.can_id-0x200)-1;

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


		positionMessage_low.data[ID] = motor_low[ID].position;
		velocityMessage_low.data[ID] = motor_low[ID].velocity;
		IMessage_low.data[ID] = motor_low[ID].I;

		if(i%4==0)
		{
			printfMultiMotorPosition_low();
			printfMultiMotorVelocity_low();
			printfMultiMotorI_low();
			printf("LOW tp is %d,%d\r\n",motor_low[0].targetPosition,motor_low[1].targetPosition,motor_low[2].targetPosition,motor_low[3].targetPosition);
			printf("LOW k kp ki are %f,%frun,%f\r\n",motor_low[0].K,motor_low[0].Kp,motor_low[0].Ki);
			printf("encoder %f,%f\r\n",motor_low[0].encoder_angle,motor_low[0].encoder_difference);
            printf("target V is %d\r\n",motor_low[0].targetVelocity);
			printf("leg_angle is  %f,  %f,  %f,  %f\r\n",motor_low[0].leg_angle,motor_low[2].leg_angle,motor_high[0].leg_angle,motor_high[2].leg_angle);
			printf("leg_tunr is  %f,  %f,  %f,  %f\r\n",motor_low[0].turns,motor_low[2].turns,motor_high[0].turns,motor_high[2].turns);
			printf("ori is %f, %f, %f, %f\r\n",motor_low[0].ori_encoder,motor_low[2].ori_encoder,motor_high[0].ori_encoder,motor_high[2].ori_encoder);
			printf("direction is %f, %f, %f, %f\r\n",motor_low[0].direction,motor_low[2].direction,motor_high[0].direction,motor_high[2].direction);
            printf("target leg are %f, %f, %f, %f\r\n",motor_low[0].target_leg,motor_low[2].target_leg,motor_high[0].target_leg,motor_high[2].target_leg);
			positionPub_low.publish(positionMessage_low);
			velocityPub_low.publish(velocityMessage_low);
			IPub_low.publish(IMessage_low);
			ofstream fout;
			fout.open("speed.txt",ios::app);
			for(int i=0; i<4; i++){
				fout<<motor_low[i].leg_angle<<"\t";
			}
			for(int i=0; i<4; i++){
				fout<<motor_high[i].leg_angle<<"\t";
			}
			fout<<"\r\n";
			fout.close();

		}
		if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				targetPositionMessage_low.data[j] = motor_low[j].position;
				motor_low[j].targetPosition = motor_low[j].position;
			}
			
		}
		if(i>16) targetPositionPub_low.publish(targetPositionMessage_low);
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}

void rxThread_high(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame_high;
	int nbytes_high;
	positionMessage_high.data.resize(4);
	velocityMessage_high.data.resize(4);
	IMessage_high.data.resize(4);
	targetPositionMessage_high.data.resize(4);

    rxCounter_high= 0;
	for (j = 0;j<4;j++)
	{
		motor_high[j].angleDifference = 0;
		motor_high[j].NumOfTurns = 0;
	}

    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes_high = read(s, &frame_high, sizeof(struct can_frame));
		if (nbytes_high < 0)
		{
			perror("Read");
			break;
		}

		ID = int(frame_high.can_id-0x200)-1;

		rxCounter_high++;

        motor_high[ID].angle = (frame_high.data[0] << 8)+ frame_high.data[1];
		motor_high[ID].realAngle = motor_high[ID].angle*360/8191;
		motor_high[ID].velocity = (frame_high.data[2] <<8) + frame_high.data[3];
		motor_high[ID].I = (frame_high.data[4] <<8) + frame_high.data[5];
		motor_high[ID].temperature = frame_high.data[6];

        if (i>4){
            motor_high[ID].angleDifference = motor_high[ID].angle - motor_high[ID].angleLast;
        }

        if(motor_high[ID].angleDifference<-4000)
        {
            motor_high[ID].NumOfTurns++;
        }
        if(motor_high[ID].angleDifference>4000)
        {
            motor_high[ID].NumOfTurns--;
        }

        motor_high[ID].position = 8192*motor_high[ID].NumOfTurns+motor_high[ID].angle;
        motor_high[ID].angleLast = motor_high[ID].angle;


		positionMessage_high.data[ID] = motor_high[ID].position;
		velocityMessage_high.data[ID] = motor_high[ID].velocity;
		IMessage_high.data[ID] = motor_high[ID].I;

		if(i%4==0)
		{
			// printfMultiMotorPosition_high();
			// printfMultiMotorVelocity_high();
			// printfMultiMotorI_high();
			// printf("HIGH tp is %d,%d\r\n",motor_high[0].targetPosition,motor_high[1].targetPosition,motor_high[2].targetPosition,motor_high[3].targetPosition);
			// printf("HIGH k kp ki are %f,%frun,%f\r\n",motor_high[0].K,motor_high[0].Kp,motor_high[0].Ki);
			// printf("encoder %d,%d\r\n",motor_high[0].encoder_angle,motor_high[0].encoder_difference);
            // printf("target V is %d\r\n",motor_high[0].targetVelocity);
			// printf("leg_angle is %d, %d\r\n",motor_high[0].leg_angle,motor_high[0].leg_angle_with_turn);
			// printf("leg_tunr is %d\r\n",motor_high[0].turns);
			// printf("ori is %d\r\n",motor_high[0].ori_encoder);
			// printf("direction is %f\r\n",motor_high[0].direction);
            positionPub_high.publish(positionMessage_high);
			velocityPub_high.publish(velocityMessage_high);
			IPub_high.publish(IMessage_high);

		}
		if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				targetPositionMessage_high.data[j] = motor_high[j].position;
				motor_high[j].targetPosition = motor_high[j].position;
			}
			
		}
		if(i>16) targetPositionPub_high.publish(targetPositionMessage_high);
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}



void controlP_calSendI_PI_low(int ID){
	motor_low[ID].positionDifference = motor_low[ID].targetPosition - motor_low[ID].position;
	motor_low[ID].positionDifferenceSum = motor_low[ID].positionDifference + motor_low[ID].positionDifferenceSum;

	motor_low[ID].sendI = motor_low[ID].Kp*motor_low[ID].positionDifference + motor_low[ID].Ki*motor_low[ID].positionDifferenceSum;
}

void controlP_calSendI_PI_high(int ID){
	motor_high[ID].positionDifference = motor_high[ID].targetPosition - motor_high[ID].position;
	motor_high[ID].positionDifferenceSum = motor_high[ID].positionDifference + motor_high[ID].positionDifferenceSum;

	motor_high[ID].sendI = motor_high[ID].Kp*motor_high[ID].positionDifference + motor_high[ID].Ki*motor_high[ID].positionDifferenceSum;
}

void controlV_calSendI_PI_low(int ID){
	int limit = 16667;
	motor_low[ID].velocityDifference = motor_low[ID].targetVelocity - motor_low[ID].velocity;
	if(motor_low[ID].velocityDifference > 1000) motor_low[ID].velocityDifference = 1000;
	if(motor_low[ID].velocityDifference < -1000) motor_low[ID].velocityDifference = -1000;
	motor_low[ID].velocityDifferenceSum = motor_low[ID].velocityDifference + motor_low[ID].velocityDifferenceSum;

	if(motor_low[ID].velocityDifferenceSum>limit) motor_low[ID].velocityDifferenceSum = limit;
	if(motor_low[ID].velocityDifferenceSum<(0-limit)) motor_low[ID].velocityDifferenceSum = (0-limit);

	motor_low[ID].sendI = motor_low[ID].Kp*motor_low[ID].velocityDifference + motor_low[ID].Ki/10*motor_low[ID].velocityDifferenceSum;
}

void controlV_calSendI_PI_high(int ID){
	int limit = 16667;
	motor_high[ID].velocityDifference = motor_high[ID].targetVelocity - motor_high[ID].velocity;
	if(motor_high[ID].velocityDifference > 1000) motor_high[ID].velocityDifference = 1000;
	if(motor_high[ID].velocityDifference < -1000) motor_high[ID].velocityDifference = -1000;
	motor_high[ID].velocityDifferenceSum = motor_high[ID].velocityDifference + motor_high[ID].velocityDifferenceSum;

	if(motor_high[ID].velocityDifferenceSum>limit) motor_high[ID].velocityDifferenceSum = limit;
	if(motor_high[ID].velocityDifferenceSum<(0-limit)) motor_high[ID].velocityDifferenceSum = (0-limit);

	motor_high[ID].sendI = motor_high[ID].Kp*motor_high[ID].velocityDifference + motor_high[ID].Ki/10*motor_high[ID].velocityDifferenceSum;
}

void controlV_calSendI_PD_low(int ID){
	int limit = 16667;
	motor_low[ID].velocityDifference = motor_low[ID].targetVelocity - motor_low[ID].velocity;
	if(motor_low[ID].velocityDifference > 1000) motor_low[ID].velocityDifference = 1000;
	if(motor_low[ID].velocityDifference < -1000) motor_low[ID].velocityDifference = -1000;

	motor_low[ID].velocity_diff = motor_low[ID].velocityDifference - motor_low[ID].velocityDifference_last;
	if(motor_low[ID].velocity_diff>500) motor_low[ID].velocity_diff = 500;
	if(motor_low[ID].velocity_diff<-500) motor_low[ID].velocity_diff = -500;

	motor_low[ID].sendI = motor_low[ID].Kp*motor_low[ID].velocityDifference + motor_low[ID].Kd*motor_low[ID].velocity_diff;
	
	motor_low[ID].velocityDifference_last = motor_low[ID].velocityDifference;
}



void controlP_calSendI_PPI_low(int ID){
	int limit = 16667;

	motor_low[ID].positionDifference = motor_low[ID].targetPosition - motor_low[ID].position;
	if(motor_low[ID].positionDifference > 4000) motor_low[ID].positionDifference = 4000;
	if(motor_low[ID].positionDifference < -4000) motor_low[ID].positionDifference = -4000;

	motor_low[ID].targetVelocity = motor_low[ID].K/100*(motor_low[ID].positionDifference);
	motor_low[ID].velocityDifference = motor_low[ID].targetVelocity - motor_low[ID].velocity;
	motor_low[ID].velocityDifferenceSum = motor_low[ID].velocityDifference + motor_low[ID].velocityDifferenceSum;

	if(motor_low[ID].velocityDifferenceSum>limit) motor_low[ID].velocityDifferenceSum = limit;
	if(motor_low[ID].velocityDifferenceSum<(0-limit)) motor_low[ID].velocityDifferenceSum = (0-limit);

	motor_low[ID].sendI = 50*motor_low[ID].Kp*motor_low[ID].velocityDifference + 19*motor_low[ID].Ki/10*motor_low[ID].velocityDifferenceSum;
}

void controlP_calSendI_PPI_high(int ID){
	int limit = 16667;

	motor_high[ID].positionDifference = motor_high[ID].targetPosition - motor_high[ID].position;
	if(motor_high[ID].positionDifference > 4000) motor_high[ID].positionDifference = 4000;
	if(motor_high[ID].positionDifference < -4000) motor_high[ID].positionDifference = -4000;

	motor_high[ID].targetVelocity = motor_high[ID].K/100*(motor_high[ID].positionDifference);
	motor_high[ID].velocityDifference = motor_high[ID].targetVelocity - motor_high[ID].velocity;
	motor_high[ID].velocityDifferenceSum = motor_high[ID].velocityDifference + motor_high[ID].velocityDifferenceSum;

	if(motor_high[ID].velocityDifferenceSum>limit) motor_high[ID].velocityDifferenceSum = limit;
	if(motor_high[ID].velocityDifferenceSum<(0-limit)) motor_high[ID].velocityDifferenceSum = (0-limit);

	motor_high[ID].sendI = 50*motor_high[ID].Kp*motor_high[ID].velocityDifference + 19*motor_high[ID].Ki/10*motor_high[ID].velocityDifferenceSum;
}

void control_encoder_speed_low(int ID){
	int limit = 16667;
	motor_low[ID].encoder_difference = motor_low[ID].target_leg - motor_low[ID].leg_angle;
	if(motor_low[ID].encoder_difference > 360) motor_low[ID].encoder_difference = 300;
	if(motor_low[ID].encoder_difference < -300) motor_low[ID].encoder_difference = -300;
	//motor_low[ID].angleDifferenceSum = motor_low[ID].angleDifference + motor_low[ID].angleDifferenceSum;

	motor_low[ID].targetVelocity = motor_low[ID].K_encoder*(motor_low[ID].encoder_difference);
	motor_low[ID].velocityDifference = motor_low[ID].targetVelocity - motor_low[ID].velocity;
	//motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;
    motor_low[ID].velocity_diff = motor_low[ID].velocityDifference - motor_low[ID].velocityDifference_last;
    
    if(motor_low[ID].velocity_diff>500) motor_low[ID].velocity_diff = 500;
	if(motor_low[ID].velocity_diff<-500) motor_low[ID].velocity_diff = -500;


	// if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
	// if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

	motor_low[ID].sendI = motor_low[ID].Kp_encoder*motor_low[ID].velocityDifference + 19*motor_low[ID].Kd_encoder/10*motor_low[ID].velocity_diff;
    motor_low[ID].velocityDifference_last = motor_low[ID].velocityDifference;
}

void control_encoder_speed_high(int ID){
	int limit = 16667;
	motor_high[ID].encoder_difference = motor_high[ID].target_leg - motor_high[ID].leg_angle;
	if(motor_high[ID].encoder_difference > 360) motor_high[ID].encoder_difference = 300;
	if(motor_high[ID].encoder_difference < -300) motor_high[ID].encoder_difference = -300;
	//motor_low[ID].angleDifferenceSum = motor_low[ID].angleDifference + motor_low[ID].angleDifferenceSum;

	motor_high[ID].targetVelocity = motor_high[ID].K_encoder*(motor_high[ID].encoder_difference);
	motor_high[ID].velocityDifference = motor_high[ID].targetVelocity - motor_high[ID].velocity;
	//motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;
    motor_high[ID].velocity_diff = motor_high[ID].velocityDifference - motor_high[ID].velocityDifference_last;
    
    if(motor_high[ID].velocity_diff>500) motor_high[ID].velocity_diff = 500;
	if(motor_high[ID].velocity_diff<-500) motor_high[ID].velocity_diff = -500;


	// if(motor[ID].velocityDifferenceSum>limit) motor[ID].velocityDifferenceSum = limit;
	// if(motor[ID].velocityDifferenceSum<(0-limit)) motor[ID].velocityDifferenceSum = (0-limit);

	motor_high[ID].sendI = motor_high[ID].Kp_encoder*motor_high[ID].velocityDifference + 19*motor_high[ID].Kd_encoder/10*motor_high[ID].velocity_diff;
    motor_high[ID].velocityDifference_last = motor_high[ID].velocityDifference;
}

void control_encoder_speed_leg_low(int ID){
	int limit = 16667;

	

	motor_low[ID].encoder_difference_leg = motor_low[ID].target_leg - motor_low[ID].leg_angle;
	// if(motor_low[ID].encoder_difference_leg > 360) motor_low[ID].encoder_difference_leg = 360;
	// if(motor_low[ID].encoder_difference_leg < -360) motor_low[ID].encoder_difference_leg = -360;
	if(fabs(motor_low[ID].encoder_difference_leg)>360-fabs(motor_low[ID].encoder_difference_leg)){
		if(motor_low[ID].encoder_difference_leg>0){
			motor_low[ID].encoder_difference_leg = fabs(motor_low[ID].encoder_difference_leg)-360;
		}
		else{
			motor_low[ID].encoder_difference_leg = 360 - fabs(motor_low[ID].encoder_difference_leg);
		}
	} 
	else if(fabs(motor_low[ID].encoder_difference_leg)<360-fabs(motor_low[ID].encoder_difference_leg)){
		motor_low[ID].encoder_difference_leg = motor_low[ID].encoder_difference_leg;
	}
	else if(fabs(motor_low[ID].encoder_difference_leg)==360-fabs(motor_low[ID].encoder_difference_leg)){
		motor_low[ID].encoder_difference_leg = fabs(motor_low[ID].encoder_difference_leg);
	}
	
	

	motor_low[ID].targetVelocity = motor_low[ID].K_encoder*(motor_low[ID].encoder_difference_leg);
	motor_low[ID].velocityDifference = motor_low[ID].targetVelocity - motor_low[ID].velocity;

    motor_low[ID].velocity_diff = motor_low[ID].velocityDifference - motor_low[ID].velocityDifference_last;
    
    if(motor_low[ID].velocity_diff>500) motor_low[ID].velocity_diff = 500;
	if(motor_low[ID].velocity_diff<-500) motor_low[ID].velocity_diff = -500;

	motor_low[ID].sendI = motor_low[ID].Kp_encoder*motor_low[ID].velocityDifference + 19*motor_low[ID].Kd_encoder/10*motor_low[ID].velocity_diff;
    motor_low[ID].velocityDifference_last = motor_low[ID].velocityDifference;
}

void control_encoder_speed_leg_high(int ID){
	int limit = 16667;

	motor_high[ID].encoder_difference_leg = motor_high[ID].target_leg - motor_high[ID].leg_angle;
	// if(motor_high[ID].encoder_difference_leg > 360) motor_high[ID].encoder_difference_leg = 360;
	// if(motor_high[ID].encoder_difference_leg < -360) motor_high[ID].encoder_difference_leg = -360;
	if(fabs(motor_high[ID].encoder_difference_leg)>360-fabs(motor_high[ID].encoder_difference_leg)){
		if(motor_high[ID].encoder_difference_leg>0){
			motor_high[ID].encoder_difference_leg = fabs(motor_high[ID].encoder_difference_leg)-360;
		}
		else{
			motor_high[ID].encoder_difference_leg = 360 - fabs(motor_high[ID].encoder_difference_leg);
		}
	} 
	else if(fabs(motor_high[ID].encoder_difference_leg)<360-fabs(motor_high[ID].encoder_difference_leg)){
		motor_high[ID].encoder_difference_leg = motor_high[ID].encoder_difference_leg;
	}
	else if(fabs(motor_high[ID].encoder_difference_leg)==360-fabs(motor_high[ID].encoder_difference_leg)){
		motor_high[ID].encoder_difference_leg = fabs(motor_high[ID].encoder_difference_leg);
	}
	

	motor_high[ID].targetVelocity = motor_high[ID].K_encoder*(motor_high[ID].encoder_difference_leg);
	motor_high[ID].velocityDifference = motor_high[ID].targetVelocity - motor_high[ID].velocity;

    motor_high[ID].velocity_diff = motor_high[ID].velocityDifference - motor_high[ID].velocityDifference_last;
    
    if(motor_high[ID].velocity_diff>500) motor_high[ID].velocity_diff = 500;
	if(motor_high[ID].velocity_diff<-500) motor_high[ID].velocity_diff = -500;

	motor_high[ID].sendI = motor_high[ID].Kp_encoder*motor_high[ID].velocityDifference + 19*motor_high[ID].Kd_encoder/10*motor_high[ID].velocity_diff;
    motor_high[ID].velocityDifference_last = motor_high[ID].velocityDifference;
}


void txThread_low(int s)
{
    struct can_frame frame_low;
    
	frame_low.can_id = 0x200;
    
	frame_low.can_dlc = 8;
    
	int j;
	sendIMessage_low.data.resize(4);
    
  
    for (j = 0; j < 4; j++)
	{
		motor_low[j].sendI = 0;
		frame_low.data[2*j] = motor_low[j].sendI << 8;
		frame_low.data[2*j+1] = motor_low[j].sendI << 0;
    
	}

	int nbytes_low;

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
            control_encoder_speed_leg_low(0);
            control_encoder_speed_leg_low(1);
            control_encoder_speed_leg_low(2);
            control_encoder_speed_leg_low(3);
			
		}
		
		// for (j = 0;j<4;j++)
		// {
		// 	if(judge_forward==1 || judge_backward==1 || judge_left==1 || judge_right==1){
        //         if (motor_low[j].sendI >15000) {
		// 		    motor_low[j].sendI = 15000;
		// 	    }
                
		// 	    if (motor_low[j].sendI <-15000) {
		// 		    motor_low[j].sendI = -15000;
		// 	    }
                
		// 	    frame_low.data[2*j] = motor_low[j].sendI>>8;
		// 	    frame_low.data[2*j+1] = motor_low[j].sendI>>0;
    
		// 	    sendIMessage_low.data[j] = motor_low[j].sendI;
                
                
                
        //     }
        //     else{
        //         frame_low.data[2*j] = 0;
		// 	    frame_low.data[2*j+1] = 0;
               
		// 	    sendIMessage_low.data[j] = 0;
        //     }
		// }
		for (j = 0;j<4;j++)
		{
			if (motor_low[j].sendI >15000) {
				motor_low[j].sendI = 15000;
			}
                
			if (motor_low[j].sendI <-15000) {
				motor_low[j].sendI = -15000;
			}
                
			frame_low.data[2*j] = motor_low[j].sendI>>8;
			frame_low.data[2*j+1] = motor_low[j].sendI>>0;
    
			sendIMessage_low.data[j] = motor_low[j].sendI;
		}
		
    
		sendIPub_low.publish(sendIMessage_low);
        // sendIPub.publish(sendIMessage_high);
        
        nbytes_low = write(s, &frame_low, sizeof(struct can_frame));
        
        if (nbytes_low == -1 ) {
			printf("low_send error\n");
        }
        
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }

}
void txThread_high(int s)
{
    struct can_frame frame_high;
    
	frame_high.can_id = 0x200;
    
	frame_high.can_dlc = 8;
    
	int j;
	sendIMessage_high.data.resize(4);
    
    
	for(j=0; j<4; j++){
        motor_high[j].sendI = 0;
    }
    for (j = 0; j < 4; j++)
	{
		frame_high.data[2*j] = motor_high[j].sendI << 8;
		frame_high.data[2*j+1] = motor_high[j].sendI << 0;
        

	}

	int nbytes_high;

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
            control_encoder_speed_leg_high(0);
            control_encoder_speed_leg_high(1);
            control_encoder_speed_leg_high(2);
            control_encoder_speed_leg_high(3);
		}
		
		// for (j = 0;j<4;j++)
		// {
		// 	if(judge_forward==1 || judge_backward || judge_left==1 || judge_right==1){
        //         if (motor_high[j].sendI >15000) {
		// 		    motor_high[j].sendI = 15000;
		// 	    }
                
		// 	    if (motor_high[j].sendI <-15000) {
		// 		    motor_high[j].sendI = -15000;
		// 	    }
                
		// 	    frame_high.data[2*j] = motor_high[j].sendI>>8;
		// 	    frame_high.data[2*j+1] = motor_high[j].sendI>>0;
    
		// 	    sendIMessage_high.data[j] = motor_high[j].sendI;
                
                
                
        //     }
        //     else{
        //         frame_high.data[2*j] = 0;
		// 	    frame_high.data[2*j+1] = 0;
               
		// 	    sendIMessage_high.data[j] = 0;
                
                
                
        //     }
            

		// }
		for (j = 0;j<4;j++)
		{
			if (motor_high[j].sendI >15000) {
				motor_high[j].sendI = 15000;
			}
                
			if (motor_high[j].sendI <-15000) {
				motor_high[j].sendI = -15000;
			}
                
			frame_high.data[2*j] = motor_high[j].sendI>>8;
			frame_high.data[2*j+1] = motor_high[j].sendI>>0;
    
			sendIMessage_high.data[j] = motor_high[j].sendI;
		}
    
		
        sendIPub_high.publish(sendIMessage_high);
        
        nbytes_high = write(s, &frame_high, sizeof(struct can_frame));
        
        if (nbytes_high == -1 ) {
			printf("high_send error\n");
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

	ofstream fout("speed.txt");
	fout<<"low[0]\tlow[1]\tlow[2]\tlow[3]\thigh[1]\thigh[2]\thigh[3]\thigh[4]\n\r";

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
		motor_low[i].Kp = 13;
		motor_low[i].Ki = 6;
		motor_low[i].K = 15;
        motor_low[i].K_encoder = 10;
        motor_low[i].target_encoder = 175;
		motor_low[i].targetPosition = 0;
		//motor_low[i].ori_encoder = 175;
        motor_low[i].Kp_encoder = 22;
        motor_low[i].Ki_encoder = 5;
        motor_low[i].Kd_encoder = 5;
		//motor_low[i].target_leg = 0;
        
		motor_high[i].Kp = 13;
		motor_high[i].Ki = 6;
		motor_high[i].K = 15;
        motor_high[i].K_encoder = 10;
        motor_high[i].target_encoder = 175;
		motor_high[i].targetPosition = 0;
		//motor_high[i].ori_encoder = 175;
        motor_high[i].Kp_encoder = 22;
        motor_high[i].Ki_encoder = 5;
        motor_high[i].Kd_encoder = 5;
		//motor_high[i].target_leg = 0;

	}
	// motor_low[0].target_leg = 0;
	// motor_low[1].target_leg = 0;
	// motor_low[2].target_leg = 0;
	// motor_low[3].target_leg = 0;
	// motor_high[0].target_leg = 0;
	// motor_high[1].target_leg = 0;
	// motor_high[2].target_leg = 0;
	// motor_high[3].target_leg = 0;

	motor_low[0].ori_encoder = 29;
	motor_low[1].ori_encoder = 29;
	motor_low[2].ori_encoder = 216;
	motor_low[3].ori_encoder = 216;
	motor_high[0].ori_encoder = 76;
	motor_high[1].ori_encoder = 76;
	motor_high[2].ori_encoder = 37;
	motor_high[3].ori_encoder = 37;


    positionPub_low = n.advertise<std_msgs::Int32MultiArray>("position_low",100);
    velocityPub_low = n.advertise<std_msgs::Int32MultiArray>("velocity_low",100);
    IPub_low = n.advertise<std_msgs::Int32MultiArray>("I_low",100);
	sendIPub_low = n.advertise<std_msgs::Int32MultiArray>("sendI_low",100);
	targetPositionPub_low = n.advertise<std_msgs::Int32MultiArray>("targetPositionFromPosition_low",100);
    positionPub_high = n.advertise<std_msgs::Int32MultiArray>("position_high",100);
    velocityPub_high = n.advertise<std_msgs::Int32MultiArray>("velocity_high",100);
    IPub_high = n.advertise<std_msgs::Int32MultiArray>("I_high",100);
	sendIPub_high = n.advertise<std_msgs::Int32MultiArray>("sendI_high",100);
	targetPositionPub_high = n.advertise<std_msgs::Int32MultiArray>("targetPositionFromPosition_high",100);


	int s_low;
	int s_high;
	struct sockaddr_can addr_low;
    struct sockaddr_can addr_high;
	struct ifreq ifr_low;
    struct ifreq ifr_high;

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

    if ((s_high = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr_high.ifr_name, "can1");
	ioctl(s_high, SIOCGIFINDEX, &ifr_high);

	addr_high.can_family = AF_CAN;
	addr_high.can_ifindex = ifr_high.ifr_ifindex;

	printf("%s at index %d\n", ifr_high.ifr_name, ifr_high.ifr_ifindex);

	if (bind(s_high, (struct sockaddr *)&addr_high, sizeof(addr_high)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	std::thread canRx_low(rxThread_low, s_low);
	
    std::thread canRX_high(rxThread_high, s_high);
	sleep(0.5);
	//std::thread canTx_low(txThread_low, s_low);
	
    //std::thread canTx_high(txThread_high, s_high);
    PISub_low = n.subscribe("PI", 10, PI_Callback_low);
	PSub_low = n.subscribe("P",10,P_Callback_low);
	targetVelocitySub_low = n.subscribe("targetVelocity", 10, targetVelocity_Callback_low);
	targetPositionSub_low = n.subscribe("targetPosition", 10, targetPosition_Callback_low);
    PISub_high = n.subscribe("PI", 10, PI_Callback_high);
	PSub_high = n.subscribe("P",10,P_Callback_high);
	targetVelocitySub_high = n.subscribe("targetVelocity", 10, targetVelocity_Callback_high);
	targetPositionSub_high = n.subscribe("targetPosition", 10, targetPosition_Callback_high);
    //encoder_sub = n.subscribe<std_msgs::Float32MultiArray>("SingleAngle",10,boost::bind(&encoder_callback, _1, _2));
	//encoder_turn_sub = n.subscribe("MultiTurn",10,encoder_turn_callback);
	//encoder_angle_sub = n.subscribe("MultiAngle",10,encoder_angle_callback);
	//encoder_dir = n.subscribe("Dir",10,dir_callback);
	encoder_angle_sum = n.subscribe("MultiAngleSum",10,encoder_angle_sum_callback);
	joy_sub = n.subscribe<sensor_msgs::Joy>("joy",10,buttonCallback);
	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
