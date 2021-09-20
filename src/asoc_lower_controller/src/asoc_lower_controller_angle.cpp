#include "ros/ros.h"

#include <unistd.h>

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>


#include <linux/can.h>
#include <linux/can/raw.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define PI 3.14159265

using namespace std;
using namespace Eigen;

int flag;

float frame_vt_max = 50;
float frame_vt_min = -50;
float frame_vn_max = 30;
float frame_vn_min = -30;
float frame_w_max = 300; // 100 = 320 degree/s
float frame_w_min = -300;

int rxCounter_low;
int rxCounter_high;
float power, forward_s;
float power_last;
int on_off = 1;
int Stop_flag = 0;

int judge_forward=0;          //high[1],high[2] angle 1
int judge_backward=0;         //high[3],high[4]   angle 0
int judge_left=0;             //low[1],low[2]    angle 2
int judge_right=0;            //low[3],low[4]   angle 3
int co =0;
int judge_angle1 = 0;
int judge_angle2 = 0;
float stick_forward = 0.0;
float stick_right = 0.0;
float stick_yaw = 0.0;
float L = 2;   //  ratio of offset/split
float r = 0.09; //radii of the wheel
// float D = 0.32; //length of the virtual rod linking the pivot rod and the center of ASOC
float linkTheta[4]; // the angle between four-parallel link and horizontal ground, variable, so need to be passed by Encoder
float D[4]={0.178,0.178,0.178,0.178};
//D = 0.178  + 0.14 * sin(linkTheta); //length of the virtual rod linking the pivot rod and the center of ASOC

float frame_vt;
float frame_vn;
float frame_w ;

Matrix<float,2,1> wheel_low0;                        //vector representation of the speed of wheels
Matrix<float,2,1> wheel_low2;
Matrix<float,2,1> wheel_high0;
Matrix<float,2,1> wheel_high2;

Matrix<float,2,1> vc_low0;                  //vector representation of the speed of pivots
Matrix<float,2,1> vc_low2;
Matrix<float,2,1> vc_high0;
Matrix<float,2,1> vc_high2;

ros::Publisher velocityPub_low;
ros::Publisher velocityPub_high;
ros::Publisher IPub_low;
ros::Publisher IPub_high;
ros::Publisher sendIPub_low;
ros::Publisher sendIPub_high;
ros::Publisher leg_angle_Pub_high;
ros::Publisher leg_angle_Pub_low;
//ros::Publisher leg_angle_sum_Pub_low;
//ros::Publisher leg_angle_sum_Pub_high;

ros::Subscriber joy_sub;
ros::Subscriber encoder_angle_sum;
ros::Subscriber upper_controller;
ros::Subscriber  Tmotor_angle;

std_msgs::Int32MultiArray velocityMessage_low,velocityMessage_high,IMessage_low,IMessage_high,sendIMessage_low,sendIMessage_high;
//std_msgs::Float32MultiArray leg_angle_Message_low, leg_angle_Message_high, leg_angle_sum_Message_low, leg_angle_sum_Message_high;
geometry_msgs::PolygonStamped leg_angle_Message_low, leg_angle_Message_high;

struct Motor{
    int rows = 2;
    int cols = 2;

	int16_t angle,velocity,I;
	uint8_t temperature;
	int16_t realAngle;
    int32_t position;
    int16_t NumOfTurns;

	int32_t targetPosition;
	float targetVelocity;

	int16_t angleDifference;
	int16_t angleLast;

	int16_t sendI;

	int32_t positionDifference;
	int32_t positionDifferenceSum;

	int16_t velocityDifference;
	int16_t velocityDifference_last;
	int32_t velocityDifferenceSum;
	int32_t velocity_diff;    //the difference between the latest velocity difference and the last velocity difference.

	float Ki;
	float Kp;
	float Kd;

	float K;

    float K_encoder;
    float Kp_encoder;
    float Ki_encoder;
    float Kd_encoder;
	float encoder_angle;         //real-time angles from the encoder
	float encoder_angle_with_turn;     //real-time angle = turns*360+encoder_angle

	float target_leg;        //the angle of the leg when it is in target position
	
	float ori_encoder;         //the angle of the encoder when it is turned on and the wheels are set to the forward position
	float encoder_difference_leg;         
	float leg_angle;              //angle of the leg. leg_angle shall be zero when the wheels are set to the forward position, and the maximum shall be 360.
	float leg_angle_with_turn;                      

    float vct;
    float vcn;

    Matrix<float, 2,2> C;     
};
Motor motor_low[4];
Motor motor_high[4];


void printfMultiMotorVelocity_low(void){
	ROS_INFO("velocity is %d,%d,%d,%d;\r\n",motor_low[0].velocity,motor_low[1].velocity,motor_low[2].velocity,motor_low[3].velocity);
}


void printfMultiMotorVelocity_high(void){
	ROS_INFO("velocity is %d,%d,%d,%d;\r\n",motor_high[0].velocity,motor_high[1].velocity,motor_high[2].velocity,motor_high[3].velocity);
}


void printfMultiMotorI_low(void){
	ROS_INFO("I is %d,%d,%d,%d;\r\n",motor_low[0].I,motor_low[1].I,motor_low[2].I,motor_low[3].I);
}


void printfMultiMotorI_high(void){
	ROS_INFO("I is %d,%d,%d,%d;\r\n",motor_high[0].I,motor_high[1].I,motor_high[2].I,motor_high[3].I);
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

void Tmotor_angle_callback(geometry_msgs::PolygonStamped MultiTmotorAngle){
	for(int i = 0; i < 4; i++){
		//D[0,1,2,3] responds to T-motor[0,1,2,3], T-motor[0,1,2,3] responds to MultiTmotor[0,1,2,3]
        //Pass the angle of Tmotor[i] to linkTheta[i]
		//degree = 15.52 * MultiTmotorAngle + 73
			linkTheta[i] = (15.52 * MultiTmotorAngle.polygon.points[i].x + 73 ) * PI / 180;
			D[i] = 0.178 + 0.14 * sin(linkTheta[i]);
		}
		//D[0,1,2,3] responds to T-motor[0,1,2,3]
	   //D[0] responds to vc_low0(chaiss X axis positive), 
	   //D[1] responds to vc_high0(chassis Y axis positive), 
	   //D[2] responds to vc_low2(chassis X axis negative),
	   //D[3] responds to vc_high2(chassis Y axis negative)
	   
}
void buttonCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //judge_forward = joy->buttons[3];
	// judge_backward = joy->buttons[0];
	// judge_left = joy->buttons[2];
	// judge_right = joy->buttons[1];
    stick_forward = joy->axes[1];
    stick_right = joy->axes[0];
	stick_yaw = joy->axes[3];
    power = joy -> buttons[8];
	forward_s = joy->buttons[5];
    if(power>power_last){
        on_off = -on_off;
    }
    power_last = power;  

	// frame_vt = 30*stick_forward;
	frame_vt = 15*stick_forward + 19 * forward_s;
	frame_vn = -15*stick_right;
	frame_w =  -60*stick_yaw;
}


void upper_controller_callback(geometry_msgs::Twist cmd_vel){
	if(cmd_vel.linear.x >= frame_vt_min && cmd_vel.linear.x <= frame_vt_max){
		frame_vt = cmd_vel.linear.x;
	}
	else if(cmd_vel.linear.x < frame_vt_min){
		frame_vt = frame_vt_min;
	}
	else{
		frame_vt = frame_vt_max;
	}

	if(cmd_vel.linear.y >= frame_vn_min && cmd_vel.linear.y <= frame_vn_max){
		frame_vn = cmd_vel.linear.y;
	}
	else if(cmd_vel.linear.y < frame_vn_min){
		frame_vn = frame_vn_min;
	}
	else{
		frame_vn = frame_vn_max;
	}

	if(cmd_vel.angular.z >= frame_w_min && cmd_vel.angular.z <= frame_w_max){
		frame_w = cmd_vel.angular.z;
	}
	else if(cmd_vel.angular.z < frame_w_min){
		frame_w = frame_w_min;
	}
	else{
		frame_w = frame_w_max;
	}
}


void controlV_calSendI_PI_low(int ID){
	int limit = 16667;
	motor_low[ID].velocityDifference = fmin(fmax(
		motor_low[ID].targetVelocity - motor_low[ID].velocity, -1000), 1000); // -1000 < d_vel < 1000

	motor_low[ID].velocityDifferenceSum = fmin(fmax(
		motor_low[ID].velocityDifference + motor_low[ID].velocityDifferenceSum, -limit), limit); // -limit < Sum < limit 

	motor_low[ID].sendI = motor_low[ID].Kp*motor_low[ID].velocityDifference + motor_low[ID].Ki/10*motor_low[ID].velocityDifferenceSum;
}


void controlV_calSendI_PI_high(int ID){
	int limit = 16667;
	motor_high[ID].velocityDifference = fmin(fmax(
		motor_high[ID].targetVelocity - motor_high[ID].velocity, -1000), 1000); // -1000 < d_vel < 1000

	motor_high[ID].velocityDifferenceSum = fmin(fmax(
		motor_high[ID].velocityDifference + motor_high[ID].velocityDifferenceSum, -limit), limit); // -limit < Sum < limit 

	motor_high[ID].sendI = motor_high[ID].Kp*motor_high[ID].velocityDifference + motor_high[ID].Ki/10*motor_high[ID].velocityDifferenceSum;
}


void body_to_wheel(float vt, float vn, float w){
    for(int i=0; i<4; i++){
        motor_low[i].vct = vt;
        motor_high[i].vcn = vn;
    }
//Adjust the length D according to the change theta
// Corresponding each length to the ASOC module, e.x, D[0] is the distance of vc_low0.

	//D[0,1,2,3] responds to T-motor[0,1,2,3]
	   //D[0] responds to vc_low0(chaiss X axis positive), 
	   //D[1] responds to vc_high0(chassis Y axis positive), 
	   //D[2] responds to vc_low2(chassis X axis negative),
	   //D[3] responds to vc_high2(chassis Y axis negative)
	   
    vc_low0(1,0) = motor_low[0].vcn = motor_low[1].vcn = vn + w * D[0];
    vc_low0(0,0) = motor_low[0].vct;
    vc_low2(1,0) = motor_low[2].vcn = motor_low[3].vcn = vn - w * D[2];
    vc_low2(0,0) = motor_low[2].vct;
    vc_high0(1,0) = motor_high[0].vcn;
    vc_high0(0,0) = motor_high[0].vct = motor_high[1].vct = vt + w * D[1];
    vc_high2(1,0) = motor_high[2].vcn;
    vc_high2(0,0) = motor_high[2].vct = motor_high[3].vct = vt - w * D[3];

	motor_low[0].C(0,0) = motor_low[1].C(0,0) = (cos(-motor_low[0].leg_angle*PI/180) + 0.5*L*sin(-motor_low[0].leg_angle*PI/180))/2;
    motor_low[0].C(0,1) = motor_low[1].C(0,1) = (-sin(-motor_low[0].leg_angle*PI/180) + 0.5*L*cos(-motor_low[0].leg_angle*PI/180))/2;
    motor_low[0].C(1,0) = motor_low[1].C(1,0) = (cos(-motor_low[0].leg_angle*PI/180) - 0.5*L*sin(-motor_low[0].leg_angle*PI/180))/2;
    motor_low[0].C(1,1) = motor_low[1].C(1,1) = (-sin(-motor_low[0].leg_angle*PI/180) - 0.5*L*cos(-motor_low[0].leg_angle*PI/180))/2;
    
    motor_low[2].C(0,0) = motor_low[3].C(0,0) = (cos(-motor_low[2].leg_angle*PI/180) + 0.5*L*sin(-motor_low[2].leg_angle*PI/180))/2;
    motor_low[2].C(0,1) = motor_low[3].C(0,1) = (-sin(-motor_low[2].leg_angle*PI/180) + 0.5*L*cos(-motor_low[2].leg_angle*PI/180))/2;
    motor_low[2].C(1,0) = motor_low[3].C(1,0) = (cos(-motor_low[2].leg_angle*PI/180) - 0.5*L*sin(-motor_low[2].leg_angle*PI/180))/2;
    motor_low[2].C(1,1) = motor_low[3].C(1,1) = (-sin(-motor_low[2].leg_angle*PI/180) - 0.5*L*cos(-motor_low[2].leg_angle*PI/180))/2;

    motor_high[0].C(0,0) = motor_high[1].C(0,0) = (cos(-motor_high[0].leg_angle*PI/180) + 0.5*L*sin(-motor_high[0].leg_angle*PI/180))/2;
    motor_high[0].C(0,1) = motor_high[1].C(0,1) = (-sin(-motor_high[0].leg_angle*PI/180) + 0.5*L*cos(-motor_high[0].leg_angle*PI/180))/2;
    motor_high[0].C(1,0) = motor_high[1].C(1,0) = (cos(-motor_high[0].leg_angle*PI/180) - 0.5*L*sin(-motor_high[0].leg_angle*PI/180))/2;
    motor_high[0].C(1,1) = motor_high[1].C(1,1) = (-sin(-motor_high[0].leg_angle*PI/180) - 0.5*L*cos(-motor_high[0].leg_angle*PI/180))/2;

    motor_high[2].C(0,0) = motor_high[3].C(0,0) = (cos(-motor_high[2].leg_angle*PI/180) + 0.5*L*sin(-motor_high[2].leg_angle*PI/180))/2;
    motor_high[2].C(0,1) = motor_high[3].C(0,1) = (-sin(-motor_high[2].leg_angle*PI/180) + 0.5*L*cos(-motor_high[2].leg_angle*PI/180))/2;
    motor_high[2].C(1,0) = motor_high[3].C(1,0) = (cos(-motor_high[2].leg_angle*PI/180) - 0.5*L*sin(-motor_high[2].leg_angle*PI/180))/2;
    motor_high[2].C(1,1) = motor_high[3].C(1,1) = (-sin(-motor_high[2].leg_angle*PI/180) - 0.5*L*cos(-motor_high[2].leg_angle*PI/180))/2;

	wheel_low0 = motor_low[0].C*vc_low0/r;
    wheel_low2 = motor_low[2].C*vc_low2/r;
    wheel_high0 = motor_high[0].C*vc_high0/r;
    wheel_high2 = motor_high[2].C*vc_high2/r;

    motor_low[0].targetVelocity = wheel_low0(0,0)*19;
    motor_low[1].targetVelocity = -wheel_low0(1,0)*19;
    motor_low[2].targetVelocity = wheel_low2(0,0)*19;
    motor_low[3].targetVelocity = -wheel_low2(1,0)*19;

    motor_high[0].targetVelocity = wheel_high0(0,0)*19;
    motor_high[1].targetVelocity = -wheel_high0(1,0)*19;
    motor_high[2].targetVelocity = wheel_high2(0,0)*19;
    motor_high[3].targetVelocity = -wheel_high2(1,0)*19;
}

// A thread to deal with Ctrl-C command
void signalCallback(int signum)
{	
	double startt,endt;
	startt = clock();
	Stop_flag = 1;
	ros::Duration(1.0).sleep();
	endt = clock();
	ROS_INFO("shutdown!!!!!!!");
	exit(1);
}

void rxThread_low(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame_low;
	int nbytes_low;

    // velocityMessage_low.data.resize(4);
    velocityMessage_low.data.resize(5);
	IMessage_low.data.resize(4);

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

        velocityMessage_low.data[ID] = motor_low[ID].velocity;
		velocityMessage_low.data[4] = frame_vt;
		IMessage_low.data[ID] = motor_low[ID].I;

		if(i%4==0)
		{
			printfMultiMotorVelocity_low();
			printfMultiMotorI_low();
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


void rxThread_high(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame_high;
	int nbytes_high;

    velocityMessage_high.data.resize(4);
	IMessage_high.data.resize(4);

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

        velocityMessage_high.data[ID] = motor_high[ID].velocity;
		IMessage_high.data[ID] = motor_high[ID].I;

        if(i%4==0)
		{
			velocityPub_high.publish(velocityMessage_high);
			IPub_high.publish(IMessage_high);
		}
        if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				motor_high[j].targetPosition = motor_high[j].position;
			}
			
		}

		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}


void txThread_low(int s)
{
    struct can_frame frame_low;
    
	frame_low.can_id = 0x200;
    
	frame_low.can_dlc = 8;
    
	int j;

    sendIMessage_low.data.resize(4);
    //leg_angle_Message_low.data.resize(4);
	//leg_angle_sum_Message_low.data.resize(4);
	leg_angle_Message_low.polygon.points.resize(4);
  
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
			controlV_calSendI_PI_low(0);
			controlV_calSendI_PI_low(1);
			controlV_calSendI_PI_low(2);
			controlV_calSendI_PI_low(3);
		}
		
		for (j = 0;j<4;j++)
		{
			//if(judge_forward==1 || judge_backward==1 || judge_left==1 || judge_right==1 || stick_forward != 0 || stick_right != 0 || stick_yaw != 0){
            if(on_off > 0 && Stop_flag ==0){    
                if (motor_low[j].sendI >15000) {
				    motor_low[j].sendI = 15000;
			    }
                
			    if (motor_low[j].sendI <-15000) {
				    motor_low[j].sendI = -15000;
			    }
			    frame_low.data[2*j] = motor_low[j].sendI>>8;
			    frame_low.data[2*j+1] = motor_low[j].sendI>>0;

                sendIMessage_low.data[j] = motor_low[j].sendI;
                //leg_angle_Message_low.data[j] = motor_low[j].leg_angle;
				//leg_angle_sum_Message_low.data[j] = motor_low[j].leg_angle_with_turn;
				leg_angle_Message_low.polygon.points[j].x=motor_low[j].leg_angle;
				leg_angle_Message_low.polygon.points[j].y=motor_low[j].leg_angle_with_turn;
				//x表示leg_angle y表示leg_angle_sum
            }
            else{
                frame_low.data[2*j] = 0;
			    frame_low.data[2*j+1] = 0;

                sendIMessage_low.data[j] = 0;
                // leg_angle_Message_low.data[j] = motor_low[j].leg_angle;
				// leg_angle_sum_Message_low.data[j] = motor_low[j].leg_angle_with_turn;
				leg_angle_Message_low.polygon.points[j].x=motor_low[j].leg_angle;
				leg_angle_Message_low.polygon.points[j].y=motor_low[j].leg_angle_with_turn;
            }
		}
        
		leg_angle_Message_low.header.stamp=ros::Time::now();
        sendIPub_low.publish(sendIMessage_low);
        leg_angle_Pub_low.publish(leg_angle_Message_low);
		//leg_angle_sum_Pub_low.publish(leg_angle_sum_Message_low);
        
        nbytes_low = write(s, &frame_low, sizeof(struct can_frame));
        
        if (nbytes_low == -1 ) {
			printf("send error\n");
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
   // leg_angle_Message_high.data.resize(4);
	//leg_angle_sum_Message_high.data.resize(4);
	leg_angle_Message_high.polygon.points.resize(4);
    
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
		
        body_to_wheel(frame_vt,frame_vn,frame_w);
		
		if (flag ==1)
		{
			controlV_calSendI_PI_high(0);
			controlV_calSendI_PI_high(1);
			controlV_calSendI_PI_high(2);
			controlV_calSendI_PI_high(3);	
		}
		
		for (j = 0;j<4;j++)
		{
			//if(judge_forward==1 || judge_backward==1 || judge_left==1 || judge_right==1 || stick_forward != 0 || stick_right != 0 || stick_yaw != 0){
            if(on_off > 0 && Stop_flag==0){
                if (motor_high[j].sendI >15000) {
				    motor_high[j].sendI = 15000;
			    }
                
			    if (motor_high[j].sendI <-15000) {
				    motor_high[j].sendI = -15000;
			    }
                
			    frame_high.data[2*j] = motor_high[j].sendI>>8;
			    frame_high.data[2*j+1] = motor_high[j].sendI>>0;

                sendIMessage_high.data[j] = motor_high[j].sendI;
               // leg_angle_Message_high.data[j] = motor_high[j].leg_angle;
			   //leg_angle_sum_Message_high.data[j] = motor_high[j].leg_angle_with_turn;
			   leg_angle_Message_high.polygon.points[j].x=motor_high[j].leg_angle;
			   leg_angle_Message_high.polygon.points[j].y=motor_high[j].leg_angle_with_turn;
            }
            else{
                frame_high.data[2*j] = 0;
			    frame_high.data[2*j+1] = 0;

                sendIMessage_high.data[j] = 0;
                // leg_angle_Message_high.data[j] = motor_high[j].leg_angle;
				//leg_angle_sum_Message_high.data[j] = motor_high[j].leg_angle_with_turn;
				leg_angle_Message_high.polygon.points[j].x=motor_high[j].leg_angle;
			    leg_angle_Message_high.polygon.points[j].y=motor_high[j].leg_angle_with_turn;
            }
		}
		
		leg_angle_Message_high.header.stamp=ros::Time::now();
        sendIPub_high.publish(sendIMessage_high);
        leg_angle_Pub_high.publish(leg_angle_Message_high);
		//leg_angle_sum_Pub_high.publish(leg_angle_sum_Message_high);
        
        nbytes_high = write(s, &frame_high, sizeof(struct can_frame));
        
        if (nbytes_high == -1 ) {
			printf("send error\n");
        }
        
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }

}


int main(int argc, char** argv) {
	flag = 0;

	ros::init(argc,argv,"asoc_lower_controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	signal(SIGINT, signalCallback);

    ofstream fout("speed.txt");
	fout<<"low[0]\tlow[1]\tlow[2]\tlow[3]\thigh[1]\thigh[2]\thigh[3]\thigh[4]\n\r";

	for(int i = 0;i<4;i++)
	{
		motor_low[i].Kp = 13;
		motor_low[i].Ki = 6;
        motor_low[i].K_encoder = 10;
		motor_low[i].targetPosition = 0;
        motor_low[i].Kp_encoder = 22;
        motor_low[i].Ki_encoder = 5;
        motor_low[i].Kd_encoder = 5;
        
		motor_high[i].Kp = 13;
		motor_high[i].Ki = 6;
        motor_high[i].K_encoder = 10;
		motor_high[i].targetPosition = 0;
        motor_high[i].Kp_encoder = 22;
        motor_high[i].Ki_encoder = 5;
        motor_high[i].Kd_encoder = 5;
	}

	ros::param::get("ori2",motor_low[0].ori_encoder);
	ros::param::get("ori2",motor_low[1].ori_encoder);
	ros::param::get("ori3",motor_low[2].ori_encoder);
	ros::param::get("ori3",motor_low[3].ori_encoder);
	ros::param::get("ori1",motor_high[0].ori_encoder);
	ros::param::get("ori1",motor_high[1].ori_encoder);
	ros::param::get("ori0",motor_high[2].ori_encoder);
	ros::param::get("ori0",motor_high[3].ori_encoder);

    
    velocityPub_low = n.advertise<std_msgs::Int32MultiArray>("velocity_low",100);
    IPub_low = n.advertise<std_msgs::Int32MultiArray>("I_low",100);
	sendIPub_low = n.advertise<std_msgs::Int32MultiArray>("sendI_low",100);
    
	velocityPub_high = n.advertise<std_msgs::Int32MultiArray>("velocity_high",100);
    IPub_high = n.advertise<std_msgs::Int32MultiArray>("I_high",100);
	sendIPub_high = n.advertise<std_msgs::Int32MultiArray>("sendI_high",100);
    
	//leg_angle_Pub_low = n.advertise<std_msgs::Float32MultiArray>("leg_angle_low",100);
    //leg_angle_Pub_high = n.advertise<std_msgs::Float32MultiArray>("leg_angle_high",100);
    leg_angle_Pub_low = n.advertise<geometry_msgs::PolygonStamped>("leg_angle_low",100);
	leg_angle_Pub_high = n.advertise<geometry_msgs::PolygonStamped>("leg_angle_high",100);
    
	// leg_angle_sum_Pub_low = n.advertise<std_msgs::Float32MultiArray>("leg_angle_sum_low",100);
	// leg_angle_sum_Pub_high = n.advertise<std_msgs::Float32MultiArray>("leg_angle_sum_high",100);
	

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
	std::thread canTx_low(txThread_low, s_low);
    std::thread canTx_high(txThread_high, s_high);

    joy_sub = n.subscribe<sensor_msgs::Joy>("joy",10,buttonCallback);
    encoder_angle_sum = n.subscribe("MultiAngleSum",10,encoder_angle_sum_callback);
	upper_controller = n.subscribe("cmd_vel",10,upper_controller_callback);
	Tmotor_angle  = n.subscribe("Tmotor_Info", 10, Tmotor_angle_callback);
	while (ros::ok())
    {

		ros::spinOnce();

		loop_rate.sleep();
	}

    return 0;
}