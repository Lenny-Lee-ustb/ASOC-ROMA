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

int Stop_flag = 0;

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