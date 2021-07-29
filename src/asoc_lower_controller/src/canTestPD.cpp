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


uint16_t KP,KI;

int flag;

int rxCounter_low;
int rxCounter_high;

int judge=1;
int co =0;

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
	int32_t velocityDifferenceSum;
	int32_t velocity_diff;    //the difference between the latest velocity difference and the last velocity difference.

	float Ki;
	float Kp;
	float Kd;

	float K;

    

};
Motor motor_low[4];
Motor motor_high[4];

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

void printfSingleMotor(Motor motor){
	printf("angle is %d,realAngle is %d,position is %d,velocity is %d,I is %d\r\n",
	motor.angle,motor.realAngle,motor.position,motor.velocity,motor.I);

}

void printfMultiMotorAngle_low(void){
	printf("angle is %d,%d,%d,%d;\r\n",motor_low[0].angle,motor_low[1].angle,motor_low[2].angle,motor_low[3].angle);
}

void printfMultiMotorAngle_high(void){
	printf("angle is %d,%d,%d,%d;\r\n",motor_high[0].angle,motor_high[1].angle,motor_high[2].angle,motor_high[4].angle);
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
    judge = joy->buttons[1];
    
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
			
            positionPub_low.publish(positionMessage_low);
			velocityPub_low.publish(velocityMessage_low);
			IPub_low.publish(IMessage_low);

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
			printfMultiMotorPosition_high();
			printfMultiMotorVelocity_high();
			printfMultiMotorI_high();
			printf("HIGH tp is %d,%d\r\n",motor_high[0].targetPosition,motor_high[1].targetPosition,motor_high[2].targetPosition,motor_high[3].targetPosition);
			printf("HIGH k kp ki are %f,%frun,%f\r\n",motor_high[0].K,motor_high[0].Kp,motor_high[0].Ki);
			
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

void controlV_calSendI_PD_high(int ID){
	int limit = 16667;
	motor_high[ID].velocityDifference = motor_high[ID].targetVelocity - motor_high[ID].velocity;
	if(motor_high[ID].velocityDifference > 1000) motor_high[ID].velocityDifference = 1000;
	if(motor_high[ID].velocityDifference < -1000) motor_high[ID].velocityDifference = -1000;

	motor_high[ID].velocity_diff = motor_high[ID].velocityDifference - motor_high[ID].velocityDifference_last;
	if(motor_high[ID].velocity_diff>500) motor_high[ID].velocity_diff = 500;
	if(motor_high[ID].velocity_diff<-500) motor_high[ID].velocity_diff = -500;
    
	motor_high[ID].sendI = motor_high[ID].Kp*motor_high[ID].velocityDifference + motor_high[ID].Kd*motor_high[ID].velocity_diff;
	
	motor_high[ID].velocityDifference_last = motor_high[ID].velocityDifference;
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

void txThread_low(int s)
{
    struct can_frame frame_low;
    
	frame_low.can_id = 0x200;
    
	frame_low.can_dlc = 8;
    
	int j;
	sendIMessage_low.data.resize(4);
    
    
	for(j=0; j<4; j++){
        motor_low[j].sendI = 0;
    }
    for (j = 0; j < 4; j++)
	{
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
			// controlV_calSendI_PI_low(0);
			// controlV_calSendI_PI_low(1);
			// controlV_calSendI_PI_low(2);
			// controlV_calSendI_PI_low(3);
            controlV_calSendI_PD_low(0);
            controlV_calSendI_PD_low(1);
            controlV_calSendI_PD_low(2);
            controlV_calSendI_PD_low(3);
			// controlP_calSendI_PI(0);
			// controlP_calSendI_PI(1);
			// controlP_calSendI_PI(2);
			// controlP_calSendI_PI(3);
			
		}
		
		for (j = 0;j<4;j++)
		{
			if(judge==1){
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
            else{
                frame_low.data[2*j] = 0;
			    frame_low.data[2*j+1] = 0;
               
			    sendIMessage_low.data[j] = 0;
                
                
                
            }
            

		}
    
		sendIPub_low.publish(sendIMessage_low);
        // sendIPub.publish(sendIMessage_high);
        
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
			// controlV_calSendI_PI_high(0);
			// controlV_calSendI_PI_high(1);
			// controlV_calSendI_PI_high(2);
			// controlV_calSendI_PI_high(3);
            controlV_calSendI_PD_high(0);
            controlV_calSendI_PD_high(1);
            controlV_calSendI_PD_high(2);
            controlV_calSendI_PD_high(3);
			// controlP_calSendI_PI(0);
			// controlP_calSendI_PI(1);
			// controlP_calSendI_PI(2);
			// controlP_calSendI_PI(3);
		}
		
		for (j = 0;j<4;j++)
		{
			if(judge==1){
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
            else{
                frame_high.data[2*j] = 0;
			    frame_high.data[2*j+1] = 0;
               
			    sendIMessage_high.data[j] = 0;
                
                
                
            }
            

		}
    
		
        sendIPub_high.publish(sendIMessage_high);
        
        nbytes_high = write(s, &frame_high, sizeof(struct can_frame));
        
        if (nbytes_high == -1 ) {
			printf("send error\n");
        }
        
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }

}


int main(int argc, char** argv) {
	flag = 0;

	ros::init(argc,argv,"canTest5");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	for(int i = 0;i<4;i++)
	{
		
		
		if(i%2==0)
		{
			motor_low[i].targetVelocity = 700;
            motor_high[i].targetVelocity = 700;
		}
		else
		{
			motor_low[i].targetVelocity = -700;
            motor_high[i].targetVelocity = -700;
		}
		
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
        motor_low[i].Kd = 10;        
        motor_high[i].Kp = 13;
		motor_high[i].Ki = 6;
		motor_high[i].K = 15;
        motor_high[i].Kd = 10;

	}

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
	sleep(0.5);
    std::thread canRX_high(rxThread_high, s_high);
	sleep(0.5);
	std::thread canTx_low(txThread_low, s_low);
	sleep(0.5);
    std::thread canTx_high(txThread_high, s_high);
    PISub_low = n.subscribe("PI", 10, PI_Callback_low);
	PSub_low = n.subscribe("P",10,P_Callback_low);
	targetVelocitySub_low = n.subscribe("targetVelocity", 10, targetVelocity_Callback_low);
	targetPositionSub_low = n.subscribe("targetPosition", 10, targetPosition_Callback_low);
    PISub_high = n.subscribe("PI", 10, PI_Callback_high);
	PSub_high = n.subscribe("P",10,P_Callback_high);
	targetVelocitySub_high = n.subscribe("targetVelocity", 10, targetVelocity_Callback_high);
	targetPositionSub_high = n.subscribe("targetPosition", 10, targetPosition_Callback_high);
    joy_sub = n.subscribe<sensor_msgs::Joy>("joy",10,buttonCallback);
	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}