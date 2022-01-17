#include "include/asoc_lower_platform.hpp"

int flag;

float frame_vt_max = 50;
float frame_vn_max = 30;
float frame_w_max = 300; // 100 = 320 degree/s

float power, forward_s;
float power_last;
int on_off = 1;

int judge_forward=0;          //high[1],high[2] angle 1
int judge_backward=0;         //high[3],high[4]   angle 0
int judge_left=0;             //low[1],low[2]    angle 2
int judge_right=0;            //low[3],low[4]   angle 3
float stick_forward = 0.0;    // forward command from joy
float stick_right = 0.0;      // right(+) command from joy
float stick_yaw = 0.0;        // yaw command from joy
float L = 2;   //  ratio of offset/split
float r = 0.09; //radius of the wheel
float linkTheta[4]; // the angle between four-parallel link and horizontal ground, variable, so need to be passed by Encoder
float D[4]={0.35,0.35,0.35,0.35};

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

ros::Publisher velocityPub_low, velocityPub_high; // pub velocity
ros::Publisher IPub_low, IPub_high; // pub current
ros::Publisher sendIPub_low, sendIPub_high; // pub cmd current
ros::Publisher leg_angle_Pub_high, leg_angle_Pub_low; // pub yaw of module
ros::Publisher RollPub_low, RollPub_high; // pub roll of module

ros::Subscriber joy_sub;
ros::Subscriber encoder_angle_sum;
ros::Subscriber upper_controller;
ros::Subscriber Tmotor_angle;

std_msgs::Int32MultiArray velocityMessage_low,velocityMessage_high,IMessage_low,IMessage_high,sendIMessage_low,sendIMessage_high;
geometry_msgs::PolygonStamped leg_angle_Message_low, leg_angle_Message_high;
geometry_msgs::PolygonStamped RollMsg_low, RollMsg_high;

Motor motor_low[4];
Motor motor_high[4];


void encoder_angle_sum_callback(std_msgs::Float32MultiArray MultiAngleSumMsg){
	motor_low[0].encoder_angle_with_turn = MultiAngleSumMsg.data[1];
	motor_low[1].encoder_angle_with_turn = MultiAngleSumMsg.data[1];
	motor_low[2].encoder_angle_with_turn = MultiAngleSumMsg.data[2];
	motor_low[3].encoder_angle_with_turn = MultiAngleSumMsg.data[2];
	motor_high[0].encoder_angle_with_turn = MultiAngleSumMsg.data[0];
	motor_high[1].encoder_angle_with_turn = MultiAngleSumMsg.data[0];
	motor_high[2].encoder_angle_with_turn = MultiAngleSumMsg.data[3];
	motor_high[3].encoder_angle_with_turn = MultiAngleSumMsg.data[3];

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
			linkTheta[i] = (15.52 * MultiTmotorAngle.polygon.points[i].x + 73 ) * PI / 180;
			D[i] = 0.178 + 0.17 * sin(linkTheta[i]);
		}
}


void buttonCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    stick_forward = joy->axes[1];
    stick_right = -joy->axes[0];
	stick_yaw = joy->axes[3];
    power = joy -> buttons[8];
	forward_s = joy->buttons[5];
    if(power>power_last){
        on_off = -on_off;
    }
    power_last = power;  

	frame_vt = 15*stick_forward + 19 * forward_s;
	frame_vn = 15*stick_right;
	frame_w =  -60*stick_yaw;
}


void upper_controller_callback(geometry_msgs::Twist cmd_vel){
	frame_vt = fmin(fmax(cmd_vel.linear.x, -frame_vt_max), frame_vt_max);
	frame_vn = fmin(fmax(cmd_vel.linear.y, -frame_vn_max), frame_vn_max);
	frame_w = fmin(fmax(cmd_vel.angular.z, -frame_w_max), frame_w_max);
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


void rxThread_low(int s)
{
	int ID_m, ID_r; // ID_m for motor, ID_r for encoder
	int i, j;
	struct can_frame frame_low;
	int nbytes_low;
	float roll_angle;

    velocityMessage_low.data.resize(5); // velocities of motors
	IMessage_low.data.resize(4); // Currents of motors
	RollMsg_low.polygon.points.resize(2); // Roll of two module  

	sleep(0.5);
	// wait for init

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
			perror("Read Error at low can");
			break;
		}
		// receive can_frame

		if(int(frame_low.can_id) < 0x200){
			ID_r = int(frame_low.data[1]-0x10); //encoder ID 0x10-0x11
			roll_angle = float(int(frame_low.data[4] << 8)+frame_low.data[3])/4096.0*120;
			if(roll_angle > 60.0){
				roll_angle = roll_angle - 120.0;
			}
			RollMsg_low.polygon.points[ID_r].x = roll_angle;
		}
		else{
			ID_m = int(frame_low.can_id-0x200)-1;
			motor_low[ID_m].angle = (frame_low.data[0] << 8)+ frame_low.data[1];
			motor_low[ID_m].realAngle = motor_low[ID_m].angle*360/8191;
			motor_low[ID_m].velocity = (frame_low.data[2] <<8) + frame_low.data[3];
			motor_low[ID_m].I = (frame_low.data[4] <<8) + frame_low.data[5];
			motor_low[ID_m].temperature = frame_low.data[6];
			
			if (i>6){
				motor_low[ID_m].angleDifference = motor_low[ID_m].angle - motor_low[ID_m].angleLast;
			}
			if(motor_low[ID_m].angleDifference<-4000)
			{
				motor_low[ID_m].NumOfTurns++;
			}
			if(motor_low[ID_m].angleDifference>4000)
			{
				motor_low[ID_m].NumOfTurns--;
			}

			motor_low[ID_m].position = 8192*motor_low[ID_m].NumOfTurns+motor_low[ID_m].angle;
			motor_low[ID_m].angleLast = motor_low[ID_m].angle;

			velocityMessage_low.data[ID_m] = motor_low[ID_m].velocity;
			IMessage_low.data[ID_m] = motor_low[ID_m].I;
		}

		if(i%6==0)
		{
			RollMsg_low.header.stamp=ros::Time::now();
			velocityPub_low.publish(velocityMessage_low);
			IPub_low.publish(IMessage_low);
			RollPub_low.publish(RollMsg_low);
		}

        if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				motor_low[j].targetPosition = motor_low[j].position;
			}
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
    }

}


void rxThread_high(int s)
{
	int ID_m, ID_r; // ID_m for motor, ID_r for encoder
	int i, j;
	struct can_frame frame_high;
	int nbytes_high;
	float roll_angle;

    velocityMessage_high.data.resize(5); // velocities of motors
	IMessage_high.data.resize(4); // Currents of motors
	RollMsg_high.polygon.points.resize(2); // Roll of two module  

	sleep(0.5);
	// wait for init

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
			perror("Read Error at high can");
			break;
		}
		// receive can_frame

		if(int(frame_high.can_id) < 0x200){
			ID_r = int(frame_high.data[1]-0x10); //encoder ID 0x10-0x11
			roll_angle = float(int(frame_high.data[4] << 8)+frame_high.data[3])/4096.0*120;
			if(roll_angle > 60.0){
				roll_angle = roll_angle - 120.0;
			}
			RollMsg_high.polygon.points[ID_r].x = roll_angle;
		}
		else{
			ID_m = int(frame_high.can_id-0x200)-1;
			motor_high[ID_m].angle = (frame_high.data[0] << 8)+ frame_high.data[1];
			motor_high[ID_m].realAngle = motor_high[ID_m].angle*360/8191;
			motor_high[ID_m].velocity = (frame_high.data[2] <<8) + frame_high.data[3];
			motor_high[ID_m].I = (frame_high.data[4] <<8) + frame_high.data[5];
			motor_high[ID_m].temperature = frame_high.data[6];
			
			if (i>6){
				motor_high[ID_m].angleDifference = motor_high[ID_m].angle - motor_high[ID_m].angleLast;
			}
			if(motor_high[ID_m].angleDifference<-4000)
			{
				motor_high[ID_m].NumOfTurns++;
			}
			if(motor_high[ID_m].angleDifference>4000)
			{
				motor_high[ID_m].NumOfTurns--;
			}

			motor_high[ID_m].position = 8192*motor_high[ID_m].NumOfTurns+motor_high[ID_m].angle;
			motor_high[ID_m].angleLast = motor_high[ID_m].angle;

			velocityMessage_high.data[ID_m] = motor_high[ID_m].velocity;
			IMessage_high.data[ID_m] = motor_high[ID_m].I;
		}

        if(i%6==0)
		{
			RollMsg_high.header.stamp=ros::Time::now();
			velocityPub_high.publish(velocityMessage_high);
			IPub_high.publish(IMessage_high);
			RollPub_high.publish(RollMsg_high);
		}
        if(i == 16)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				motor_high[j].targetPosition = motor_high[j].position;
			}
			
		}
		if(i%600 == 0){
			ROS_INFO("Vt,Vn,w   [%.2f, %.2f, %.2f]\r", frame_vt, frame_vn, frame_w);
			ROS_INFO("leg_angle [%.2f, %.2f, %.2f, %.2f]\r",motor_low[0].leg_angle,motor_low[2].leg_angle,motor_high[0].leg_angle,motor_high[2].leg_angle);   
			ROS_INFO("D[0,2,1,3][%.2f, %.2f, %.2f, %.2f]\r",D[0], D[2], D[1], D[3]);        
            ROS_INFO("target_V_L[%.2f, %.2f, %.2f, %.2f]\r",motor_low[0].targetVelocity,motor_low[1].targetVelocity,motor_low[2].targetVelocity,motor_low[3].targetVelocity);
			ROS_INFO("target_V_H[%.2f, %.2f, %.2f, %.2f]\r",motor_high[0].targetVelocity,motor_high[1].targetVelocity,motor_high[2].targetVelocity,motor_high[3].targetVelocity);
			ROS_INFO("Roll      [%.2f, %.2f, %.2f, %.2f]\r",RollMsg_low.polygon.points[0].x, RollMsg_low.polygon.points[1].x, RollMsg_high.polygon.points[0].x, RollMsg_high.polygon.points[1].x);

			std::cout << "----------------\n";
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(10000));
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
  

	sleep(0.5);
	// wait for init

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
			printf("send error at low\n");
        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(2000000));
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
	
	sleep(0.5);
	// wait for init
	
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
			printf("send error at high\n");
        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(2000000));
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

	// get initial leg_angle for each module
	ros::param::get("ori1",motor_low[0].ori_encoder);
	ros::param::get("ori1",motor_low[1].ori_encoder);
	ros::param::get("ori2",motor_low[2].ori_encoder);
	ros::param::get("ori2",motor_low[3].ori_encoder);
	ros::param::get("ori0",motor_high[0].ori_encoder);
	ros::param::get("ori0",motor_high[1].ori_encoder);
	ros::param::get("ori3",motor_high[2].ori_encoder);
	ros::param::get("ori3",motor_high[3].ori_encoder);
	ROS_INFO("ori       [%.2f, %.2f, %.2f, %.2f]\r",motor_low[0].ori_encoder,motor_low[2].ori_encoder,motor_high[0].ori_encoder,motor_high[2].ori_encoder);	  

    velocityPub_low = n.advertise<std_msgs::Int32MultiArray>("velocity_low",100);
    IPub_low = n.advertise<std_msgs::Int32MultiArray>("I_low",100);
	sendIPub_low = n.advertise<std_msgs::Int32MultiArray>("sendI_low",100);
	leg_angle_Pub_low = n.advertise<geometry_msgs::PolygonStamped>("leg_angle_low",100);
	RollPub_low = n.advertise<geometry_msgs::PolygonStamped>("Roll_low",100);;

	velocityPub_high = n.advertise<std_msgs::Int32MultiArray>("velocity_high",100);
    IPub_high = n.advertise<std_msgs::Int32MultiArray>("I_high",100);
	sendIPub_high = n.advertise<std_msgs::Int32MultiArray>("sendI_high",100);
	leg_angle_Pub_high = n.advertise<geometry_msgs::PolygonStamped>("leg_angle_high",100);
	RollPub_high = n.advertise<geometry_msgs::PolygonStamped>("Roll_high",100);

	int s_low;
	int s_high;
	struct sockaddr_can addr_low;
    struct sockaddr_can addr_high;
	struct ifreq ifr_low;
    struct ifreq ifr_high;
	struct can_frame frame;

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

	CanInit(frame, s_low);
	CanInit(frame, s_high);

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