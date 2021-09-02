#include "include/upper_controller.hpp"

double last_d_theta = 0;
double last_lateral_dist = 0;
double last_speed = 0;
float last_roll = 0;
float last_pitch = 0;

UpperController::UpperController() {
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Controller parameter
  pn.param("controller_freq", controller_freq, 20.0);
  pn.param("goalRadius", goalRadius, 1.0);
  pn.param("goal_pose_err", goal_pose_err, 1.0);

  pn.param("baseSpeed", baseSpeed, 0.0);
  pn.param("rot_angle", rot_angle, 0.0);
  pn.param("P_Long", P_Long, 1.0);
  pn.param("I_Long", I_Long, 1.0);
  pn.param("D_Long", D_Long, 1.0);

  pn.param("P_Yaw", P_Yaw, 1.0);
  pn.param("I_Yaw", I_Yaw, 0.0);
  pn.param("D_Yaw", D_Yaw, 1.0);
  pn.param("forward_dist", forward_dist, 1.0);


  pn.param("P_Lateral", P_Lateral, 1.0);
  pn.param("I_Lateral", I_Lateral, 1.0);
  pn.param("D_Lateral", D_Lateral, 1.0);

  pn.param("Kp", Kp, 1.0);
  pn.param("Kd", Kd, 0.0);
  pn.param("zero_pos", zero_pos, 1.0);
  pn.param("roll_rot_factor", roll_rot_factor, 1.0);
  pn.param("roll_lat_factor", roll_lat_factor, 1.0);
  pn.param("velocity_factor",velocity_factor,0.1);
  pn.param("P_pit", P_pit, 150.0);
  pn.param("D_pit", D_pit, 1.0);
  pn.param("P_rol", P_rol, 150.0);
  pn.param("D_rol", D_rol, 1.0);
  // Publishers and Subscribers
  odom_sub = n_.subscribe("/odometry/filtered", 1, &UpperController::odomCB, this);

  path_sub = n_.subscribe("/fix_path", 1,
                          &UpperController::pathCB, this);
  goal_sub = 
             n_.subscribe("/move_base_simple/goal", 1, &UpperController::goalCB, this);

  marker_pub = n_.advertise<visualization_msgs::Marker>("/car_path", 10);

  pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  pub_suspension = n_.advertise<std_msgs::Float32MultiArray>("/suspension_cmd", 1);

  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &UpperController::controlLoopCB,
                          this); // Duration(0.05) -> 20Hz
  timer2 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &UpperController::goalReachingCB,
                          this); // Duration(0.05) -> 20Hz

  // Init variables
  goal_received = false;
  goal_reached = false;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;

  // Show param info
  ROS_INFO("[param] controller_freq: %.2f", controller_freq);
  ROS_INFO("[param] goalRadius: %.2f", goalRadius);
  ROS_INFO("[param] goal_pose_err: %.2f", goal_pose_err);
  ROS_INFO("[param] baseSpeed: %.2f", baseSpeed);
  ROS_INFO("[param] forward_dist: %.2f", forward_dist);
  ROS_INFO("[param] rot_angle: %.2f", rot_angle);
  ROS_INFO("[param] P_Yaw: %.2f", P_Yaw);
  ROS_INFO("[param] I_Yaw: %.2f", I_Yaw);
  ROS_INFO("[param] D_Yaw: %.2f", D_Yaw);
  ROS_INFO("[param] P_Lateral: %.2f", P_Lateral);
  ROS_INFO("[param] I_Lateral: %.2f", I_Lateral);
  ROS_INFO("[param] D_Lateral: %.2f", D_Lateral);
  ROS_INFO("[param] P_Long: %.2f", P_Long);
  ROS_INFO("[param] I_Long: %.2f", I_Long);
  ROS_INFO("[param] D_Long: %.2f", D_Long);
  ROS_INFO("[param] Kp: %.2f", Kp);
  ROS_INFO("[param] Kd: %.2f", Kd);
  ROS_INFO("[param] zero_pos: %.2f", zero_pos);
  ROS_INFO("[param] roll_rot_factor: %.2f", roll_rot_factor);  
  ROS_INFO("[param] roll_lat_factor: %.2f", roll_lat_factor);
  ROS_INFO("[param] velocity_factor: %.2f",velocity_factor);
  ROS_INFO("[param] P_pit, D_pit: %.2f, %.2f", P_pit, D_pit);  
  ROS_INFO("[param] P_rol, D_pit: %.2f, %.2f", P_rol, D_rol);
  // Visualization Marker Settings
  initMarker();
}

void UpperController::controlLoopCB(const ros::TimerEvent &) {

  geometry_msgs::Pose carPose = odom.pose.pose;
  geometry_msgs::Twist carVel = odom.twist.twist;
  geometry_msgs::Pose LateralPose = getTrackPose(carPose);
  geometry_msgs::Pose ForwardPose = getTrackForwardPose(carPose,forward_dist);
  double LateralDir = GetLateralDir(carPose, LateralPose);
  double rot_rad = rot_angle / 180.0 * PI;
  double vt,vn,w;
  lateral_dist = LateralDir * getLateralDist(carPose, LateralPose);

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;
  // susp_cmd.data={float(zero_pos),float(zero_pos),
  //                float(zero_pos),float(zero_pos)};
  susp_cmd.data={0,0,
                 0,0};

  if (goal_received) {
    double thetar = getYawFromPose(carPose); // ego yaw
    double theta = getYawFromPose(ForwardPose);// yaw on path
    double roll = getRollFromPose(carPose); // ego roll
    double pitch =  getPitchFromPose(carPose); // ego pitch
    double rollForward = getRollFromPose(ForwardPose);
    double pitchForward = getPitchFromPose(ForwardPose);

    double d_theta = theta - thetar;
    double d_roll = rollForward - roll;
    double d_pitch = pitchForward - pitch;
    double slow_factor = 1.0- 1.5 * fabs(pow(d_theta/3.14,3));
    if (foundForwardPt) {
        if (!goal_reached) {
          // PID control
          w = - (P_Yaw * d_theta + D_Yaw * (d_theta - last_d_theta));
          vt = slow_factor * P_Long;
          vn = -(P_Lateral * lateral_dist + D_Lateral * (lateral_dist - last_lateral_dist));
          
          last_speed = baseSpeed - carVel.linear.x;
          last_d_theta = d_theta;
          last_lateral_dist = lateral_dist;
          
          // Rot_angle
          cmd_vel.angular.z = w;
          cmd_vel.linear.y = vn * cos(rot_rad) - vt * sin(rot_rad);//vn'
          cmd_vel.linear.x = vn * sin(rot_rad) + vt * cos(rot_rad);//vt'

          // body control
          susp_cmd.data[0] = P_pit * pitch + D_pit * (pitch - last_pitch) + P_rol * roll + D_rol * (roll - last_roll);
          susp_cmd.data[1] = P_pit * pitch + D_pit * (pitch - last_pitch) - (P_rol * roll + D_rol * (roll - last_roll));
          susp_cmd.data[2] = -(P_pit * pitch + D_pit * (pitch - last_pitch)) - (P_rol * roll + D_rol * (roll - last_roll));
          susp_cmd.data[3] = -(P_pit * pitch + D_pit * (pitch - last_pitch)) + P_rol * roll + D_rol * (roll - last_roll);

          last_pitch = pitch;
          last_roll = roll;
/****************************************************************************************/
          // if(d_theta > PI/12.0 || d_theta < -PI/12.0){
          //   susp_cmd.data[0] = susp_cmd.data[3] = zero_pos + d_theta * roll_rot_factor + lateral_dist * roll_lat_factor + sqrt(carVel.linear.x*carVel.linear.x+carVel.linear.y*carVel.linear.y)*velocity_factor; // add velocity factor
          //   susp_cmd.data[1] = susp_cmd.data[2] = zero_pos - d_theta * roll_rot_factor - lateral_dist * roll_lat_factor - sqrt(carVel.linear.x*carVel.linear.x+carVel.linear.y*carVel.linear.y)*velocity_factor; 
          // };
          
            // susp_cmd.data[0] = susp_cmd.data[3] = zero_pos + lateral_dist * roll_factor; // right of the body up (d_theta<0)
            // susp_cmd.data[1] = susp_cmd.data[2] = zero_pos - lateral_dist * roll_factor; // left  of the body up (d_theta>0)
/***************************************************************************************/

          // limit max values
          for(int i=0; i<4; i++){
            susp_cmd.data[i] = fmin(fmax(susp_cmd.data[i],-8.0),8.0);
          }

          cmd_vel.linear.x=fmax(cmd_vel.linear.x,0);
          cmd_vel.linear.y=fmin(fmax(cmd_vel.linear.y,-100.0),100.0);
          cmd_vel.linear.z=fmin(fmax(cmd_vel.linear.z,-100.0),100.0);

          ROS_INFO("----------");
          ROS_INFO("Roll:%.2f, Pitch:%.2f, Yaw:%.2f",roll,pitch,thetar);
          ROS_INFO("d_yaw:%.2f, slow_factor:%.2f",d_theta,slow_factor);
          // ROS_INFO("pos:(%.2f,%.2f)",ForwardPose.position.x,ForwardPose.position.y);
          ROS_INFO("lateral_dist:%.2f, long_vel:%.2f",lateral_dist,carVel.linear.x);
          ROS_INFO("Vyaw:%.2f, Vt:%.2f, Vn:%.2f",w,vt,vn);
        }
    }
    pub_.publish(cmd_vel);
    pub_suspension.publish(susp_cmd);
  }else{
    cmd_vel.angular.z = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.x = 0;
    pub_.publish(cmd_vel);
    pub_suspension.publish(susp_cmd);
  }
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv) {
  // Initiate ROS
  ros::init(argc, argv, "UpperController");
  UpperController controller;
  ros::spin();
  return 0;
}