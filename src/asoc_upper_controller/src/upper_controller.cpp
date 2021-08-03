#include "upper_controller.hpp"

double last_d_theta = 0;

UpperController::UpperController() {
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Controller parameter
  pn.param("controller_freq", controller_freq, 20.0);
  pn.param("baseSpeed", baseSpeed, 0.0);
  pn.param("goalRadius", goalRadius, 1.0);
  pn.param("goal_pose_err", goal_pose_err, 1.0);
  pn.param("P_Yaw", P_Yaw, 1.0);
  pn.param("I_Yaw", I_Yaw, 0.0);
  pn.param("D_Yaw", D_Yaw, 1.0);
  pn.param("P_Lateral", P_Lateral, 1.0);
  pn.param("I_Lateral", I_Lateral, 1.0);
  pn.param("D_Lateral", D_Lateral, 1.0);

  // Publishers and Subscribers
  odom_sub = n_.subscribe("/odometry/filtered", 1, &UpperController::odomCB, this);

  path_sub = n_.subscribe("/fix_path", 1,
                          &UpperController::pathCB, this);
  goal_sub =
      n_.subscribe("/move_base_simple/goal", 1, &UpperController::goalCB, this);

  marker_pub = n_.advertise<visualization_msgs::Marker>("/car_path", 10);

  pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &UpperController::controlLoopCB,
                          this); // Duration(0.05) -> 20Hz
  timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq),
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
  ROS_INFO("[param] P_Yaw: %.2f", P_Yaw);
  ROS_INFO("[param] I_Yaw: %.2f", I_Yaw);
  ROS_INFO("[param] D_Yaw: %.2f", D_Yaw);
  ROS_INFO("[param] P_Lateral: %.2f", P_Lateral);
  ROS_INFO("[param] I_Lateral: %.2f", I_Lateral);
  ROS_INFO("[param] D_Lateral: %.2f", D_Lateral);

  // Visualization Marker Settings
  initMarker();
}

void UpperController::controlLoopCB(const ros::TimerEvent &) {

  geometry_msgs::Pose carPose = odom.pose.pose;
  geometry_msgs::Twist carVel = odom.twist.twist;
  geometry_msgs::Pose ForwardPose = getTrackPose(carPose);
  lateral_dist = getLateralDist(carPose, ForwardPose);
  cmd_vel.linear.x = baseSpeed;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;

  if (goal_received) {
    double thetar = getYawFromPose(carPose);
    double theta = getYawFromPose(ForwardPose);
    double d_theta = theta - thetar;
    if (foundForwardPt) {
        cmd_vel.angular.z = P_Yaw * d_theta + D_Yaw * (d_theta - last_d_theta);//PD control here!! no finish
        cmd_vel.linear.y = -(P_Lateral * lateral_dist);

    last_d_theta = d_theta;
    if (!goal_reached) {
        cmd_vel.linear.x =baseSpeed;
    }
  }
  ROS_INFO("----------");
  ROS_INFO("Yaw:%.2f, TrackYaw:%.2f",thetar,theta);
  ROS_INFO("lateral:%.2f",lateral_dist);
  ROS_INFO("Vyaw:%.2f, Vt:%.2f, Vn:%.2f",cmd_vel.angular.z,cmd_vel.linear.x,cmd_vel.linear.y);
  pub_.publish(cmd_vel);
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