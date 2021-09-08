#include "include/upper_controller.hpp"

float last_roll = 0;
float last_pitch = 0;
double zero_pos, velocity_factor, P_pit, D_pit, P_rol, D_rol;

UpperController::UpperController()
{
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Controller parameter
  pn.param("controller_freq", controller_freq, 100.0);
  pn.param("P_pit", P_pit, 150.0);
  pn.param("D_pit", D_pit, 1.0);
  pn.param("P_rol", P_rol, 150.0);
  pn.param("D_rol", D_rol, 1.0);
  // Publishers and Subscribers
  odom_sub = n_.subscribe("/odometry/filtered", 1, &UpperController::odomCB, this);

  pub_suspension = n_.advertise<geometry_msgs::PolygonStamped>("/suspension_cmd", 1);

  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &UpperController::controlLoopCB,
                          this); // Duration(0.05) -> 20Hz

  // Show param info
  ROS_INFO("[param] controller_freq: %.2f", controller_freq);
  ROS_INFO("[param] P_pit, D_pit: %.2f, %.2f", P_pit, D_pit);
  ROS_INFO("[param] P_rol, D_pit: %.2f, %.2f", P_rol, D_rol);
  // Visualization Marker Settings
}

void UpperController::controlLoopCB(const ros::TimerEvent &)
{

  geometry_msgs::Pose carPose = odom.pose.pose;
  geometry_msgs::Twist carVel = odom.twist.twist;

  susp_cmd.polygon.points.resize(4);

  double roll = getRollFromPose(carPose);   // ego roll
  double pitch = getPitchFromPose(carPose); // ego pitch
  // double rollForward = getRollFromPose(ForwardPose);
  // double pitchForward = getPitchFromPose(ForwardPose);
  double rollForward = 0.0;
  double pitchForward = 0.0;

  // body control
  susp_cmd.polygon.points[0].x = P_pit * pitch + D_pit * (pitch - last_pitch) + P_rol * roll + D_rol * (roll - last_roll);
  susp_cmd.polygon.points[1].x = P_pit * pitch + D_pit * (pitch - last_pitch) - (P_rol * roll + D_rol * (roll - last_roll));
  susp_cmd.polygon.points[2].x = -(P_pit * pitch + D_pit * (pitch - last_pitch)) - (P_rol * roll + D_rol * (roll - last_roll));
  susp_cmd.polygon.points[3].x = -(P_pit * pitch + D_pit * (pitch - last_pitch)) + P_rol * roll + D_rol * (roll - last_roll);

  last_pitch = pitch;
  last_roll = roll;

  // limit max values
  for (int i = 0; i < 4; i++)
  {
    susp_cmd.polygon.points[i].x = fmin(fmax(susp_cmd.polygon.points[i].x, -8.0), 8.0);
  }

  ROS_INFO("----------");
  ROS_INFO("Roll:%.2f, Pitch:%.2f", roll, pitch);

  susp_cmd.header.stamp = ros::Time::now();
  pub_suspension.publish(susp_cmd);
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "UpperController");
  UpperController controller;
  ros::spin();
  return 0;
}