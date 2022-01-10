#include "include/upper_controller_simple.hpp"
//配合 upper_controller_simple.h的一个imu&悬挂测试

double zero_pos, P_pit, D_pit, P_rol, D_rol, last_pitch, last_roll;

UpperController::UpperController()
{
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Controller parameter
  pn.param("P_pit", P_pit, 0.02);
  pn.param("D_pit", D_pit, 0.02);
  pn.param("P_rol", P_rol, 0.02);
  pn.param("D_rol", D_rol, 0.02);

  // Publishers and Subscribers
  imu_sub = n_.subscribe("/imu_rpy0", 1, &UpperController::imuCB, this);
  pub_suspension = n_.advertise<geometry_msgs::PolygonStamped>("/suspension_cmd", 1);

  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / 20),
                          &UpperController::controlLoopCB,
                          this); // Duration(0.05) -> 20Hz
}

void UpperController::controlLoopCB(const ros::TimerEvent &)
{
  std_msgs::Float32MultiArray imuMsg = imu_msg;
  susp_cmd.polygon.points.resize(4);

  // double roll = imuMsg.data[0];  // ego roll
  // double pitch = imuMsg.data[1]; // ego pitch
  // double yaw = imuMsg.data[2];

  // body control
  susp_cmd.polygon.points[2].x = P_pit * pitch + D_pit * (pitch - last_pitch);
  susp_cmd.polygon.points[0].x = -(P_rol * roll + D_rol * (roll - last_roll));
  susp_cmd.polygon.points[1].x = -(P_pit * pitch + D_pit * (pitch - last_pitch));
  susp_cmd.polygon.points[3].x = P_rol * roll + D_rol * (roll - last_roll);

  last_pitch = pitch;
  last_roll = roll;

  // limit max values
  for (int i = 0; i < 4; i++)
  {
    susp_cmd.polygon.points[i].x = fmin(fmax(susp_cmd.polygon.points[i].x, -8.0), 8.0);
  }

  ROS_INFO("----------");
  ROS_INFO("Roll:%.2f, Pitch:%.2f,Yaw: %.2f", roll, pitch, yaw);

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