//IMU 直接读取rpy

#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <asoc_upper_controller/controller_Config.h>
#include <ctime>
#include <math.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class UpperController
{
public:
  UpperController();
  double getRollFromPose(const std_msgs::Float32MultiArray &sensorMsg);
  double getPitchFromPose(const std_msgs::Float32MultiArray &sensorMsg);
  double getYawFromPose(const std_msgs::Float32MultiArray &sensorMsg);
  void imuCB(const std_msgs::Float32MultiArray &sensorMsg);
  void controlLoopCB(const ros::TimerEvent &);

private:
  ros::NodeHandle n_;
  ros::Subscriber imu_sub;
  ros::Publisher pub_suspension;
  ros::Timer timer1;

  geometry_msgs::PolygonStamped susp_cmd;
  std_msgs::Float32MultiArray imu_msg;
  double roll,pitch,yaw;
};

void UpperController::imuCB(const std_msgs::Float32MultiArray &sensorMsg)
{
  // imu_msg = sensorMsg;
  roll=sensorMsg.data[0];
  pitch=sensorMsg.data[1];
  yaw=sensorMsg.data[2];
}
