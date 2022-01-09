//IMU获取车体姿态信息，取代相机的一个测试

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
#include "sensor_msgs/Imu.h"

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class UpperController
{
public:
  UpperController();
  double getRollFromPose(const sensor_msgs::Imu &sensorMsg);
  double getPitchFromPose(const sensor_msgs::Imu &sensorMsg);
  double getYawFromPose(const sensor_msgs::Imu &sensorMsg);
  void imuCB(const sensor_msgs::Imu &sensorMsg);
  void controlLoopCB(const ros::TimerEvent &);

private:
  ros::NodeHandle n_;
  ros::Subscriber imu_sub;
  ros::Publisher pub_suspension;
  ros::Timer timer1;

  geometry_msgs::PolygonStamped susp_cmd;
  sensor_msgs::Imu imu_msg;
};

void UpperController::imuCB(const sensor_msgs::Imu &sensorMsg)
{
  imu_msg = sensorMsg;
}

double UpperController::getRollFromPose(const sensor_msgs::Imu &sensorMsg)
{
  float x = sensorMsg.orientation.x;
  float y = sensorMsg.orientation.y;
  float z = sensorMsg.orientation.z;
  float w = sensorMsg.orientation.w;

  double tmp, roll;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(roll, tmp, tmp);

  return roll;
}

double UpperController::getPitchFromPose(const sensor_msgs::Imu &sensorMsg)
{
  float x = sensorMsg.orientation.x;
  float y = sensorMsg.orientation.y;
  float z = sensorMsg.orientation.z;
  float w = sensorMsg.orientation.w;

  double tmp, pitch;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, pitch, tmp);

  return pitch;
}

double UpperController::getYawFromPose(const sensor_msgs::Imu &sensorMsg)
{
  float x = sensorMsg.orientation.x;
  float y = sensorMsg.orientation.y;
  float z = sensorMsg.orientation.z;
  float w = sensorMsg.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}
