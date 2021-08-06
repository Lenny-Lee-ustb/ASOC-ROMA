#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <ctime>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class UpperController {
public:
    UpperController();
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point &wayPt,
                                                  const geometry_msgs::Pose &carPose);
    double getYawFromPose(const geometry_msgs::Pose &carPose);
    double getLateralDist(const geometry_msgs::Pose &carPose,const geometry_msgs::Pose &ForwardPt);
    double getCar2GoalDist();
    double GetLateralDir(const geometry_msgs::Pose &carPose,const geometry_msgs::Pose &ForwardPose);
    double isRightorLeft(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose);
    double getEta(const geometry_msgs::Pose &carPose);
    geometry_msgs::Pose  getTrackPose(const geometry_msgs::Pose &carPose);

private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub, path_sub, goal_sub;
  ros::Publisher pub_, marker_pub;
  ros::Timer timer1, timer2;
  tf::TransformListener tf_listener;

  visualization_msgs::Marker points, line_strip, goal_circle;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point odom_goal_pos;
  nav_msgs::Odometry odom;
  nav_msgs::Path map_path;

  double controller_freq, baseSpeed;
  double  goalRadius, goal_pose_err;
  double lateral_dist;
  double P_Yaw, I_Yaw, D_Yaw;
  double P_Lateral, I_Lateral, D_Lateral;
  double P_Long, I_Long, D_Long;

  bool foundForwardPt,goal_received, goal_reached;

  void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
  void goalReachingCB(const ros::TimerEvent &);

  void controlLoopCB(const ros::TimerEvent &);

};
void UpperController::initMarker(){
    points.header.frame_id = line_strip.header.frame_id =
        goal_circle.header.frame_id = "camera_odom_frame";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action =
        visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w =
        goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    // LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.02;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

void UpperController::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  odom = *odomMsg;
}

void UpperController::pathCB(const nav_msgs::Path::ConstPtr &pathMsg) {
  map_path = *pathMsg;
}

void UpperController::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
  try {
    geometry_msgs::PoseStamped odom_goal;
    odom_goal = *goalMsg;
    tf_listener.transformPose("camera_odom_frame", ros::Time(0), *goalMsg, "map", odom_goal);
    odom_goal_pos = odom_goal.pose.position;
    goal_received = true;
    goal_reached = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

double UpperController::getYawFromPose(const geometry_msgs::Pose &carPose) {
  float x = carPose.orientation.x;
  float y = carPose.orientation.y;
  float z = carPose.orientation.z;
  float w = carPose.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}

double UpperController::getLateralDist(const geometry_msgs::Pose &carPose,const geometry_msgs::Pose &ForwardPt){

  double car2pt_x = ForwardPt.position.x - carPose.position.x;
  double car2pt_y = ForwardPt.position.y - carPose.position.y;
  double dist = sqrt(car2pt_x * car2pt_x + car2pt_y * car2pt_y);
  
  return dist;
}

double UpperController::getCar2GoalDist() {
  geometry_msgs::Point car_pose = odom.pose.pose.position;
  double car2goal_x = odom_goal_pos.x - car_pose.x;
  double car2goal_y = odom_goal_pos.y - car_pose.y;

  double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

  return dist2goal;
}

void UpperController::goalReachingCB(const ros::TimerEvent &) {

  if (goal_received) {
    double car2goal_dist = getCar2GoalDist();
    if (car2goal_dist < goalRadius && 0) {
        // !!! not finish here !!!
      goal_reached = true;
      goal_received = false;
      ROS_INFO("Goal Reached !");
    }
  }
}

bool UpperController::isForwardWayPt(const geometry_msgs::Point &wayPt,
                                  const geometry_msgs::Pose &carPose) {
  float car2wayPt_x = wayPt.x - carPose.position.x;
  float car2wayPt_y = wayPt.y - carPose.position.y;
  double car_theta = getYawFromPose(carPose);

  float car_car2wayPt_x =
      cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y =
      -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

  if (car_car2wayPt_x > 0) /*is Forward WayPt*/
    return true;
  else
    return false;
}

double UpperController::isRightorLeft(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose){
  float car2wayPt_x = wayPt.x - carPose.position.x;
  float car2wayPt_y = wayPt.y - carPose.position.y;
  double car_theta = getYawFromPose(carPose);

  float car_car2wayPt_x = 
      cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y = 
      -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;
  
  // true means Left, false means right
  if (car_car2wayPt_y > 0)
      return 1;
  else
      return -1;
}

double UpperController::GetLateralDir(const geometry_msgs::Pose &carPose,const geometry_msgs::Pose &ForwardPose){
  double ForwardPose_yaw = getYawFromPose(ForwardPose);
  double ForwardPose_x,ForwardPose_y;
  double Car_x,Car_y;

  ForwardPose_x = cos(ForwardPose_yaw);
  ForwardPose_y = sin(ForwardPose_yaw);
  Car_x = carPose.position.x - ForwardPose.position.x;
  Car_y = carPose.position.y - ForwardPose.position.y;

  if(ForwardPose_x*Car_y - ForwardPose_y*Car_x >=0){
      return -1;
  }
  else{
      return 1;
  }
  }

geometry_msgs::Pose UpperController::getTrackPose(const geometry_msgs::Pose &carPose){
clock_t startTime,endTime;
double carPose_yaw = getYawFromPose(carPose);
double min_dist = 10;
double min_i;
geometry_msgs::PointStamped forwardPt;
geometry_msgs::Point odom_car2WayPtVec;
geometry_msgs::Point carPose_pos = carPose.position;
geometry_msgs::Pose forwardPose;


if (!goal_reached) {
    // startTime = clock();
    for (int i = 0; i < map_path.poses.size(); i++) {
    // for (int i = map_path.poses.size(); i >0 ; i--) {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;
            try
            {
                tf_listener.transformPose("camera_odom_frame", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                double dist = sqrt((odom_path_wayPt.x-carPose_pos.x)*(odom_path_wayPt.x-carPose_pos.x) +
                                   (odom_path_wayPt.y-carPose_pos.y)*(odom_path_wayPt.y-carPose_pos.y));
                if(dist<min_dist){
                    min_dist=dist;
                    min_i = i;
                    forwardPt.point = odom_path_wayPt;
                    forwardPose = odom_path_pose.pose;
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        i = i + 4;
    }
    foundForwardPt = true;
}else if (goal_reached) {
    forwardPt.point = odom_goal_pos;
    foundForwardPt = false;
    // ROS_INFO("goal REACHED!");
  }

 /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt && !goal_reached) {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt.point);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt.point);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.point.x - carPose_pos.x) +
                        sin(carPose_yaw) * (forwardPt.point.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.point.x - carPose_pos.x) +
                        cos(carPose_yaw) * (forwardPt.point.y - carPose_pos.y);
//   endTime = clock();
  ROS_INFO("point=(%.2f,%.2f)",odom_car2WayPtVec.x,odom_car2WayPtVec.y);
  return forwardPose;
};

