#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265358979
double last_d_theta = 0;

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
    double getCar2GoalDist();
    geometry_msgs::Pose  getTrackPose(const geometry_msgs::Pose &carPose);
    double getEta(const geometry_msgs::Pose &carPose);

private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub, path_sub, goal_sub;
  ros::Publisher pub_, marker_pub;
  ros::Timer timer1, timer2;

  visualization_msgs::Marker points, line_strip, goal_circle;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point odom_goal_pos;
  nav_msgs::Odometry odom;
  nav_msgs::Path map_path;

  double controller_freq, baseSpeed;
  double  goalRadius, goal_pose;

  bool foundForwardPt,goal_received, goal_reached;

  void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
  void goalReachingCB(const ros::TimerEvent &);

  void controlLoopCB(const ros::TimerEvent &);

};

UpperController::UpperController() {
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Controller parameter
  pn.param("controller_freq", controller_freq, 20.0);
  pn.param("baseSpeed", baseSpeed, 0.0);
  pn.param("goalRadius", goalRadius, 1.0);

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
  ROS_INFO("[param] baseSpeed: %.2f", baseSpeed);
  ROS_INFO("[param] controller_freq: %.2f", controller_freq);
  ROS_INFO("[param] goalRadius: %.2f", goalRadius);

  // Visualization Marker Settings
  initMarker();
}

void UpperController::initMarker(){
    points.header.frame_id = line_strip.header.frame_id =
        goal_circle.header.frame_id = "odom";
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
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

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

geometry_msgs::Pose UpperController::getTrackPose(const geometry_msgs::Pose &carPose){
double carPose_yaw = getYawFromPose(carPose);
geometry_msgs::Point forwardPt;
geometry_msgs::Point odom_car2WayPtVec;
geometry_msgs::Point carPose_pos = carPose.position;
geometry_msgs::Pose forwardPose;


if (!goal_reached) {
    for (int i = 0; i < map_path.poses.size(); i++) {
      geometry_msgs::Point odom_path_wayPt = map_path.poses[i].pose.position;
      bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);
      
      if (_isForwardWayPt) {
              forwardPt = odom_path_wayPt;
              forwardPose = map_path.poses[i].pose;
              foundForwardPt = true;
              break;
        }
    }     
}else if (goal_reached) {
    forwardPt = odom_goal_pos;
    foundForwardPt = false;
    // ROS_INFO("goal REACHED!");
  }
 
 
 
 /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt && !goal_reached) {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  // return odom_car2WayPtVec;
  return forwardPose;
}


// double UpperController::getEta(const geometry_msgs::Pose &carPose) {
//   geometry_msgs::Point odom_car2WayPtVec = getTrackPt(carPose);

//   double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
//   return eta;
// };

void UpperController::controlLoopCB(const ros::TimerEvent &) {

  geometry_msgs::Pose carPose = odom.pose.pose;
  geometry_msgs::Twist carVel = odom.twist.twist;
  geometry_msgs::Pose ForwardPose = getTrackPose(carPose);
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;

  if (goal_received) {
        double thetar = getYawFromPose(carPose);
        double theta = getYawFromPose(ForwardPose);
        double d_theta = theta - thetar;
        if (foundForwardPt) {
          cmd_vel.angular.z =d_theta;//PD control here!! no finish

        last_d_theta = d_theta;
        if (!goal_reached) {
            cmd_vel.linear.x =0.5;
        }
  }
  ROS_INFO("Vyaw:%.2f,Vt:%.2f",cmd_vel.angular.z,cmd_vel.linear.x);
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