#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



float calculateYaw(geometry_msgs::Pose pose)
{
  double roll, pitch, yaw;
  double quatx = pose.orientation.x;
  double quaty = pose.orientation.y;
  double quatz = pose.orientation.z;
  double quatw = pose.orientation.w;
  tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 rotMatrix(quaternion);
  rotMatrix.getRPY(roll, pitch, yaw);
  return yaw;
}


int main(int argc, char** argv) {

  //Check that a urdf file path has been pased in
  if (argc != 3){
    ROS_ERROR("Need a distance and yaw");
    return -1;
  }

  ros::init(argc, argv, "test_movement");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("goal", 1000);

  float distance = atof(argv[1]);
  float goalYaw = atof(argv[2]);
  goalYaw = goalYaw * M_PI/180;

  geometry_msgs::Pose pose;
  tf::StampedTransform transform;

  nav_msgs::Odometry startOdom = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odometry/filtered", n));


  float yaw = calculateYaw(startOdom.pose.pose);

  float distanceX = distance*cos(yaw);
  float distanceY = distance*sin(yaw);
  startOdom.pose.pose.position.x += distanceX;
  startOdom.pose.pose.position.y += distanceY;

  pose.position = startOdom.pose.pose.position;
  pose.orientation = tf::createQuaternionMsgFromYaw(goalYaw+yaw);
  pub.publish(pose);

  ROS_INFO("Goal Published");


  


  ros::shutdown();

  return 0;
}