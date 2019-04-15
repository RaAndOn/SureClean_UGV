#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



float calculateYaw(geometry_msgs::Pose pose)
// Calculate and return the yaw of a pose quaternion
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
  // This function is passed two arguments, a distance in meters and a yaw in degrees and converts it to
  // a command in the robot's current odometry frame
  if (argc != 3){
    ROS_ERROR("Need a distance and yaw");
    return -1;
  }

  ros::init(argc, argv, "test_movement");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("goal", 1000);

  // Convert arguments to float
  float distance = atof(argv[1]);
  float deltaYaw = atof(argv[2]);
  deltaYaw = deltaYaw * M_PI/180; // Convert to radians


  // Get current odometry of the robot
  nav_msgs::Odometry startOdom = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odometry/filtered", n));

  float currYaw = calculateYaw(startOdom.pose.pose);
  float goalYaw = currYaw + deltaYaw;
  // Rotate Distance to robot's current rotaton
  float deltaX = distance*cos(goalYaw);
  float deltaY = distance*sin(goalYaw);
  // Add rotated distance to robot's current position
  startOdom.pose.pose.position.x += deltaX;
  startOdom.pose.pose.position.y += deltaY;

  // Set goal pose
  geometry_msgs::Pose goalPose;
  goalPose.position = startOdom.pose.pose.position;
  goalPose.orientation = tf::createQuaternionMsgFromYaw(goalYaw);

  // Publish
  pub.publish(goalPose);
  ROS_INFO("Goal Published:");
  ROS_INFO("Rotate %f degrees", deltaYaw);
  ROS_INFO("Move %f meters", distance);
  ros::shutdown();

  return 0;
}