#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sureclean_utils/LitterGoal.h>
#include <sureclean_utils/controller_utils.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
  // This function is passed two arguments, a distance in meters and a yaw in
  // degrees and converts it to
  // a command in the robot's current odometry frame
  if (argc != 3) {
    ROS_ERROR_STREAM("Need a distance and yaw");
    return -1;
  }

  ros::init(argc, argv, "test_movement");
  ros::NodeHandle n;
  ros::Publisher pubGoal =
      n.advertise<nav_msgs::Path>("/sureclean/ugv_1/goal/path", 1);

  // Convert arguments to float
  float distance = atof(argv[1]);
  float deltaYaw = atof(argv[2]);
  deltaYaw = deltaYaw * M_PI / 180; // Convert to radians

  // Get current odometry of the robot
  nav_msgs::Odometry startOdom =
      *(ros::topic::waitForMessage<nav_msgs::Odometry>(
          "/sureclean/ugv_1/filtered/gps/odometry", n));

  float currYaw = sureclean::calculateYawFromQuaterion(startOdom.pose.pose);
  float goalYaw = currYaw + deltaYaw;
  // Rotate Distance to robot's current rotaton
  float deltaX = distance * std::cos(goalYaw);
  float deltaY = distance * std::sin(goalYaw);
  // Add rotated distance to robot's current position
  startOdom.pose.pose.position.x += deltaX;
  startOdom.pose.pose.position.y += deltaY;

  // Set goal pose
  geometry_msgs::PoseStamped goalPose;
  goalPose.pose.position = startOdom.pose.pose.position;
  goalPose.pose.orientation = tf::createQuaternionMsgFromYaw(goalYaw);

  nav_msgs::Path goalPath;
  goalPath.poses.push_back(goalPose);

  // Publish
  pubGoal.publish(goalPath);
  ROS_INFO_STREAM("Goal Published: Rotate " << deltaYaw << " degrees and Move "
                                            << distance << " meters");
  ros::shutdown();

  return 0;
}