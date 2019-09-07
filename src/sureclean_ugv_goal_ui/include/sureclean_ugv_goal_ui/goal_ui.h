#ifndef GOAL_UI_H
#define GOAL_UI_H

#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

class GoalUI {
public:
  GoalUI(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~GoalUI();

private:
  ros::NodeHandle privateNH_;
  ros::NodeHandle publicNH_;
  ros::Publisher pubGoalToController_;
  ros::Publisher pubGoalToCoveragePlanner_;
  ros::Publisher pubGoalMarker_;
  ros::Subscriber subGPS_;
  ros::Subscriber subOdom_;
  ros::Subscriber subGoalStatus_;
  ros::ServiceServer serviceCollectGoalGPS_;
  ros::ServiceServer serviceCollectGoalOdom_;
  ros::ServiceServer serviceGoToGoal_;

  // Vector of goal locations
  nav_msgs::Path goalList_;

  // Current GPS Coordinates
  sensor_msgs::NavSatFix currGPS_;

  // Mutex to prevent callback and service from accessing current gps
  // simultaneously
  std::mutex gpsMutex_;

  // Current odometry Coordinates
  nav_msgs::Odometry currOdom_;

  // Mutex to prevent callback and service from accessing current odom
  // simultaneously
  std::mutex odomMutex_;

  // Flag to determine if navigation to all goals will happen automatically
  bool autonomous_{};

  bool useCoverage_{};

  // Frame in which navigation happens
  std::string navigationFrame_;

  /// @brief Callback to update robot state once a goal has been achieved
  /// @param empty message, topic is only published to when goal is achieved
  void goalAchievedCallback(const nav_msgs::Path &path);

  /// @brief Update the current GPS variable (currGPS_) with latest from topic
  /// @param - New GPS coordinate
  void updateCurrentGPSCallback(const sensor_msgs::NavSatFix &gps);

  /// @brief Update the current Odom variable (currOdom_) with latest from topic
  /// @param - New Odom pose
  void updateCurrentOdomCallback(const nav_msgs::Odometry &odom);

  /// @brief Take current GPS coordinates of Robot and add to goal vector
  bool collectGoalGPSService(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res);

  /// @brief Take current fused odometry of Robot and add to goal vector
  bool collectGoalOdomService(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res);

  /// @brief Service to command robot to move to next goal in vector
  bool nextGoalService(std_srvs::Empty::Request &req,
                       std_srvs::Empty::Response &res);

  /// @brief This function passes waypoints to the controller
  /// @return bool indicating if a goal was successfully published
  bool moveToNextGoal();
};

#endif