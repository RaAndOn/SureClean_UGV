#ifndef POINT_AND_SHOOT_CONTROLLER_H
#define POINT_AND_SHOOT_CONTROLLER_H

#include <boost/optional.hpp>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

class PointAndShootController {
public:
  PointAndShootController(ros::NodeHandle &privateNH,
                          ros::NodeHandle &publicNH);

  ~PointAndShootController();

  /// @brief Sets the command velocities for the robot to reach the goal
  ///        It prioritizes driving the front of the robot directly over the
  ///        goal
  void movementController();

private:
  // Proportional gains
  double kpAngular_{};

  bool moveSignal_{};
  bool onFinalApproach_{};
  nav_msgs::Path goalPath_;
  boost::optional<geometry_msgs::Pose> robotPose_;

  std::mutex robotPoseMutex_;

  bool rotationComplete_{};
  // Angular min and max commands
  double minAngularVelocityCommand_{};
  double maxAngularVelocityCommand_{};
  // Linear min and max commands
  double minLinearVelocityCommand_{};
  double maxLinearVelocityCommand_{};
  // Range at which to do final adjustment - meters
  double finalApproachRange_{};
  // Angular convergence threshold to continue moving- radians
  double angularThresholdToContinueMovement_{};
  // Angular convergence threshold to begin moving - radians
  double angularThresholdToBeginMovement_{};
  // Linear convergence threshold to achieve goal- meters
  double linearThresholdToAchieveGoal_{};

  // Name of frame attached to the robot
  std::string robotFrame_;
  // Name of frame in which the robot navigates
  std::string worldFrame_;

  tf::TransformListener tfListener_;
  ros::Publisher pubCommand_;
  ros::Publisher pubPathStatus_;
  ros::Publisher pubGoalMarkers_;
  ros::Subscriber subVehicleOdometry_;
  ros::Subscriber subGoalPath_;
  ros::ServiceServer serviceEmergencyStop_;
  ros::ServiceServer serviceContinueMovement_;
  ros::NodeHandle privateNH_;
  ros::NodeHandle publicNH_;

  /// @brief Callback which sets a new goal
  /// @param the odometry nav_msg of the new goal
  void setGoalCallback(const nav_msgs::Path &newGoalPath);

  /// @brief This function updates the robots pose based on the lates odometry
  /// message
  /// @param Latest robot odometry
  void updateRobotPoseCallback(const nav_msgs::Odometry &odom);

  /// @brief This function returns the angular command for the robot
  /// @param Current error of the robot's angle
  double angularVelocityController(const double &angularError);

  /// @brief This function returns the linear command for the robot
  /// @param Current error of the robot's linear position
  double linearVelocityController(const double &positionError) const;

  /// @brief This function determines whether the robot has reachced its goal
  /// @param Current error of the robot's position
  /// @return Whether the robot has reached the goal
  bool goalAchievedCheck(const double &positionError);

  /// @brief This function checks whether the robot has overshot the its goal
  /// @param Current error of the robot's position
  bool checkForOvershoot(const double &positionError) const;

  /// @brief This function sets/resets the flags associated with the contorl
  /// loop based on if it is passed a new goal
  /// @param new goal which indicates whether this is the end of a loop or a new
  /// one
  bool setLoopTerms(const nav_msgs::Path &newGoalPath);

  /// @brief This service will stop the robot
  bool emergencyStop(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);

  /// @brief This service will undo the emergency stop
  bool continueMove(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res);
};

#endif