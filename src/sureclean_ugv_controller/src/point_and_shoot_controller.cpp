#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sureclean_ugv_controller/point_and_shoot_controller.h>
#include <sureclean_utils/controller_utils.h>
#include <sureclean_utils/goal_ui_utils.h>

// Point x -- east, y -- north and z -- yaw (up)
PointAndShootController::PointAndShootController(ros::NodeHandle &privateNH,
                                                 ros::NodeHandle &publicNH)
    : privateNH_{privateNH}, publicNH_{publicNH} {
  // Get parameters from Param server
  privateNH_.param("kp_angular", kpAngular_, 1.0);
  privateNH_.param("min_angular_velocity_command", minAngularVelocityCommand_,
                   0.15);
  privateNH_.param("max_angular_velocity_command", maxAngularVelocityCommand_,
                   0.3);
  privateNH_.param("min_linear_velocity_command", minLinearVelocityCommand_,
                   0.4);
  privateNH_.param("max_linear_velocity_command", maxLinearVelocityCommand_,
                   0.75);
  privateNH_.param("final_approach_range", finalApproachRange_, 2.0);
  privateNH_.param("angular_threshold_begin_movement",
                   angularThresholdToBeginMovement_, 0.04);
  privateNH_.param("angular_threshold_continue_movement",
                   angularThresholdToContinueMovement_, 0.13);
  privateNH_.param("linear_threshold_to_achieve_goal",
                   linearThresholdToAchieveGoal_, 0.1);
  privateNH_.param<std::string>("robot_frame", robotFrame_, "base_link");
  privateNH_.param<std::string>("world_frame", worldFrame_, "map");
  // Set publishers and subscribers
  // Publisher: the controller output
  pubCommand_ = publicNH_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // Publisher: whether goal has been achieved
  pubPathStatus_ = publicNH_.advertise<nav_msgs::Path>("goal_path_status", 1);
  pubGoalMarkers_ = publicNH_.advertise<visualization_msgs::Marker>(
      "markers/goal/coverage_path", 1);
  // Subscriber: get current odometry
  subVehicleOdometry_ = publicNH_.subscribe(
      "filtered/gps/odometry", 1,
      &PointAndShootController::updateRobotPoseCallback, this);
  // Subscriber: sets odometry goal
  subGoalPath_ = publicNH_.subscribe(
      "goal/path", 1000, &PointAndShootController::setGoalCallback, this);
  // Service: sets flag to stop movement
  serviceEmergencyStop_ = publicNH_.advertiseService(
      "emergency_stop", &PointAndShootController::emergencyStop, this);
  // Service: sets flag to continue movement
  serviceContinueMovement_ = publicNH_.advertiseService(
      "continue_mission", &PointAndShootController::continueMove, this);

  // Initialize loop terms
  setLoopTerms(nav_msgs::Path{});
  pubPathStatus_.publish(goalPath_);
}

PointAndShootController::~PointAndShootController() = default;

void PointAndShootController::setGoalCallback(
    const nav_msgs::Path &newGoalPath) {
  if (goalPath_.poses.empty()) {
    setLoopTerms(newGoalPath);
    pubPathStatus_.publish(goalPath_);
    ROS_INFO_STREAM("-------New Goal Path-------");
  } else {
    ROS_WARN_STREAM(
        "Attempted to publish goal path, while goal is still active");
  }
}

void PointAndShootController::updateRobotPoseCallback(
    const nav_msgs::Odometry &odom) {
  std::lock_guard<std::mutex> lock(robotPoseMutex_);
  robotPose_ = odom.pose.pose;
}

void PointAndShootController::movementController() {
  geometry_msgs::Twist command;
  double angularError{0};
  double positionError{0};
  if (not goalPath_.poses.empty() and robotPose_) {
    const auto goalPose = goalPath_.poses.front().pose;
    // Protect robotPose_ while calculating the error
    std::lock_guard<std::mutex> lock(robotPoseMutex_);
    angularError =
        sureclean::calculateDeltaYawFromPositions(robotPose_.get(), goalPose);
    positionError = sureclean::calculateDistance(robotPose_.get(), goalPose);
    // Release robotPose_ when done calculating error
    robotPoseMutex_.unlock();
  }
  if (moveSignal_ && not goalAchievedCheck(positionError)) {
    if (onFinalApproach_) {
      command.linear.x = linearVelocityController(positionError);
    }
    // If within range, perform final yaw correction before final approach
    else if (positionError <= finalApproachRange_ and not onFinalApproach_) {
      command.angular.z = angularVelocityController(angularError);
      if (fabs(angularError) < angularThresholdToBeginMovement_) {
        ROS_INFO_STREAM("---------- FINAL APPROACH ----------");
        onFinalApproach_ = true;
      }
    } else {
      command.angular.z = angularVelocityController(angularError);
      // Once the robot has achieved angularThresholdToBeginMovement_, you can
      // move. Only stop if it drops bellow angularThresholdToContinueMovement_
      if (fabs(angularError) < angularThresholdToContinueMovement_ and
          rotationComplete_) {
        command.linear.x = linearVelocityController(positionError);
      }
    }
    pubCommand_.publish(command);
  }
}

double
PointAndShootController::angularVelocityController(const double &angularError) {
  double commandAngular = 0;
  if (fabs(angularError) > angularThresholdToBeginMovement_) {
    commandAngular = kpAngular_ * angularError;
    commandAngular = sureclean::signNum(commandAngular) *
                     std::max(fabs(commandAngular), minAngularVelocityCommand_);
    commandAngular = sureclean::signNum(commandAngular) *
                     std::min(fabs(commandAngular), maxAngularVelocityCommand_);
  } else {
    rotationComplete_ = true;
  }
  return commandAngular;
}

double PointAndShootController::linearVelocityController(
    const double & /*positionError*/) const {
  // The robot only has two speeds, one for collection on final approach and one
  // for traveling between waypoints
  if (onFinalApproach_) {
    return minLinearVelocityCommand_;
  }
  return maxLinearVelocityCommand_;
}

bool PointAndShootController::checkForOvershoot(
    const double & /*positionError*/) const {
  // TODO(bertha): Confirm this is a good implamentation. A bad connection could
  // cause
  // the transform to fail repeatedly and stop the robot
  if (onFinalApproach_) {
    try {
      // Transform the goal pose from the world frame to the robot frame
      tf::StampedTransform transform;
      ros::Time timeNow{ros::Time(0)};
      tfListener_.lookupTransform(robotFrame_, worldFrame_, timeNow, transform);

      geometry_msgs::PointStamped goalPoint;
      goalPoint.point = goalPath_.poses.front().pose.position;
      goalPoint.header.frame_id = worldFrame_;
      goalPoint.header.stamp = timeNow;
      geometry_msgs::PointStamped goalPointInRobotFrame;
      tfListener_.transformPoint(robotFrame_, timeNow, goalPoint, worldFrame_,
                                 goalPointInRobotFrame);

      // The goal has been overshot if it is behind the robot in the robot frame
      return (goalPointInRobotFrame.point.x < 0);
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM(ex.what());
    }
  }
  return false;
}

bool PointAndShootController::goalAchievedCheck(const double &positionError) {
  const auto goalOvershoot{checkForOvershoot(positionError)};
  const auto goalAchieved{positionError < linearThresholdToAchieveGoal_};
  if (not goalOvershoot and not goalAchieved) {
    return false;
  }
  if (goalAchieved) {
    ROS_INFO_STREAM("---------- Goal Achieved --------");
  }
  if (goalOvershoot) {
    ROS_INFO_STREAM("---------- Over Shot Goal, Skipping --------");
  }
  pubPathStatus_.publish(goalPath_);
  goalPath_.poses.erase(goalPath_.poses.begin());
  setLoopTerms(goalPath_);
  return true;
}

bool PointAndShootController::setLoopTerms(const nav_msgs::Path &newGoalPath) {
  rotationComplete_ = false;
  onFinalApproach_ = false;
  if (not newGoalPath.poses.empty()) {
    goalPath_ = newGoalPath;
    moveSignal_ = true;
  } else {
    moveSignal_ = false;
    pubPathStatus_.publish(goalPath_);
  }
  visualization_msgs::Marker goalMarkers;
  sureclean::createGoalMarkers(goalPath_, worldFrame_, goalMarkers,
                               sureclean::Color{0.0f, 1.0f, 0.0f});
  pubGoalMarkers_.publish(goalMarkers);
}

bool PointAndShootController::emergencyStop(
    std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/) {
  moveSignal_ = false;
  return true;
}

bool PointAndShootController::continueMove(
    std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/) {
  moveSignal_ = true;
  return true;
}
