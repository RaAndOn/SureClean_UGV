#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sureclean_utils/LitterGoal.h>

class CoveragePlanner {
public:
  CoveragePlanner(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH);

  ~CoveragePlanner();

private:
  ros::NodeHandle &privateNH_;
  ros::NodeHandle &publicNH_;
  // Subscriber: receives points around which to plan coverage path
  ros::Subscriber subGoalPose_;
  // Subscriber: updates the vehicle pose
  ros::Subscriber subVehicleOdometry_;
  // Publisher: coverage path
  ros::Publisher pubCoveragePath_;
  double defaultCoverageSideMeters_{};
  double pickUpWidthMeters_{};
  std::string navigationFrame_;
  geometry_msgs::PoseStamped vehiclePose_;
  std::vector<Eigen::Vector3d> eigenWaypoints_;

  /// @brief Takes in a goal waypoint and creates a series of waypoints to cover
  /// the area surrounding the goal
  /// @param Original goal waypoint, giving best estimate of litter location
  void createCoverageWaypointsCallback(
      const sureclean_utils::LitterGoal &originalGoal);

  /// @brief Callback to update the vehicles odometry to whatever was most
  /// recently published
  /// @param Most recently published vehicle odometry
  void updateVehiclePoseCallback(const nav_msgs::Odometry &odom);

  /// @brief Populate eigenWaypoints_ with generic points representing the
  /// location of the waypoints in the litter frame
  void createGenericWaypoints(const double coverageSideMeters);
};

#endif