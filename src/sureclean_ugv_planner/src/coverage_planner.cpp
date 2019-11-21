#include <cmath>
#include <sureclean_ugv_planner/coverage_planner.h>
#include <sureclean_utils/controller_utils.h>
#include <sureclean_utils/goal_ui_utils.h>
#include <sureclean_utils/planner_utils.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

CoveragePlanner::CoveragePlanner(ros::NodeHandle &privateNH,
                                 ros::NodeHandle &publicNH)
    : privateNH_{privateNH}, publicNH_{publicNH} {
  // Get parameters from Param server
  privateNH_.param("coverage_side_meters", coverageSideMeters_, 1.0);
  privateNH_.param("pick_up_width_meters", pickUpWidthMeters_, 0.7);
  privateNH_.param<std::string>("navigation_frame", navigationFrame_, "map");
  pubCoveragePath_ = publicNH_.advertise<nav_msgs::Path>("goal/path", 1);
  subGoalPose_ = publicNH_.subscribe(
      "goal/pose", 1000, &CoveragePlanner::createCoverageWaypointsCallback,
      this);
  subVehicleOdometry_ =
      publicNH_.subscribe("filtered/gps/odometry", 1,
                          &CoveragePlanner::updateVehiclePoseCallback, this);

  numberOfCleanupPasses_ =
      static_cast<size_t>(std::ceil(coverageSideMeters_ / pickUpWidthMeters_));
  createGenericWaypoints();
}

CoveragePlanner::~CoveragePlanner() = default;

void CoveragePlanner::createGenericWaypoints() {
  eigenWaypoints_.clear();
  eigenWaypoints_.reserve(numberOfCleanupPasses_ * 5);
  double y = coverageSideMeters_;
  double x = -0.5 * (pickUpWidthMeters_ * numberOfCleanupPasses_);
  eigenWaypoints_.emplace_back(-y / 2, 0, 1);
  eigenWaypoints_.emplace_back(y / 2, 0, 1);
  for (auto i = 0; i < std::floor(numberOfCleanupPasses_ / 2); i++) {
    // Ensure proper coverage order
    eigenWaypoints_.emplace_back(y / 2, x + pickUpWidthMeters_ * i, 1);
    eigenWaypoints_.emplace_back(-y / 2, x + pickUpWidthMeters_ * i, 1);
    eigenWaypoints_.emplace_back(-y / 2, -x - pickUpWidthMeters_ * i, 1);
    eigenWaypoints_.emplace_back(y / 2, -x - pickUpWidthMeters_ * i, 1);
  }
}

void CoveragePlanner::createCoverageWaypointsCallback(
    const geometry_msgs::PoseStamped &originalGoal) const {
  geometry_msgs::PoseStamped waypointPose;
  nav_msgs::Path path;
  path.header.frame_id = navigationFrame_;
  waypointPose.header = originalGoal.header;
  Eigen::Vector3d tempPoint;
  double goalYaw = sureclean::calculateDesiredYawFromPoses(vehiclePose_.pose,
                                                           originalGoal.pose);
  Eigen::Matrix3d transform;
  sureclean::createTransform2D(transform, goalYaw, originalGoal.pose.position.x,
                               originalGoal.pose.position.y);
  for (const auto &eigenWaypoint : eigenWaypoints_) {
    tempPoint = transform * eigenWaypoint;
    waypointPose.pose.position.x = tempPoint(0);
    waypointPose.pose.position.y = tempPoint(1);
    path.poses.push_back(waypointPose);
  }
  pubCoveragePath_.publish(path);
}

void CoveragePlanner::updateVehiclePoseCallback(
    const nav_msgs::Odometry &odom) {
  vehiclePose_.header = odom.header;
  vehiclePose_.pose = odom.pose.pose;
}