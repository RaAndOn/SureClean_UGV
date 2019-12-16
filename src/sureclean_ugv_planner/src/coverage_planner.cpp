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
  privateNH_.param("coverage_side_meters", defaultCoverageSideMeters_, 1.0);
  privateNH_.param("pick_up_width_meters", pickUpWidthMeters_, 0.7);
  privateNH_.param<std::string>("navigation_frame", navigationFrame_, "map");
  pubCoveragePath_ = publicNH_.advertise<nav_msgs::Path>("goal/path", 1);
  subGoalPose_ = publicNH_.subscribe(
      "goal/pose", 1000, &CoveragePlanner::createCoverageWaypointsCallback,
      this);
  subVehicleOdometry_ =
      publicNH_.subscribe("filtered/gps/odometry", 1,
                          &CoveragePlanner::updateVehiclePoseCallback, this);

  createGenericWaypoints(defaultCoverageSideMeters_);
}

CoveragePlanner::~CoveragePlanner() = default;

void CoveragePlanner::createGenericWaypoints(const double coverageSideMeters) {
  const auto numberOfCleanupPasses = static_cast<size_t>(
      std::ceil(coverageSideMeters / pickUpWidthMeters_) + 1);
  eigenWaypoints_.clear();
  eigenWaypoints_.reserve(numberOfCleanupPasses * 5);
  double y = (numberOfCleanupPasses * pickUpWidthMeters_) / 2 + 0.5;
  double x = -0.5 * (pickUpWidthMeters_ * numberOfCleanupPasses);
  eigenWaypoints_.emplace_back(-y, 0, 1);
  eigenWaypoints_.emplace_back(y, 0, 1);
  for (auto i = 0; i < std::floor(numberOfCleanupPasses / 2); i++) {
    // Ensure proper coverage order
    eigenWaypoints_.emplace_back(y, x + pickUpWidthMeters_ * i, 1);
    eigenWaypoints_.emplace_back(-y, x + pickUpWidthMeters_ * i, 1);
    eigenWaypoints_.emplace_back(-y, -x - pickUpWidthMeters_ * i, 1);
    eigenWaypoints_.emplace_back(y, -x - pickUpWidthMeters_ * i, 1);
  }
}

void CoveragePlanner::createCoverageWaypointsCallback(
    const sureclean_utils::LitterGoal &originalGoal) {
  geometry_msgs::PoseStamped waypointPose;
  nav_msgs::Path path;
  path.header.frame_id = navigationFrame_;
  waypointPose.header = originalGoal.header;
  Eigen::Vector3d tempPoint;
  double goalYaw = sureclean::calculateDesiredYawFromPoses(
      vehiclePose_.pose, originalGoal.point.pose);
  Eigen::Matrix3d transform;
  sureclean::createTransform2D(transform, goalYaw,
                               originalGoal.point.pose.position.x,
                               originalGoal.point.pose.position.y);
  if (originalGoal.coverageSide < 0.0) {
    createGenericWaypoints(defaultCoverageSideMeters_);
  }
  if (originalGoal.coverageSide > 0.0) {
    createGenericWaypoints(originalGoal.coverageSide);
  }
  if (originalGoal.coverageSide == 0.0) {
    eigenWaypoints_.clear();
    eigenWaypoints_.emplace_back(0, 0, 1);
  }
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