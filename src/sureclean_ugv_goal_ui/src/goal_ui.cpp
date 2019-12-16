#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <robot_localization/navsat_conversions.h>
#include <sureclean_ugv_goal_ui/goal_ui.h>
#include <sureclean_utils/goal_ui_utils.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

GoalUI::GoalUI(ros::NodeHandle &privateNH, ros::NodeHandle &publicNH)
    : privateNH_{privateNH}, publicNH_{publicNH} {
  privateNH_.param("autonomous", autonomous_, true);
  privateNH_.param("coverage", useCoverage_, true);
  publicNH_.param<std::string>("navigation_frame", navigationFrame_, "map");
  // Publisher: sends goal pose to coverage planner
  pubGoalToCoveragePlanner_ =
      publicNH_.advertise<sureclean_utils::LitterGoal>("goal/pose", 1);
  // Publisher: Create markers for all goals
  pubGoalMarker_ =
      publicNH_.advertise<visualization_msgs::Marker>("markers/goal/raw", 1);
  // Subscriber: says if goal has been achieved
  subGoalStatus_ = publicNH_.subscribe("goal_path_status", 1,
                                       &GoalUI::goalAchievedCallback, this);
  // Subscriber: gets latest GPS coordinates
  subGPS_ = publicNH_.subscribe("rtk_gps", 1, &GoalUI::updateCurrentGPSCallback,
                                this);
  // Subscriber: gets latest fused odometry in the navigation frame
  subOdom_ = publicNH_.subscribe("filtered/gps/odometry", 1,
                                 &GoalUI::updateCurrentOdomCallback, this);
  // Service: converts current GPS coordinates to navigation frame
  serviceCollectGoalGPS_ = publicNH_.advertiseService(
      "collect_goal_gps", &GoalUI::collectGoalGPSService, this);
  // Service: takes current fused odometry in navigation frame as goal
  serviceCollectGoalOdom_ = publicNH_.advertiseService(
      "collect_goal_odom", &GoalUI::collectGoalOdomService, this);
  // Service: begins navigation to goal
  serviceGoToGoal_ = publicNH_.advertiseService("go_to_next_goal",
                                                &GoalUI::nextGoalService, this);
  serviceServerGPSGoal_ = publicNH_.advertiseService(
      "server_gps_goal", &GoalUI::serverGPSGoal, this);
}

GoalUI::~GoalUI() = default;

void GoalUI::goalAchievedCallback(const nav_msgs::Path &path)
// This function will automatically pass the next waypoint to the controller
{
  if (path.poses.empty() and autonomous_) {
    moveToNextGoal();
  }
}

void GoalUI::updateCurrentGPSCallback(const sensor_msgs::NavSatFix &gps) {
  // This callback updates the currGPS_ variable with the robot's current gps
  // coordinates
  std::lock_guard<std::mutex> gpsLock(gpsMutex_);
  currGPS_ = gps;
}

void GoalUI::updateCurrentOdomCallback(const nav_msgs::Odometry &odom) {
  // This callback updates the currOdom_ variable with the robot's current
  // odometry
  std::lock_guard<std::mutex> odomLock(odomMutex_);
  currOdom_ = odom;
}

bool GoalUI::collectGoalGPSService(std_srvs::Empty::Request & /*req*/,
                                   std_srvs::Empty::Response & /*res*/) {
  std::lock_guard<std::mutex> gpsLock(gpsMutex_);
  const geometry_msgs::PointStamped utmGoal =
      sureclean::latitudeLongitudeToUTM(currGPS_.latitude, currGPS_.longitude);
  // Transform UTM to map point navigation frame
  geometry_msgs::PointStamped goalPoint =
      sureclean::transformPointToFrame(utmGoal, navigationFrame_);
  sureclean_utils::LitterGoal goalPose;
  goalPose.point.pose.position.x = goalPoint.point.x;
  goalPose.point.pose.position.y = goalPoint.point.y;
  if (useCoverage_) { // Indicate use of the default coverage size
    goalPose.coverageSide = -1.0;
  }
  if (not useCoverage_) { // Indicate do not use coverage
    goalPose.coverageSide = 0.0;
  }
  goalList_.push_back(goalPose); // add goal to queue
  visualization_msgs::Marker goalMarkers;
  sureclean::createGoalMarkers(sureclean::litterGoalToPath(goalList_),
                               navigationFrame_, goalMarkers);
  pubGoalMarker_.publish(goalMarkers);
  ROS_INFO_STREAM("---------- Goal collected --------");
  ROS_INFO_STREAM("X: " << goalPose.point.pose.position.x
                        << "; Y: " << goalPose.point.pose.position.y);
  ROS_INFO_STREAM("Latitude: " << currGPS_.latitude);
  ROS_INFO_STREAM("Longitude: " << currGPS_.longitude);
  return true;
}

bool GoalUI::serverGPSGoal(sureclean_ugv_goal_ui::GPSGoal::Request &req,
                           sureclean_ugv_goal_ui::GPSGoal::Response &res) {
  const geometry_msgs::PointStamped utmGoal =
      sureclean::latitudeLongitudeToUTM(req.latitude, req.longitude);
  // Transform UTM to map point navigation frame
  geometry_msgs::PointStamped goalPoint =
      sureclean::transformPointToFrame(utmGoal, navigationFrame_);
  sureclean_utils::LitterGoal goalPose;
  goalPose.point.pose.position.x = goalPoint.point.x;
  goalPose.point.pose.position.y = goalPoint.point.y;
  if (useCoverage_) { // Use the given coverage size
    goalPose.coverageSide = req.coverageSide;
  }
  if (not useCoverage_) { // Indicate do not use coverage
    goalPose.coverageSide = 0.0;
  }
  goalList_.push_back(goalPose); // add goal to queue
  visualization_msgs::Marker goalMarkers;
  sureclean::createGoalMarkers(sureclean::litterGoalToPath(goalList_),
                               navigationFrame_, goalMarkers);
  pubGoalMarker_.publish(goalMarkers);
  ROS_INFO_STREAM("---------- Goal collected --------");
  ROS_INFO_STREAM("X: " << goalPose.point.pose.position.x
                        << "; Y: " << goalPose.point.pose.position.y);
  ROS_INFO_STREAM("Latitude: " << req.latitude);
  ROS_INFO_STREAM("Longitude: " << req.longitude);
  res.odomX = goalPose.point.pose.position.x;
  res.odomY = goalPose.point.pose.position.y;
  res.coverageSide = goalPose.coverageSide;
  return true;
}

bool GoalUI::collectGoalOdomService(std_srvs::Empty::Request & /*req*/,
                                    std_srvs::Empty::Response & /*res*/) {
  std::lock_guard<std::mutex> odomLock(odomMutex_);
  sureclean_utils::LitterGoal goalPose;
  goalPose.point.pose.position.x = currOdom_.pose.pose.position.x;
  goalPose.point.pose.position.y = currOdom_.pose.pose.position.y;
  if (useCoverage_) { // Indicate use of the default coverage size
    goalPose.coverageSide = -1.0;
  }
  if (not useCoverage_) { // Indicate do not use coverage
    goalPose.coverageSide = 0.0;
  }
  goalList_.push_back(goalPose); // add goal to queue
  visualization_msgs::Marker goalMarkers;
  sureclean::createGoalMarkers(sureclean::litterGoalToPath(goalList_),
                               navigationFrame_, goalMarkers);
  pubGoalMarker_.publish(goalMarkers);
  ROS_INFO_STREAM("---------- Goal collected --------");
  ROS_INFO_STREAM("X: " << goalPose.point.pose.position.x
                        << "; Y: " << goalPose.point.pose.position.y);
  return true;
}

bool GoalUI::nextGoalService(std_srvs::Empty::Request & /*req*/,
                             std_srvs::Empty::Response & /*res*/) {
  // This service calls will manually send the next goal to the controller
  return moveToNextGoal();
}

bool GoalUI::moveToNextGoal() {
  // This function passes waypoints to the controller

  visualization_msgs::Marker goalMarkers;
  sureclean::createGoalMarkers(sureclean::litterGoalToPath(goalList_),
                               navigationFrame_, goalMarkers);
  pubGoalMarker_.publish(goalMarkers);
  // Raise error if goal list is empty
  if (goalList_.empty()) {
    ROS_WARN_STREAM("No goal gps in the goal list");
    return false;
  }
  // Remove goal from the front of the queue and publish it
  const auto nextGoal = goalList_.front();
  goalList_.erase(goalList_.begin());
  pubGoalToCoveragePlanner_.publish(nextGoal);
  ROS_INFO_STREAM("---------- Goal sent to planner --------");
  return true;
}