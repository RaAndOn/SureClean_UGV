#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <sureclean_utils/goal_ui_utils.h>
#include <tf/transform_listener.h>

namespace sureclean {
geometry_msgs::PointStamped latitudeLongitudeToUTM(const double &latitude,
                                                   const double &longitude) {
  double utmX = 0, utmY = 0;
  std::string utmZone;
  geometry_msgs::PointStamped utmPointOutput;

  // convert lat/long to utm
  RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, utmY, utmX,
                                                utmZone);

  // Construct UTM_point and map_point geometry messages
  utmPointOutput.header.frame_id = "utm";
  utmPointOutput.header.stamp = ros::Time(0);
  utmPointOutput.point.x = utmX;
  utmPointOutput.point.y = utmY;
  utmPointOutput.point.z = 0;

  return utmPointOutput;
}

geometry_msgs::PointStamped
transformPointToFrame(const geometry_msgs::PointStamped &point,
                      const std::string &targetFrame) {
  geometry_msgs::PointStamped transformedPoint;
  const std::string initialFrame{point.header.frame_id};
  bool notDone = true;
  tf::TransformListener
      listener; // create transformlistener object called listener
  ros::Time timeNow = ros::Time::now();

  while (notDone) {
    try {
      // Transform UTM point to the odometry frame
      listener.waitForTransform(targetFrame, initialFrame, timeNow,
                                ros::Duration(3.0));
      listener.transformPoint(targetFrame, point, transformedPoint);
      notDone = false;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();
    }
  }
  return transformedPoint;
}

void createGoalMarkers(const nav_msgs::Path &goalList,
                       const std::string &frameID,
                       visualization_msgs::Marker &goalMarkers,
                       boost::optional<Color> color) {
  goalMarkers.header.frame_id = frameID;
  goalMarkers.header.stamp = ros::Time::now();
  goalMarkers.ns = "spheres";
  goalMarkers.action = visualization_msgs::Marker::ADD;
  goalMarkers.pose.orientation.w = 1.0;

  goalMarkers.id = 0;

  goalMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
  // POINTS markers use x and y scale for width/height respectively
  goalMarkers.scale.x = 0.1;
  goalMarkers.scale.y = 0.1;
  goalMarkers.scale.z = 0.1;

  goalMarkers.color.a = 1.0;
  if (color) {
    goalMarkers.color.r = color->red;
    goalMarkers.color.g = color->green;
    goalMarkers.color.b = color->blue;
  } else {
    // Default color to red
    goalMarkers.color.r = 1.0f;
    goalMarkers.color.g = 0.0f;
    goalMarkers.color.b = 0.0f;
  }
  for (const auto &goal : goalList.poses) {
    goalMarkers.points.push_back(goal.pose.position);
  }
}
} // namespace sureclean