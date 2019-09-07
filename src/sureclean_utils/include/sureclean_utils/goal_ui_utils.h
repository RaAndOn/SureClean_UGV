#ifndef GOAL_UI_UTILS_H
#define GOAL_UI_UTILS_H
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

namespace sureclean {

struct Color {
  float red;
  float green;
  float blue;
};

/// @brief Convert latitude and longitude to stamped point in UTM coordinates
/// @param latitude - latitude coordinate
/// @param longitude - latitude longitude
/// @return latitude and longitude in UTM coordinates
geometry_msgs::PointStamped latitudeLongitudeToUTM(const double &latitude,
                                                   const double &longitude);

/// @brief Transform point into a different frame
/// @param point - Point to be transformed to a new frame
/// @param targetFrame - The new frame to transform the point into
/// @return The transformed point in its new frame
geometry_msgs::PointStamped
transformPointToFrame(const geometry_msgs::PointStamped &point,
                      const std::string &targetFrame);

/// @brief Turn a path into a list of markers for visualization
/// @param goalList - path which we wish to visualize
/// @param frameID - Frame in which to publish the goal
/// @param goalMarkers - Visualization msg to fill with markers
void createGoalMarkers(const nav_msgs::Path &goalList,
                        const std::string &frameID,
                        visualization_msgs::Marker &goalMarkers,
                        boost::optional<Color> color = boost::none);
}

#endif