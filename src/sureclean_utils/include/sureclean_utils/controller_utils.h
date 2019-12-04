#ifndef CONTROLLER_UTILS_H
#define CONTROLLER_UTILS_H
#include <geometry_msgs/Pose.h>

namespace sureclean {
/// @brief Return the sign of a number
/// @param number whose sign to evaluate
int signNum(const double &num);

/// @brief This function calculates the linear distance between two points
/// @param the current pose of the robot
/// @param the goal pose of the robot, which we want the distance to
double calculateDistance(const geometry_msgs::Pose &currPose,
                         const geometry_msgs::Pose &goalPose);

/// @brief Calculate and return the yaw of a pose from a quaternion
/// @param pose of the robot, from which we will extract the orientation as a
/// quaternion
double calculateYawFromQuaterion(const geometry_msgs::Pose &pose);

/// @brief This function normalizes the difference between two angles to account
/// for looping
/// @param current angle of the robot
/// @param goal angle of the robot
double normalizeAngleDiff(const double &currAngle, const double &goalAngle);

/// @brief This function calculates the angle which aligns two points
/// @param current pose of the robot
/// @param goal pose of the robot
/// @return Return angle which aligns the robots pose with the goal pose
double calculateDesiredYawFromPoses(const geometry_msgs::Pose &currPose,
                                    const geometry_msgs::Pose &goalPose);

/// @brief This function determines the error between the current yaw and the
/// desired yaw
/// @param current pose of the robot
/// @param goal pose of the robot
/// @param return the difference between the robot's orientation and the desired
/// orientation
double calculateDeltaYawFromPositions(const geometry_msgs::Pose &currPose,
                                      const geometry_msgs::Pose &goalPose);
}

#endif