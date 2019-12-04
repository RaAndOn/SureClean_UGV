#include <angles/angles.h>
#include <cmath>

#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>

namespace sureclean {
int signNum(const double &num)
// Return the sign of a number
{
  return (num < 0.0) ? -1 : 1;
}

double calculateDistance(const geometry_msgs::Pose &currPose,
                         const geometry_msgs::Pose &goalPose)
// This function calculates the linear distance between two points
{
  double xErr = currPose.position.x - goalPose.position.x;
  double yErr = currPose.position.y - goalPose.position.y;
  return sqrt(pow(xErr, 2) + pow(yErr, 2));
}

double calculateYawFromQuaterion(const geometry_msgs::Pose &pose)
// Calculate and return the yaw of a pose from a quaternion
{
  double roll, pitch, yaw;
  double quatx = pose.orientation.x;
  double quaty = pose.orientation.y;
  double quatz = pose.orientation.z;
  double quatw = pose.orientation.w;
  tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 rotMatrix(quaternion);
  rotMatrix.getRPY(roll, pitch, yaw);
  return yaw;
}

double normalizeAngleDiff(const double &currAngle, const double &goalAngle)
// This function normalizes the difference between two angles to account for
// looping
{
  double diff = goalAngle - currAngle;
  if (diff > M_PI) {
    return diff - 2 * M_PI;
  }
  if (diff < -M_PI) {
    return diff + 2 * M_PI;
  }
  return diff;
}

double calculateDesiredYawFromPoses(const geometry_msgs::Pose &currPose,
                                    const geometry_msgs::Pose &goalPose) {
  double dyGoal = goalPose.position.y - currPose.position.y;
  double dxGoal = goalPose.position.x - currPose.position.x;
  return atan2(dyGoal, dxGoal);
}

double calculateDeltaYawFromPositions(const geometry_msgs::Pose &currPose,
                                      const geometry_msgs::Pose &goalPose)
// This function determines the error between the current yaw and the desired
// yaw
{
  double yawCurr =
      calculateYawFromQuaterion(currPose); // Get current yaw in odom frame
  double yawGoal = calculateDesiredYawFromPoses(currPose, goalPose);
  return normalizeAngleDiff(yawCurr, yawGoal);
}
} // namespace sureclean