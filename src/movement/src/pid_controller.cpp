#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <math.h>



class Move
{
  public:
    Move()
    {
      // Gains for linear PID controller
      kpLinear_ = 1.0;
      kdLinear_ = 0.8;
      kiLinear_ = 0.0;

      // Gains for angular PID controller
      kpAngular_ = 1;
      kdAngular_ = .2;
      kiAngular_ = .0;

      // Initialize Error terms
      errLinear_ = 0.0;
      errDiffLinear_ = 0.0;
      prevErrLinear_ = 0;

      errAngular_ = 0.0;
      errDiffAngular_ = 0.0;
      prevErrAngular_ = 0;

      // Initialize loop terms
      noCommandIterations_ = 0;
      activeGoal_ = false;
      rotationComplete_ = false;

      // Min and Max values
      minAngular_ = 0.2;
      maxAngular_ = .5;

      minLinear_ = 0.5;
      maxLinear_ = 1.0;


      // Set publishers and subscribers 
      pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
      // subOdom_ = n_.subscribe("odometry/filtered", 1000, &Move::pointAndShootCallback, this);
      subOdom_ = n_.subscribe("odometry/filtered", 1000, &Move::pidCallback, this);
      subGoal_ = n_.subscribe("goal", 1000, &Move::setGoalCallback, this);
    }

    int signNum(float num)
    // Return the sign of a number
    {
      return (num < 0.0) ? -1 : 1;
    }

    float calculateDistance(geometry_msgs::Pose currPose, geometry_msgs::Pose goalPose)
    // This function calculates the linear distance between two points
    {
      float xErr = currPose.position.x - goalPose.position.x;
      float yErr = currPose.position.y - goalPose.position.y;
      return sqrt(pow(xErr,2) + pow(yErr,2));
    }

    float calculateYawFromQuaterion(geometry_msgs::Pose pose)
    // Calculate and return the yaw of a pose quaternion
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

    void setGoalCallback(const geometry_msgs::Pose newGoal)
    // Set a new goal
    {
      if(activeGoal_ == false) // only set goal if there is not one currently active
      {
        // Set the starting odometry as reference
        startOdom_ = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odometry/filtered", n_));

        // Set goal pose for pidCallback
        goal_ = newGoal;
        // Set linear and angular goal terms for pointAndShootCallback
        xGoal_ = calculateDistance(newGoal, startOdom_.pose.pose);
        yawGoal_ = calculateYawFromQuaterion(newGoal);

        // Set loop terms
        rotationComplete_ = false;
        activeGoal_ = true;
      }
      else
      {
        ROS_ERROR("Attempted to publish goal, while goal still active");
      }
    }

    //
    // The following functions primarily relate to the point and shoot fucntion which has the robot rotate then move
    //

    void pointAndShootCallback(const nav_msgs::Odometry odom)
    // This function has the robot move to a goal position if a goal is active
    // It is broken down such that the robot will first rotate and then move forward to simplify things
    {
      if(activeGoal_)
      {
        geometry_msgs::Twist command;

        if (!rotationComplete_) // Complete Linear Motion
        {
          command = angularController(odom);
          pub_.publish(command);
          // If there are more than 10 loops without a command, move on
          if(noCommandIterations_ > 10)
          {
            rotationComplete_ = true;
            prevErrAngular_ = 0;
            noCommandIterations_ = 0;
          }
        }
        else // Complete linear motion
        {
          command = distanceController(odom);
          pub_.publish(command);
          // If there are more than 10 loops without a command, move on
          if(noCommandIterations_ > 10)
          {
            activeGoal_ = false;
            prevErrLinear_ = 0;
            noCommandIterations_ = 0;
          }
        }
      }
    }

    float normalizeAngleDiff(float currAngle, float goalAngle)
    // This function normalizes the difference between two angles to account for looping
    {
      float diff = goalAngle - currAngle;
      if (diff > M_PI)
      {
        return diff - 2*M_PI;
      }
      else if (diff < -M_PI)
      {
        return diff + 2*M_PI;
      }
      else
      {
        return diff;
      }
    }

    geometry_msgs::Twist distanceController(nav_msgs::Odometry odom)
    // This function moves the robot forward at a minimum speed until it traves a predetermined distance. 
    // It only moves forward, not back, due to the way the Sureclean robot picks up litter
    {
      geometry_msgs::Twist command;
      float traveled = calculateDistance(odom.pose.pose, startOdom_.pose.pose);
      errLinear_ = xGoal_ - traveled;
      if (fabs(errLinear_) < .05)
      {
        noCommandIterations_++;
      }
      else
      {
        command.linear.x = signNum(errLinear_)*minLinear_;
      }
      return command;
    }

    geometry_msgs::Twist angularController(nav_msgs::Odometry odom)
    // This function rotates the robot to a pre-determined goal yaw.
    {
      geometry_msgs::Twist command;
      float yawCurr = calculateYawFromQuaterion(odom.pose.pose);
      errAngular_ = normalizeAngleDiff(yawCurr, yawGoal_);

      if (fabs(errAngular_) < .01)
      {
        noCommandIterations_++;
      }
      else
      {
        noCommandIterations_ = 0;
        command.angular.z = signNum(errAngular_)*minAngular_;
      }
      prevErrAngular_ = errAngular_;
      return command;
    }


    //
    // The following functions primarily relate to the PID fucntion which has the robot move and rotate simultaneously
    //

    void pidCallback(const nav_msgs::Odometry odom)
    // This function has the robot move to a goal position if a goal is active
    // It is broken down such that the robot will first rotate and then move forward to simplify things
    {
      if(activeGoal_)
      {
        geometry_msgs::Twist command = PID(odom);
        pub_.publish(command);
        // If there are more than 10 loops without a command, reset and move on
        if(noCommandIterations_ > 10)
        {
          prevErrLinear_ = 0;
          prevErrAngular_ = 0;
          noCommandIterations_ = 0;
          activeGoal_ = false;
        }
      }
    }

    float calculateDeltaYawFromPositions(geometry_msgs::Pose currPose)
    // This functino determines the yaw needed to rotate robot such that it is facing it's goal
    {
      float yawCurr = calculateYawFromQuaterion(currPose); // Get current yaw in odom frame

      // Rotate positons by inverse transform, to put them in the robot's body frame
      float robotFrameX = currPose.position.x*cos(yawCurr) + currPose.position.y*sin(yawCurr);
      float robotFrameY = -currPose.position.x*sin(yawCurr) + currPose.position.y*cos(yawCurr);

      float robotFrameGoalX = goal_.position.x*cos(yawCurr) + goal_.position.y*sin(yawCurr);
      float robotFrameGoalY = -goal_.position.x*sin(yawCurr) + goal_.position.y*cos(yawCurr);

      // Calculate the delta yaw needed to directly face the goal
      float deltaX = robotFrameGoalX - robotFrameX;
      float deltaY = robotFrameGoalY - robotFrameY;
      return atan2(deltaY, deltaX);
    }

    geometry_msgs::Twist PID(nav_msgs::Odometry odom)
    // This function performs a PID for the angular rotation.
    {
      geometry_msgs::Twist command;

      // Angular PID
      errAngular_ = calculateDeltaYawFromPositions(odom.pose.pose);
      errDiffAngular_ = errAngular_ - prevErrAngular_;
      command.angular.z = kpAngular_*errAngular_ + kdAngular_*errDiffAngular_;
      if (fabs(command.angular.z) < minAngular_ && !rotationComplete_)
      {
        command.angular.z = signNum(command.angular.z)*minAngular_;
      }

      prevErrAngular_ = errAngular_;

      // Ensure the robot has reached certain angular accuracy before moving forward      
      if (fabs(errAngular_) < .02)
      {
        rotationComplete_ = true;
      }

      if (rotationComplete_)
      {
        errLinear_ = calculateDistance(odom.pose.pose, goal_);

        // Conditions of goal completed
        if (fabs(errLinear_) < .1)
        {
          command.linear.x = 0;
          noCommandIterations_++;
        }
        else
        {
          // Linear PID
          noCommandIterations_ = 0;
          errDiffLinear_ = errLinear_ - prevErrLinear_;
          command.linear.x = kpLinear_*errLinear_ + kdLinear_*errDiffLinear_;
          if (command.linear.x < 0.0)
          {
            command.linear.x = 0;
          }
        }
        prevErrLinear_ = errLinear_;
      }
      return command;
    }

  private:
    float kpLinear_;
    float kdLinear_;
    float kiLinear_;

    float kpAngular_;
    float kdAngular_;
    float kiAngular_;

    float errLinear_;
    float prevErrAngular_;
    float errDiffLinear_;

    float errAngular_;
    float prevErrLinear_;
    float errDiffAngular_;

    int noCommandIterations_;

    bool activeGoal_;
    geometry_msgs::Pose goal_;
    float yawGoal_;

    float xGoal_;

    bool rotationComplete_;

    nav_msgs::Odometry startOdom_;

    float minAngular_;
    float maxAngular_;

    float minLinear_;
    float maxLinear_;

    ros::Publisher pub_;
    ros::Subscriber subOdom_;
    ros::Subscriber subGoal_;
    ros::NodeHandle n_;
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "move_pid");
  
  Move move;
  
  

  ros::Rate loop_rate(200);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
