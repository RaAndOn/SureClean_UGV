#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

class Move
{
  public:
      // Point x -- east, y -- north and z -- yaw (up)
    Move()
    {
      // Initialize loop terms
      moveSignal_ = false;
      onFinalApproach_ = false;
      rotationComplete_ = false;
      linearComplete_ = false;
      activeGoal_ = false;
      // Proportional gains
      n_.param("/movement_node/kp_linear", kpLinear_, 1.0);
      n_.param("/movement_node/kp_angular", kpAngular_, 1.0);
      // Angular min and max commands
      n_.param("/movement_node/min_angular", minAngular_, 0.15);
      n_.param("/movement_node/max_angular", maxAngular_, 0.3);
      // Linear min and max commands
      n_.param("/movement_node/min_linear", minLinear_, 0.4);
      n_.param("/movement_node/max_linear", maxLinear_, 0.75);
      // Range at which to do final adjustment - meters
      n_.param("/movement_node/final_approach_range", finalApproachRange_, 2.0);
      // Angular convergence threshold - radians
      n_.param("/movement_node/angular_thresh", angularThresh_, 0.04);
      // Angular convergence threshold to perform movement- radians
      n_.param("/movement_node/angular_thresh_movement", angularThreshMovement_, 0.13);
      // Linear convergence threshold - meters
      n_.param("/movement_node/linear_thresh", linearThresh_, 0.1);

      // Set publishers and subscribers 
      pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000); // Publisher: the controller output
      pubStatus_ = n_.advertise<std_msgs::Bool>("goal_achieve_status",0); // Publisher: whether goal has been achieved
      subOdom_ = n_.subscribe("odometry/filtered_gps", 1000, &Move::huskyControlCallback, this); // Subscriber: get current odometry 
      subGoal_ = n_.subscribe("odometry_goal", 1000, &Move::setGoalCallback, this); // Subscriber: sets odometry goal
      emergencyStop_ = n_.advertiseService("emergency_stop",&Move::emergencyStop,this); // Service: sets flag to stop movement
      continueMovement_   = n_.advertiseService("continue_mission",&Move::continueMove,this); // Service: sets flag to continue movement
    }

    ~Move() {}

    int signNum(double num)
    // Return the sign of a number
    {
      return (num < 0.0) ? -1 : 1;
    }

    double calculateDistance(geometry_msgs::Pose currPose, geometry_msgs::Pose goalPose)
    // This function calculates the linear distance between two points
    {
      double xErr = currPose.position.x - goalPose.position.x;
      double yErr = currPose.position.y - goalPose.position.y;
      return sqrt(pow(xErr,2) + pow(yErr,2));
    }

    double calculateYawFromQuaterion(geometry_msgs::Pose pose)
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

    void setGoalCallback(const nav_msgs::Odometry newGoal)
    // Set a new goal
    {
      if(!activeGoal_) // only set goal if there is not one currently active
      {
        // Set goal pose for pidCallback
        goal_ = newGoal.pose.pose;
        ROS_INFO("-------New Goal-------");
        std::cout << "x: "<< goal_.position.x << "; y: "<< goal_.position.y << std::endl;

        // Reset control loop terms
        rotationComplete_ = false;
        linearComplete_ = false;
        onFinalApproach_ = false;
        moveSignal_ = true;
        activeGoal_ = true;
      }
      else
      {
        ROS_ERROR("Attempted to publish goal, while goal still active");
      }
    }


    double normalizeAngleDiff(double currAngle, double goalAngle)
    // This function normalizes the difference between two angles to account for looping
    {
      double diff = goalAngle - currAngle;
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


    double calculateDeltaYawFromPositions(geometry_msgs::Pose currPose)
    // This function determines the error between the current yaw and the desired position
    {
      double yawCurr = calculateYawFromQuaterion(currPose); // Get current yaw in odom frame
      double dyGoal = goal_.position.y - currPose.position.y;
      double dxGoal = goal_.position.x - currPose.position.x;
      double yawGoal = atan2(dyGoal,dxGoal);
      return normalizeAngleDiff(yawCurr, yawGoal);
    }

    void huskyControlCallback(const nav_msgs::Odometry odom)
    // This function sets the command velocities for the robot to reach the goal
    // It prioritizes driving the front of the robot directly over the goal
    {
      geometry_msgs::Twist command;
      double errAngular = calculateDeltaYawFromPositions(odom.pose.pose);
      double errLinear = calculateDistance(odom.pose.pose, goal_);
      // If there is an activeGoal_, check for convergence, before moving
      if (activeGoal_)
      {
        statusCheck(errLinear);
      }
      if (moveSignal_) // Only move calculate and publish command if moveSignal_ is true
      {
        // If within range, perform final yaw correction before final approach
        if (errLinear <= finalApproachRange_ && !onFinalApproach_)
        {
          command.angular.z = angularController(errAngular);
          if (fabs(errAngular) < angularThresh_)
          {
            ROS_INFO("FINAL APPROACH");
            onFinalApproach_ = true;
          }
        }
        else
        {
          command.angular.z = angularController(errAngular); // Set angular command
          // Once the robot has achieved angularThresh_, yoyu can move
          // Only stop if it drops bellow angularThreshMovement_
          if (fabs(errAngular) < angularThreshMovement_ && rotationComplete_)
          {
            command.linear.x = linearController(errLinear);
          }
        }
        ROS_INFO("Linear Command: %f", command.linear.x);
        ROS_INFO("Angular Command: %f", command.angular.z);
        pub_.publish(command); // Publish command
      }

    }

    double angularController(double errAngular)
    // This function returns the angular command for the robot
    {
      double commandAngular = 0;
      if (fabs(errAngular) > angularThresh_) {
        if (fabs(errAngular) > M_PI_2) {
          errAngular = signNum(errAngular)*M_PI_2;
        }
        commandAngular = kpAngular_ * errAngular / M_PI_2;
        if (fabs(commandAngular) < minAngular_) {
          commandAngular = signNum(commandAngular) * minAngular_;
        }
        if (fabs(commandAngular) > maxAngular_) {
          commandAngular = signNum(commandAngular) * maxAngular_;
        }
        rotationComplete_ = false;
      }
      else
      {
        rotationComplete_ = true;
      }
      return commandAngular;
      
    }

    double linearController(double errLinear)
    // This function returns the linear command for the robot
    {
      double commandLinear = 0;
      if (errLinear > linearThresh_) {
        if (errLinear > finalApproachRange_) {
          errLinear = finalApproachRange_;
        }
        commandLinear  = kpLinear_ * minLinear_ * errLinear/finalApproachRange_;
        if (commandLinear < minLinear_) {
          commandLinear = minLinear_;
        }
        if (commandLinear > maxLinear_) {
          commandLinear = maxLinear_;
        }
      }
      else
      {
        linearComplete_ = true;
      }
      
      return commandLinear;
    }

    void statusCheck(double errLinear) {
    // This function determines whether the robot has reachced its goal
      std_msgs::Bool status_msgs;
      status_msgs.data = false;
      if (errLinear < linearThresh_) {
        status_msgs.data = true;
        activeGoal_ = false;
        moveSignal_ = false;
        linearComplete_ = false;
        ROS_INFO("---------- Goal Achieved --------");
        pubStatus_.publish(status_msgs);
      }
    }

    bool emergencyStop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    // This service will stop the robot
      moveSignal_ = false;
      return true;
    }

    bool continueMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    // This service will undo the emergency stop
      moveSignal_ = true;
      return true;
    }

  private:
    double kpLinear_;
    double kpAngular_;

    bool moveSignal_;
    bool activeGoal_;
    bool onFinalApproach_;
    geometry_msgs::Pose goal_;

    bool rotationComplete_;
    bool linearComplete_;

    double minAngular_;
    double maxAngular_;

    double minLinear_;
    double maxLinear_;

    double finalApproachRange_;
    double angularThreshMovement_;
    double angularThresh_;
    double linearThresh_;


    ros::Publisher pub_;
    ros::Publisher pubStatus_;
    ros::Subscriber subOdom_;
    ros::Subscriber subGoal_;
    ros::ServiceServer emergencyStop_;
    ros::ServiceServer continueMovement_;
    ros::NodeHandle n_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "movement_node");
  Move move;
  ros::Rate loop_rate(20);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
