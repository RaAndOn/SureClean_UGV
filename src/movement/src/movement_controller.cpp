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
      // Gains for linear PID controller
      // kpLinear_ = 1.0;
      // kdLinear_ = 0.05;

      // Gains for angular PID controller
      // kpAngular_ = 1.0;
      // kdAngular_ = 0.05;

      // Initialize loop terms
      noCommandIterations_ = 0;
      moveSignal_ = false;
      onFinalApproach_ = false;
      rotationComplete_ = false;
      linearComplete_ = false;

      // Min and Max values
      // minAngular_ = 0.15;
      // maxAngular_ = .5;

      // minLinear_ = 0.2;
      // maxLinear_ = 0.5;

      n_.param("kp_linear", kpLinear_, 1.0);
      n_.param("kp_angular", kpAngular_, 1.0);
      n_.param("min_angular", minAngular_, 0.15);
      n_.param("max_angular_", maxAngular_, 0.5);
      n_.param("min_linear", minLinear_, 0.2);
      n_.param("max_linear", maxLinear_, 0.5);
      n_.param("final_approach_range", finalApproachRange_, 2.0);
      n_.param("angular_thresh", angularThresh_, 0.04);
      n_.param("linear_thresh", linearThresh_, 0.1);

      // Set publishers and subscribers 
      pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
      pubStatus_ = n_.advertise<std_msgs::Bool>("/goal_achieve_status",0);
      subOdom_ = n_.subscribe("odometry/filtered_gps", 1000, &Move::huskyControlCallback, this);
      subGoal_ = n_.subscribe("odometry_goal", 1000, &Move::setGoalCallback, this);
      emergencyStop_ = n_.advertiseService("/emergency_stop",&Move::emergencyStop,this);
      continueMovement_   = n_.advertiseService("/continue_mission",&Move::continueMove,this);
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

    void setGoalCallback(const nav_msgs::Odometry newGoal)
    // Set a new goal
    {
      if(moveSignal_ == false) // only set goal if there is not one currently active
      {
        // Set goal pose for pidCallback
        goal_ = newGoal.pose.pose;

        // ROS_INFO("---------- Moving to Goal --------");
        std::cout << "x: "<< goal_.position.x << "; y: "<< goal_.position.y << std::endl;

        // Set loop terms
        rotationComplete_ = false;
        linearComplete_ = false;
        moveSignal_ = true;
        onFinalApproach_ = false;
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
    // This function determines the yaw needed to rotate robot such that it is facing its goal
    {
      double yawCurr = calculateYawFromQuaterion(currPose); // Get current yaw in odom frame

      // Rotate positons by inverse transform, to put them in the robot's body frame
      double robotFrameX = currPose.position.x*cos(yawCurr) + currPose.position.y*sin(yawCurr);
      double robotFrameY = -currPose.position.x*sin(yawCurr) + currPose.position.y*cos(yawCurr);

      double robotFrameGoalX = goal_.position.x*cos(yawCurr) + goal_.position.y*sin(yawCurr);
      double robotFrameGoalY = -goal_.position.x*sin(yawCurr) + goal_.position.y*cos(yawCurr);

      // Calculate the delta yaw needed to directly face the goal
      double deltaX = robotFrameGoalX - robotFrameX;
      double deltaY = robotFrameGoalY - robotFrameY;
      return atan2(deltaY, deltaX);
    }

    void huskyControlCallback(const nav_msgs::Odometry odom)
    {
      geometry_msgs::Twist command;
      // double errAngular = calculateDeltaYawFromPositions(odom.pose.pose);
      double yawCurr = calculateYawFromQuaterion(odom.pose.pose);
      double errAngular = normalizeAngleDiff(yawCurr, yawGoal_);
      double errLinear = calculateDistance(odom.pose.pose, goal_);
      if (moveSignal_)
      {
        if (errLinear <= finalApproachRange_ && !onFinalApproach_)
        {
          ROS_INFO("FINAL APPROACH");
          command.angular.z = angularController(errAngular);
          if (rotationComplete_)
          {
            onFinalApproach_ = true;
          }
        }
        else
        {
          command.angular.z = angularController(errAngular);
          if (fabs(errAngular) > M_PI / 24)
          {
            command.linear.x = linearController(errLinear);
          }
        }
      }
      pub_.publish(command);
      status_check();

    }

    double angularController(double errAngular)
    {
      double commandAngular = 0;
      if (fabs(errAngular) > angularThresh_ ) {
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
    // This function moves the robot forward at a minimum speed until it traves a predetermined distance. 
    // It only moves forward, not back, due to the way the Sureclean robot picks up litter
    {
      double commandLinear = 0;

      if (fabs(errLinear) > linearThresh_) {
        if (errLinear > finalApproachRange_) {
          errLinear = finalApproachRange_;
        }
        commandLinear  = kpLinear_ * minLinear_ * errLinear/finalApproachRange_;
        if (fabs(commandLinear) < minLinear_) {
          commandLinear = signNum(commandLinear) * minLinear_;
        }
        if (fabs(commandLinear) > maxLinear_) {
          commandLinear = signNum(commandLinear) * maxLinear_;
        }
      }
      else
      {
        linearComplete_ = true;
      }
      
      return commandLinear;
    }

    void status_check() {
      std_msgs::Bool status_msgs;
      status_msgs.data = false;
      if (linearComplete_) {
        status_msgs.data = true;
        moveSignal_ = false;
        ROS_INFO("---------- Goal Achieved --------");
        pubStatus_.publish(status_msgs);
      }
    }

    bool emergencyStop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        moveSignal_ = false;
        return true;
    }

    bool continueMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        moveSignal_ = true;
        return true;
    }

  private:
    double kpLinear_;
    // double kdLinear_;

    double kpAngular_;
    // double kdAngular_;

    int noCommandIterations_;

    bool moveSignal_;
    bool onFinalApproach_;
    geometry_msgs::Pose goal_;
    double yawGoal_;

    double xGoal_;

    bool rotationComplete_;
    bool linearComplete_;

    double minAngular_;
    double maxAngular_;

    double minLinear_;
    double maxLinear_;

    double finalApproachRange_;
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
