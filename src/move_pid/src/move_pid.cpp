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
      kpLinear = 0.8;
      kdLinear = 0.8;
      kiLinear = 0.0;

      kpAngular = .5;
      kdAngular = .4;
      kiAngular = 0.0;

      prevErr = 0.0;

      err = 0.0;
      errDiff = 0.0;

      noCommandIterations = 0;

      activeGoal = false;
      
      rotationComplete = false;
      pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
      subOdom_ = n_.subscribe("odometry/filtered", 1000, &Move::moveCallback, this);
      subGoal_ = n_.subscribe("goal", 1000, &Move::setGoal, this);
    }

    float calculateYaw(geometry_msgs::Pose pose)
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

    void setGoal(const geometry_msgs::Pose newGoal)
    {
      ROS_INFO("HERE");
      if(activeGoal == false)
      {
        startOdom = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odometry/filtered", n_));
        xGoal = fabs(getLinearMagnitude(newGoal) - getLinearMagnitude(startOdom.pose.pose));
        activeGoal = true;
        yawGoal = calculateYaw(newGoal);
        rotationComplete = false;
      }
      else
      {
        ROS_ERROR("Attempted to publish goal, while goal still active");
      }
    }

    float getLinearMagnitude(geometry_msgs::Pose pose)
    {
      float poseX = pose.position.x;
      float poseY = pose.position.y;
      return sqrt(pow(poseX,2) + pow(poseY,2));
    }


    float normalizeAngleDiff(float currAngle, float goalAngle)
    {
      float diff = currAngle - goalAngle;
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

    void linearPID(nav_msgs::Odometry odom)
    {
      float traveled = fabs(getLinearMagnitude(odom.pose.pose) - getLinearMagnitude(startOdom.pose.pose));
      err = traveled - xGoal;
      ROS_INFO("IN LINEAR");
      ROS_INFO("%f",getLinearMagnitude(odom.pose.pose));
      if (err > .05)
      {
        command.linear.x = 0;
        noCommandIterations++;
      }
      else
      {
        errDiff = prevErr - err;
        command.linear.x = -kpLinear*err - kdLinear*errDiff;
        if (command.linear.x > 0.0)
        {
          command.linear.x = .5;
        }
        else if (command.linear.x < 0.0)
        {
          command.linear.x = 0;
          noCommandIterations++;
        }
        prevErr = err;
      }
    }

    void angularPID(nav_msgs::Odometry odom)
    {
      yawCurr = calculateYaw(odom.pose.pose);
      ROS_INFO("IN ANGULAR");
      err = normalizeAngleDiff(yawCurr, yawGoal);

      if (fabs(err) < .01)
      {
        command.angular.z = 0;
        noCommandIterations++;
      }
      else
      {
        errDiff = prevErr - err;
        command.angular.z = -kpAngular*err - kdAngular*errDiff;
        if (command.angular.z > 0)
        {
          command.angular.z = .3;
        }
        else if (command.angular.z < -0)
        {
          command.angular.z = -.3;
        }
        prevErr = err;
      }
    }

    void moveCallback(const nav_msgs::Odometry odom)
    {
      if(activeGoal)
      {
        if (!rotationComplete)
        {
          angularPID(odom);
          pub_.publish(command);
          if(noCommandIterations > 10)
          {
            rotationComplete = true;
            prevErr = 0;
            noCommandIterations = 0;
          }
        }
        else
        {
          linearPID(odom);
          pub_.publish(command);
          if(noCommandIterations > 10)
          {
            activeGoal = false;
            prevErr = 0;
            noCommandIterations = 0;
          }
        }
      }
    }

  private:
    float kpLinear;
    float kdLinear;
    float kiLinear;

    float kpAngular;
    float kdAngular;
    float kiAngular;

    float prevErr;

    float err;
    float errDiff;

    int noCommandIterations;

    geometry_msgs::Twist command;

    bool activeGoal;
    geometry_msgs::Pose goal;
    float yawGoal;
    float yawCurr;

    float xGoal;

    bool rotationComplete;

    nav_msgs::Odometry startOdom;

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
