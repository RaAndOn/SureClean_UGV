#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <math.h>



class Move
{
  public:
    Move()
    {
      float kpLinear = 0.8;
      float kdLinear = 0.8;
      float kiLinear = 0.0;

      float kpAngular = .5;
      float kdAngular = .4;
      float kiAngular = 0.0;

      float prevErr = 0.0;

      float err = 0.0;
      float errDiff = 0.0;

      int noCommandIterations = 0;

      geometry_msgs::Twist command;

      bool activeGoal;
      nav_msgs::Odometry goal;
      float yawGoal;
      float yawCurr;

      bool rotationComplete = false;
      pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
      subOdom_ = n_.subscribe("odometry/filtered", 1000, &Move::moveCallback, this);
      subGoal_ = n_.subscribe("goal", 1000, &Move::setGoal, this);
    }

    float calculateYaw(nav_msgs::Odometry odom)
    {
      double roll, pitch, yaw;
      double quatx= odom.pose.pose.orientation.x;
      double quaty= odom.pose.pose.orientation.y;
      double quatz= odom.pose.pose.orientation.z;
      double quatw= odom.pose.pose.orientation.w;
      tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
      tf::Matrix3x3 rotMatrix(quaternion);
      rotMatrix.getRPY(roll, pitch, yaw);
      return yaw;
    }

    void setGoal(const nav_msgs::Odometry newGoal)
    {
      if(activeGoal == false)
      {
        goal = newGoal;
        activeGoal = true;
        yawGoal = calculateYaw(newGoal);
        rotationComplete = false;
      }
      else
      {
        ROS_ERROR("Attempted to publish goal, while goal still active");
      }
    }

    float getLinearMagnitude(nav_msgs::Odometry odom)
    {
      float poseX = odom.pose.pose.position.x;
      float poseY = odom.pose.pose.position.y;
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
      err = getLinearMagnitude(odom) - getLinearMagnitude(goal);

      if (fabs(err) < .3)
      {
        command.linear.x = 0;
        noCommandIterations++;
      }
      else
      {
        errDiff = prevErr - err;
        command.linear.x = -kpLinear*err - kdLinear*errDiff;
        if (command.linear.x > 0.5)
        {
          command.linear.x = 0.5;
        }
        else if (command.linear.x < -0.5)
        {
          command.linear.x = -0.5;
        }
        prevErr = err;
      }
    }

    void angularPID(nav_msgs::Odometry odom)
    {
      yawCurr = calculateYaw(odom);

      err = normalizeAngleDiff(yawCurr, yawGoal);

      if (fabs(err) < .05)
      {
        command.angular.z = 0;
        noCommandIterations++;
      }
      else
      {
        errDiff = prevErr - err;
        command.angular.z = -kpAngular*err - kdAngular*errDiff;
        if (command.angular.z > 1)
        {
          command.angular.z = 1;
          command.linear.z = 1;
        }
        else if (command.angular.z < -1)
        {
          command.angular.z = -1;
        }
        prevErr = err;
      }
    }

    void moveCallback(const nav_msgs::Odometry odom)
    {
      noCommandIterations = 0;
      while(activeGoal)
      {
        if (!rotationComplete)
        {
          angularPID(odom);
          pub_.publish(command);
          if(noCommandIterations > 10)
          {
            rotationComplete = true;
            prevErr = 0;
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
    nav_msgs::Odometry goal;
    float yawGoal;
    float yawCurr;

    bool rotationComplete;

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
