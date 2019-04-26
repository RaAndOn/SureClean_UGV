
#include <ros/ros.h>
#include <ros/package.h>
#include <iomanip>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <queue> 
#include <math.h>
#include <robot_localization/navsat_conversions.h>

using namespace std;

class GpsNav {
private:
  ros::Subscriber subGPS_;
  ros::Publisher pubGoal_;
  ros::Subscriber subGoalStatus_;
  ros::ServiceServer serviceCollect_;
  ros::ServiceServer serviceMove_;
  ros::NodeHandle n_;
  queue<nav_msgs::Odometry> goalList_;

  sensor_msgs::NavSatFix currGPS_;
  
  double magnetic_declination_;

  bool autonomous_;

public:
  GpsNav() {

    autonomous_ = true;
    magnetic_declination_ = -0.16347917327;
    pubGoal_ = n_.advertise<nav_msgs::Odometry>("/odometry_goal",0);
    subGoalStatus_ = n_.subscribe("/goal_achieve_status",0,&GpsNav::goalAchieved,this);
    serviceCollect_ = n_.advertiseService("/collect_goal",&GpsNav::collectGoal,this);
    serviceMove_ = n_.advertiseService("/move_next_goal",&GpsNav::nextGoalService,this);
    subGPS_ = n_.subscribe("gps/fix",0,&GpsNav::updateCurrentGPS,this);
  }
  ~GpsNav() {}

  geometry_msgs::PointStamped latLongtoUTM(double latiInput, double longiInput)
  {
    double utm_x = 0, utm_y = 0;
    std::string utm_zone;
    geometry_msgs::PointStamped utmPointOutput;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(latiInput, longiInput, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    utmPointOutput.header.frame_id = "utm";
    utmPointOutput.header.stamp = ros::Time(0);
    utmPointOutput.point.x = utm_x;
    utmPointOutput.point.y = utm_y;
    utmPointOutput.point.z = 0;

    return utmPointOutput;
  }

  geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
  {
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
      try
      {
        UTM_input.header.stamp = ros::Time::now();
        listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
        listener.transformPoint("odom", UTM_input, map_point_output);
        notDone = false;
      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.01).sleep();
        //return;
      }
    }
    return map_point_output;
  }

  void goalAchieved(std_msgs::Bool msg)
  {
      
  }

  void updateCurrentGPS(const sensor_msgs::NavSatFix gps) {
    currGPS_ = gps;
    }

  bool collectGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
      geometry_msgs::PointStamped utmGoal = latLongtoUTM(currGPS_.latitude, currGPS_.longitude);
      //Transform UTM to map point in odom frame
      geometry_msgs::PointStamped goalPoint = UTMtoMapPoint(utmGoal);
      nav_msgs::Odometry odomGoal;
      odomGoal.pose.pose.position.x = goalPoint.point.x;
      odomGoal.pose.pose.position.y = goalPoint.point.y;
      
      goalList_.push(odomGoal);
      return true;
  }

  bool nextGoalService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
      return moveToNextGoal();
  }

  bool moveToNextGoal() {
      if (goalList_.empty()) {
          ROS_ERROR("No goal gps in the goal list");
          return false;
      }
      nav_msgs::Odometry nextGoal = goalList_.front();
      pubGoal_.publish(nextGoal);
      goalList_.pop();
      ROS_INFO("---------- Go to the next goal --------");
      return true;
  }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_nav_node");
    GpsNav gpsNav;
    ros::Rate loop_rate(200);
    ROS_INFO("In main\n");
    while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
}





