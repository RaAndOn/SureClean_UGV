
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
  ros::NodeHandle n_;
  ros::Publisher pubGoal_;
  ros::Subscriber subGPS_;
  ros::Subscriber subGoalStatus_;
  ros::ServiceServer serviceCollect_;
  ros::ServiceServer serviceMove_;

  queue<nav_msgs::Odometry> goalList_; // Queue of goal locations
  nav_msgs::Odometry currOdom_; // Current GPS Coordinates
  bool autonomous_; // Flag to determine if navigation to all goals will happen automatically

  double magnetic_declination_; // Magnetic declination

public:
  GpsNav() {
    n_.param("/gps_nav_node/autonomous", autonomous_, true);
    magnetic_declination_ = -0.16347917327; // magnetic declination of Pittsburgh
    pubGoal_ = n_.advertise<nav_msgs::Odometry>("/odometry_goal",0); // Publisher: sends goal odometry to controller
    subGoalStatus_ = n_.subscribe("/goal_achieve_status",0,&GpsNav::goalAchieved,this); // Subscriber: sayss if goal has been achieved
    subGPS_ = n_.subscribe("odometry/filtered_gps",0,&GpsNav::updateCurrentGPS,this); // Subscriber: gets latest GPS coordinates
    serviceCollect_ = n_.advertiseService("/collect_goal",&GpsNav::collectGoal,this); // Service: converts current GPS coordinates to odometry goal
    serviceMove_ = n_.advertiseService("/move_next_goal",&GpsNav::nextGoalService,this); // Service: begins navigation to goal
  }

  ~GpsNav() {}

  geometry_msgs::PointStamped latLongtoUTM(double latiInput, double longiInput)
  {
  // This Function transforms latitude and longitude into a point in the UTM frame
    double utmX = 0, utmY = 0;
    std::string utmZone;
    geometry_msgs::PointStamped utmPointOutput;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(latiInput, longiInput, utmY, utmX, utmZone);

    //Construct UTM_point and map_point geometry messages
    utmPointOutput.header.frame_id = "utm";
    utmPointOutput.header.stamp = ros::Time(0);
    utmPointOutput.point.x = utmX;
    utmPointOutput.point.y = utmY;
    utmPointOutput.point.z = 0;

    return utmPointOutput;
  }

  geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTMInput)
  {
  // This function transforms UTM points into the odometry frame

    geometry_msgs::PointStamped odomFrameOutput;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time timeNow = ros::Time::now();

    while(notDone)
    {
      try
      {
        // Transform UTM point to the odometry frame
        UTMInput.header.stamp = ros::Time::now();
        listener.waitForTransform("odom", "utm", timeNow, ros::Duration(3.0));
        listener.transformPoint("odom", UTMInput, odomFrameOutput);
        notDone = false;
      }
      catch (tf::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.01).sleep();
        //return;
      }
    }
    return odomFrameOutput;
  }

  void goalAchieved(std_msgs::Bool msg)
  // This function will automatically pass the next waypoint to the controller
  {
    if (msg.data && autonomous_)
    {
      moveToNextGoal();
    }
  }

  void updateCurrentGPS(const nav_msgs::Odometry odom) {
  // This callback updates the currGPS_ variable with the robot's current gps coordinates
    currOdom_ = odom;
  }

  bool collectGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  // This Service transforms the current lat and long into the odometry frame and stores it as a waypoint goal
    // geometry_msgs::PointStamped utmGoal = latLongtoUTM(currGPS_.latitude, currGPS_.longitude);
    // //Transform UTM to map point in odom frame
    // geometry_msgs::PointStamped goalPoint = UTMtoMapPoint(utmGoal);
    // nav_msgs::Odometry odomGoal;
    // odomGoal.pose.pose.position.x = goalPoint.point.x;
    // odomGoal.pose.pose.position.y = goalPoint.point.y;
    goalList_.push(currOdom_); // add goal to queue
    ROS_INFO("---------- Goal collected --------");
    std::cout << "x: "<< currOdom_.pose.pose.position.x << "; y: "<< currOdom_.pose.pose.position.y << std::endl;
    return true;
  }

  bool nextGoalService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  // This service calls will manually send the next goal to the controller
    return moveToNextGoal();
  }

  bool moveToNextGoal() {
  // This function passes waypoints to the controller

    // Raise error if goal list is empty
    if (goalList_.empty()) {
      ROS_ERROR("No goal gps in the goal list");
      return false;
    }
    // Remove goal from the front of the queue and publish it
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





