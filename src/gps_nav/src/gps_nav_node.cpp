#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternoin.h>
#include <nav_msgs/Odometry.h>
#include <gps_nav/GoalCollect.h>
#include <queue> 
#include <math.h>

using namespace std;


#define THRED 0.05
#define RADIUS_EARTH 6378.137
#define MAX_SPEED 1
#define MAX_ANGULAR 0.5
#define DIS_RANGE 10
#define Aver_Time 50

ros::Subscribe sub_gps;
ros::Subscribe sub_odom;
ros::Publisher pub_cmd;
ros::Publisher pub_status;
ros::ServiceServer server_goal;
ros::ServiceServer server_move;
ros::ServiceServer server_stop;
ros::ServiceServer server_stop;
ros::ServiceClient client_goal;

sensor_msgs::NavSatFix goal_gps_;
sensor_msgs::NavSatFix ori_gps_;
sensor_msgs::NavSatFix gps_current;


geometry_msgs::Ouaternoin goal_pose;
geometry_msgs::Quaternoin robot_pose;

geometry_msgs::Twist robot_vel;
geometry_msgs::Twist ctrl_msg;

queue<geometry_msgs::Ouaternoin> goal_list;

int ori_index = 0;
bool status_ = false;
bool move_signal = false;
bool move_status = false;
bool ori_status = false;

double UTC2Map(double lat1, double lat2, double lon1, double lon2) {
    double R = RADIUS_EARTH;
    double dLat = lat2 * M_PI / 180 - lat1 * M_PI / 180;
    double dLon = lon2 * M_PI / 180 - lon1 * M_PI / 180;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;
    return d * 1000;
}

void status_check(bool linear, bool angular) {
    std_msgs::Bool status_msgs;
    status_msgs.data = false;
    status_ = false;
    if (linear && angular) {
        status_msgs.data = true;
        status_ = true;
    }
    pub_status.publish(status_msgs);
}

void contrl_husky() {
    float angular_thred = 0.02;
    float linear_thred = 0.1;
    // PD control of husky
    float Kp = 1;
    float Kd = 0.05;

    float goal_angular = goal_pose.w;
    float current_angular = robot_pose.w;
    float d_angular = goal_angular - current_angular;

    float dx = goal_pose.x - robot_pose.x;
    float dy = goal_pose.y - robot_pose.y;
    double d_linear = sqrt(pow(dx,2) + pow(dy,2));

    bool check_linear;
    bool check_angular;
    
    if (d_linear > linear_thred)) {
        double ctrl_vel_linear = robot_vel.linear.x;
        if (d_linear > DIS_RANGE) {
            d_linear = DIS_RANGE;
        }
        ctrl_msg.linear.x  = Kp * MAX_SPEED * d_linear/DIS_RANGE - Kd * ctrl_vel_linear;
        check_linear = false;
    }
    else {
        ctrl_msg.linear.x = 0;
        check_linear = true;
    }

    if (fabs(d_angular) > angular_thred) {
        double ctrl_vel_angular = robot_vel.angular.z;
        if (fabs(d_angular) > M_PI/2) {
            d_angular = M_PI/2;
        }
        ctrl_msg.angular.z = Kp * MAX_ANGULAR * d_angular / (M_PI/2) - Kd * ctrl_vel_angular;
        check_angular = false;
    }
    else {
        ctrl_msg.angular.z = 0;
        check_angular = true;
    }

    status_check(check_linear,check_angular);
}



void getPose(const sensor_msgs::NavSatFix &msg) {
    // set origin
    if (move_status == false && ori_index <= Aver_Time && ori_status == false) {
        ori_gps = msg;
        ROS_INFO("Initializing Origin --- Robot NOT Moving");
        ROS_INFO("Yaw is not useful right now");
        //use for generate postion
        double lat_ori += ori_gps.latitude;
        double lon_ori += ori_gps.longitude;
        ori_index += 1;
    }

    if ((ori_index > Aver_Time || move_status == true) && ori_status == false) {
        lat_ori = lat_ori / ori_index;
        lon_ori = lon_ori / ori_index;
        ori_gps.latitude = lat_ori;
        ori_gps.longitude = lon_ori;
        ROS_INFO("------------------Origin Initialization Completed-------------")
        ori_status = true;
    }
    gps_current = ori_gps;

    // use for yaw generation
    double lat1 = gps_current.latitude;
    double lon1 = gps_current.longitude;

    // the new gps signal
    double lat2 = msg.latitude;
    double lat2 = msg.longitude;

    // define the x-axis point to the North ----- latitude
    double dx = UTC2Map(lat1,lat2,lon2,lon2);
    // define the x-axis point to the West ----- longitude
    double dy = UTC2Map(lat2,lat2,lon1,lon2);
    double angular = atan2(dy,dx);
    // get map position
    double x = UTC2Map(lat_ori,lat2,lon2,lon2);
    double y = UTC2Map(lat2,lat2,lon_ori,lon2);

    robot_pose.x = x;
    robot_pose.y = y;
    robot_pose.z = 0;
    robot_pose.w = angular;

    if (move_signal) {
        control_husky();
    }
    
}

void updateOdom(const nav_msgs::Odometry &msg) {
    double vel_thred = THRED;
    geometry_msgs::Twist vel = msg.twist.twist;
    if (fabs(vel.linear.x) > vel_thred || fabs(vel.linear.y) > vel_thred) {
        move_status = true;
    }
    else move_status = false;
    robot_vel = vel;
}

// gps_nav::GoalCollect::Request  &req,
// gps_nav::GoalCollect::Response &res

bool getGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    goal_list.push_back(gps_current = ori_gps);
    return true;
}
bool NextGoalMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    if (goal_list.empty()) {
        move_signal = false
        ROS_ERROR("No goal gps in the goal list")
        return false;
    }
    move_signal = true;
    goal_gps_ = goal_list.front();
    goal_list.pop();
    ROS_INFO("----------Go to the next goal--------");
    move_signal = true;
    return true;
}

bool emergency_stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    move_signal = false;
}

bool continue_move(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    move_signal = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle n;
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel",0);
    pub_status = n.advertise<std_msgs::Bool>("/goal_achieve_status",0);
    sub_odom = n.subscribe("/husky_velocity_controller/odom",0,&updateOdom);
    sub_gps = n.subscribe("/gps/filtered",0,&getPose);
    server_goal = n.advertiseService("/collect_goal",&getGoal);
    server_move = n.advertiseService("/move_next_goal",&NextGoalMove);
    server_stop = n.advertiseService("/emergency_stop",&emergency_stop);
    server_go = n.advertiseService("/continue_mission",&continue_move);

    ros::spin();
}





