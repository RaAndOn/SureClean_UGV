
#include <ros/ros.h>
#include <iomanip>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <queue> 
#include <math.h>

using namespace std;

#define THRED 0.01
#define RADIUS_EARTH 6378.137
#define MAX_SPEED 1
#define MIN_SPEED 0.3
#define MAX_ANGULAR 0.5
#define MIN_ANGULAR 0.1
#define DIS_RANGE 10
#define Aver_Time 50

ros::Subscriber sub_gps;
ros::Subscriber sub_odom;
ros::Publisher pub_cmd;
ros::Publisher pub_status;
ros::ServiceServer server_goal;
ros::ServiceServer server_move;
ros::ServiceServer server_stop;
ros::ServiceServer server_go;

ros::ServiceClient client_goal;

sensor_msgs::NavSatFix goal_gps_;
sensor_msgs::NavSatFix ori_gps_;
sensor_msgs::NavSatFix gps_current_;

// Point x -- north, y -- west and z -- yaw
geometry_msgs::Point goal_pose_;
geometry_msgs::Point robot_pose_;

geometry_msgs::Twist robot_vel_;

queue<sensor_msgs::NavSatFix> goal_list_;

int ori_index_ = 0;
bool status_ = false;
bool move_signal_ = false;
bool move_status_ = false;
bool ori_status_ = false;

double UTC2Map(double lat1, double lat2, double lon1, double lon2) {
    int sign = 1;
    if (lat2 >= lat1) sign = 1;  // heading north
    else sign = -1;
    if (lon2 <= lon1) sign = 1;  // heading west
    else sign = -1;
    double R = RADIUS_EARTH;
    double dLat = lat2 * M_PI / 180 - lat1 * M_PI / 180;
    double dLon = lon2 * M_PI / 180 - lon1 * M_PI / 180;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = sign * R * c * 1000;
    return d;
}

void status_check(bool linear, bool angular) {
    std_msgs::Bool status_msgs;
    status_msgs.data = false;
    status_ = false;
    if (linear && angular) {
        status_msgs.data = true;
        status_ = true;
        move_signal_ = false;
    }
    pub_status.publish(status_msgs);
}

float normalizeAngleDiff(float currAngle, float goalAngle)
// This function normalizes the difference between two angles to account for looping
{
    float diff = goalAngle - currAngle;
    if (diff > M_PI) return diff - 2 * M_PI;
    else if (diff < -M_PI) return diff + 2 * M_PI;
    else return diff;
}

void contrl_husky() {
    geometry_msgs::Twist ctrl_msg;

    float angular_thred = 0.05;
    float linear_thred = 0.1;
    // PD control of husky
    float Kp = 1;
    float Kd = 0.05;

    float goal_angular = goal_pose_.z;
    float current_angular = robot_pose_.z;
    float d_angular = normalizeAngleDiff(current_angular, goal_angular);

    float dx = goal_pose_.x - robot_pose_.x;
    float dy = goal_pose_.y - robot_pose_.y;
    double d_linear = sqrt(pow(dx,2) + pow(dy,2));

    bool check_linear;
    bool check_angular;
    
    if (d_linear > linear_thred) {
        double ctrl_vel_linear = robot_vel_.linear.x;
        if (d_linear > DIS_RANGE) {
            d_linear = DIS_RANGE;
        }
        ctrl_msg.linear.x  = Kp * MAX_SPEED * d_linear/DIS_RANGE - Kd * ctrl_vel_linear;
        if (ctrl_msg.linear.x < MIN_SPEED){
	        ctrl_msg.linear.x = MIN_SPEED;
        check_linear = false;
    }
    else {
        ctrl_msg.linear.x = 0;
        check_linear = true;
    }

    if (fabs(d_angular) > angular_thred && check_linear == false) {
        double ctrl_vel_angular = ctrl_msg.angular.z;
        if (fabs(d_angular) > M_PI/2) {
            d_angular = M_PI/2;
        }
        if (ctrl_msg.angular.z < MIN_ANGULAR) {
	        ctrl_msg.angular.z = MIN_ANGULAR;
        // right handed
        ctrl_msg.angular.z =  (Kp * MAX_ANGULAR * d_angular / (M_PI/2) - Kd * ctrl_vel_angular);
        check_angular = false;
    }
    else {
        ctrl_msg.angular.z = 0;
        check_angular = true;
    }
    pub_cmd.publish(ctrl_msg);
    cout << "Control output " << ctrl_msg.linear.x << "; "<< ctrl_msg.angular.z << endl;
    status_check(check_linear,check_angular);
}


void getPose(const sensor_msgs::NavSatFix msg) {
    // set origin

    if (move_status_ == false && ori_index_ <= Aver_Time && ori_status_ == false) {
        ori_gps_ = msg;
        ROS_INFO("Initializing Origin --- Robot NOT Moving");
        ROS_INFO("Yaw is not useful right now");
        //use for generate postion
        ori_index_ += 1;
    }

    if ((ori_index_ > Aver_Time || move_status_ == true) && ori_status_ == false) {

        cout << setprecision(10) <<"Ori_GPS = " <<ori_gps_.latitude<<"; "<<ori_gps_.longitude<<"; "<<endl;
        ROS_INFO("------------------Origin Initialization Completed-------------");
        ori_status_ = true;
    }

    double lat_ori = ori_gps_.latitude;
    double lon_ori = ori_gps_.longitude;

    // use for yaw generation
    double lat1 = gps_current_.latitude;
    double lon1 = gps_current_.longitude;

    // the new gps signal
    double lat2 = msg.latitude;
    double lon2 = msg.longitude;
    cout << setprecision(10) << "GPS_ = "<<lat2<<"; "<<lon2<<endl;
    cout << setprecision(10) << "Ori_GPS_ = "<<lat_ori<<"; "<<lon_ori<<endl;

    // define the x-axis point to the North ----- latitude
    double dx = UTC2Map(lat1,lat2,0,0);
    // define the x-axis point to the West ----- longitude
    double dy = UTC2Map(0,0,lon1,lon2);
    double angular = atan2(dy,dx);

    // get map position
    // cout << setprecision(10) << "Diff_GPS = "<<lat2-lat_ori<<"; "<<lon2-lon_ori<<endl;
    double x = UTC2Map(lat_ori,lat2,0,0);
    double y = UTC2Map(0,0,lon_ori,lon2);

    //update the goal oritation
    double lat_goal = goal_gps_.latitude;
    double lon_goal = goal_gps_.longitude;

    double dx_goal = UTC2Map(lat2,lat_goal,0,0);
    double dy_goal = UTC2Map(0,0,lon2,lon_goal);

    goal_pose_.z = atan2(dy_goal,dx_goal);
    
    cout <<"Goal Pose = " << goal_pose_.x << "; "<< goal_pose_.y << "; " << goal_pose_.z <<endl;
    
    robot_pose_.x = x;
    robot_pose_.y = y;
    robot_pose_.z = 0;
    robot_pose_.z = angular;

    gps_current_ = msg;
    cout <<"Current Pose = " << x << "; "<< y <<"; " << angular <<endl;

    ROS_INFO("------------------------------");

    if (move_signal_) {
        contrl_husky();
    }
    
}

void updateOdom(const nav_msgs::Odometry &msg) {
    double vel_thred = THRED;
    geometry_msgs::Twist vel = msg.twist.twist;
    if (fabs(vel.linear.x) > vel_thred || fabs(vel.linear.y) > vel_thred) {
        move_status_ = true;
    }
    else move_status_ = false;
    robot_vel_ = vel;
}

// gps_nav::GoalCollect::Request  &req,
// gps_nav::GoalCollect::Response &res

bool getGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    goal_list_.push(gps_current_);
    return true;
}

bool NextGoalMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    if (goal_list_.empty()) {
        move_signal_ = false;
        ROS_ERROR("No goal gps in the goal list");
        return false;
    }
    move_signal_ = true;
    goal_gps_ = goal_list_.front();
    goal_list_.pop();
    ROS_INFO("----------Go to the next goal--------");
    // get x and y for goal_pose_
    double lat_goal = goal_gps_.latitude;
    double lon_goal = goal_gps_.longitude;
    double lat_ori  = ori_gps_.latitude;
    double lon_ori  = ori_gps_.longitude;

    goal_pose_.x = UTC2Map(lat_ori,lat_goal,0,0);
    goal_pose_.y = UTC2Map(0,0,lon_ori,lon_goal);

    move_signal_ = true;
    return true;
}

bool emergency_stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    move_signal_ = false;
    return true;
}

bool continue_move(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    move_signal_ = true;
    return true;
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
    server_go   = n.advertiseService("/continue_mission",&continue_move);

    ros::spin();
}





