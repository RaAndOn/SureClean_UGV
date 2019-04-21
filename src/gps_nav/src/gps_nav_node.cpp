
#include <ros/ros.h>
#include <iomanip>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <queue> 
#include <math.h>

using namespace std;

class Gps_nav {
private:
    ros::Subscriber sub_gps;
    ros::Subscriber sub_odom_filtered;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_imu;
    ros::Publisher pub_cmd;
    ros::Publisher pub_status;
    ros::Publisher pub_mission_status;
    ros::ServiceServer server_goal;
    ros::ServiceServer server_move;
    ros::ServiceServer server_stop;
    ros::ServiceServer server_go;

    sensor_msgs::NavSatFix goal_gps_;
    nav_msgs::Odometry odom_filtered_goal_;
    sensor_msgs::NavSatFix ori_gps_;
    nav_msgs::Odometry odom_filtered_ori_;
    sensor_msgs::NavSatFix gps_current_;
    nav_msgs::Odometry odom_filtered_current_;
    nav_msgs::Odometry odom_current_;
    // Point x -- north, y -- west and z -- yaw
    geometry_msgs::Point goal_pose_;
    geometry_msgs::Point robot_pose_;
    queue<nav_msgs::Odometry> goal_list_;
    
    bool use_imu_;
    double magnetic_declination_;
    double yaw_offset_;
    int ori_index_; 
    bool mission_status_;
    bool status_;
    bool move_signal_;
    bool move_status_;
    bool ori_status_ ;
    double d_yaw_odom_;
    double lat_ori_accu_;
    double lon_ori_accu_;
    double imu_yaw_;
    float Kp_;
    float Kd_;

    float THRED_MOVEMENT;
    float ANGULAR_THRED; 
    float LINEAR_THRED; 
    float RADIUS_EARTH;
    float MAX_SPEED;
    float MIN_SPEED;
    float MAX_ANGULAR;
    float MIN_ANGULAR;
    float DIS_RANGE;
    int   Aver_Time;

public:
    Gps_nav() {

        use_imu_ = false;

        THRED_MOVEMENT = 0.01;
        ANGULAR_THRED = 0.04;
        LINEAR_THRED = 0.1;
        RADIUS_EARTH = 6378.137;
        MAX_SPEED = 1;
        MIN_SPEED = 0.4;
        MAX_ANGULAR = 0.5;
        MIN_ANGULAR = 0.15;
        DIS_RANGE = 1;
        Aver_Time = 50;

        Kp_ = 1;
        Kd_ = 0.05;
        ori_index_ = 0;
        mission_status_ = false;
        status_ = false;
        move_signal_ = false;
        move_status_ = false;
        ori_status_ = false;
        d_yaw_odom_ = 0;
        lat_ori_accu_ = 0;
        lon_ori_accu_ = 0;

        magnetic_declination_ = -0.16347917327;
        yaw_offset_ = 0;
    }
    ~Gps_nav() {}

    void Loop() {
        ros::NodeHandle n;
        pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel",0);
        pub_status = n.advertise<std_msgs::Bool>("/goal_achieve_status",0);
        pub_mission_status = n.advertise<std_msgs::Bool>("/mission_status",0);
        sub_odom = n.subscribe("/husky_velocity_controller/odom",0,&Gps_nav::updateOdomYaw,this);
        sub_imu = n.subscribe("/imu/data_raw",0,&Gps_nav::updateIMUYaw,this);
        sub_gps = n.subscribe("/odometry/filtered_gps",0,&Gps_nav::GPS_CallBack_Main,this);
	//sub_odom_filtered = n.subscribe("/odometry/filtered_gps",0,&Gps_nav::ODOM_CALLBack_Main);
        server_goal = n.advertiseService("/collect_goal",&Gps_nav::getGoal,this);
        server_move = n.advertiseService("/move_next_goal",&Gps_nav::NextGoalMove,this);
        server_stop = n.advertiseService("/emergency_stop",&Gps_nav::emergency_stop,this);
        server_go   = n.advertiseService("/continue_mission",&Gps_nav::continue_move,this);
        ros::spin();
    }

    double UTC2Map(double lat1, double lat2, double lon1, double lon2) {
        double sign = 1;

        if (lon1 == 0 && lon2 == 0) {
            if (lat2 >= lat1) sign = 1;  // heading north
            else sign = -1;
        }
        if (lat1 == 0 && lat2 == 0) {
            if (lon2 >= lon1) sign = 1;  // heading west
            else sign = -1;
        }
        
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

    double calculateYaw(geometry_msgs::Quaternion ori) {
    // Calculate and return the yaw of a pose quaternion
        double roll, pitch, yaw;
        double quatx = ori.x;
        double quaty = ori.y;
        double quatz = ori.z;
        double quatw = ori.w;
        tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
        tf::Matrix3x3 rotMatrix(quaternion);
        rotMatrix.getRPY(roll, pitch, yaw);
        return yaw;
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

    float normalizeAngleDiff(float currAngle, float goalAngle) {
    // This function normalizes the difference between two angles to account for looping
        float diff = goalAngle - currAngle;
        if (diff > M_PI) return diff - 2 * M_PI;
        else if (diff < -M_PI) return diff + 2 * M_PI;
        else return diff;
    }

    void contrl_husky() {
        geometry_msgs::Twist ctrl_msg;
        // PD control of husky

        float goal_angular = goal_pose_.z;
        float current_angular = robot_pose_.z;
        float d_angular = normalizeAngleDiff(current_angular, goal_angular);

        float dx = goal_pose_.x - robot_pose_.x;
        float dy = goal_pose_.y - robot_pose_.y;
        double d_linear = sqrt(pow(dx,2) + pow(dy,2));

        bool check_linear;
        bool check_angular;
        
        if (d_linear > LINEAR_THRED) {
            double ctrl_vel_linear = ctrl_msg.linear.x;
            if (d_linear > DIS_RANGE) {
                d_linear = DIS_RANGE;
            }
            ctrl_msg.linear.x  = Kp_ * MAX_SPEED * d_linear/DIS_RANGE;
            if (ctrl_msg.linear.x < MIN_SPEED) ctrl_msg.linear.x = MIN_SPEED;
            check_linear = false;
        }
        else {
            ctrl_msg.linear.x = 0;
            check_linear = true;
        }

        if (fabs(d_angular) > ANGULAR_THRED && fabs(d_linear) >= DIS_RANGE) {
            double ctrl_vel_angular = ctrl_msg.angular.z;
            if (fabs(d_angular) > M_PI/2) {
                d_angular = M_PI/2;
            }
            // right handed
            ctrl_msg.angular.z =  (Kp_ * MAX_ANGULAR * d_angular / (M_PI/2));
            
            if (fabs(ctrl_msg.angular.z) < MIN_ANGULAR) {
                float angular_sign = ctrl_msg.angular.z / fabs(ctrl_msg.angular.z);
                ctrl_msg.angular.z = angular_sign * MIN_ANGULAR;
            }

            check_angular = false;
        }
        else {
            ctrl_msg.angular.z = 0;
            check_angular = true;
	    ROS_INFO("Angular_Achieve------");
        }

        if (fabs(d_angular) > M_PI / 6) {
            ctrl_msg.linear.x = 0;
        }

        pub_cmd.publish(ctrl_msg);

        cout << "Control output " << ctrl_msg.linear.x << "; "<< ctrl_msg.angular.z << endl;
        status_check(check_linear,check_angular);
    }
   

    void GPS_CallBack_Main(const nav_msgs::Odometry msg) {
        // set origin
        bool odom_yaw = false;
        if (move_status_ == false && ori_index_ <= Aver_Time && ori_status_ == false) {
            //lat_ori_accu_ += msg.latitude;
            //lon_ori_accu_ += msg.longitude;
            ROS_INFO("Initializing Origin --- Robot NOT Moving");
            ROS_INFO("Yaw is not useful right now");
            //use for generate postion
            ori_index_ += 1;
        }

        if ((ori_index_ > Aver_Time || move_status_ == true) && ori_status_ == false) {
            odom_filtered_ori_ = msg;
            //ori_gps_.latitude = lat_ori_accu_ / ori_index_;
            //ori_gps_.longitude = lon_ori_accu_ / ori_index_;
            cout << setprecision(10) <<"Ori_GPS = " <<odom_filtered_ori_.pose.pose.position.x<<"; "<<odom_filtered_ori_.pose.pose.position.y<<"; "<<endl;
            ROS_INFO("------------------Origin Initialization Completed-------------");
            ori_status_ = true;
        }

        //double lat_ori = ori_gps_.latitude;
        //double lon_ori = ori_gps_.longitude;

        // use for yaw generation
        //double lat1 = gps_current_.latitude;
        //double lon1 = gps_current_.longitude;

        // the new gps signal
        //double lat2 = msg.latitude;
        //double lon2 = msg.longitude;
        //cout << setprecision(10) << "Position_current = "<<msg.pose.pose.position.x<<"; "<< msg.pose.pose.position.y<<endl;
        cout << setprecision(10) << "Ori_Position_ = "<<odom_filtered_ori_.pose.pose.position.x<<"; "<<odom_filtered_ori_.pose.pose.position.y<<endl;

        // define the x-axis point to the North ----- latitude
        //double dy = UTC2Map(lat1,lat2,0,0);
        // define the x-axis point to the West ----- longitude
        //double dx = UTC2Map(0,0,lon1,lon2);
        double angular = calculateYaw(msg.pose.pose.orientation);
        //if (fabs(dx) < THRED_MOVEMENT && fabs(dy) < THRED_MOVEMENT) odom_yaw = true;
        //cout << "The output of dx and dy: " << dx <<"; "<< dy << endl;
        cout << "Current Orientation: " << angular * 180 / M_PI << endl;
        // get map position
        // cout << setprecision(10) << "Diff_GPS = "<<lat2-lat_ori<<"; "<<lon2-lon_ori<<endl;
        double y = msg.pose.pose.position.y;
        double x = msg.pose.pose.position.x;

        //update the goal oritation
        //double lat_goal = goal_gps_.latitude;
        //double lon_goal = goal_gps_.longitude;

        double dy_goal = odom_filtered_goal_.pose.pose.position.y - msg.pose.pose.position.y;
        double dx_goal = odom_filtered_goal_.pose.pose.position.x - msg.pose.pose.position.x;

        goal_pose_.z = atan2(dy_goal,dx_goal);
        
        cout <<"Goal Pose = " <<goal_pose_.x<< "; "<< goal_pose_.y << "; " << goal_pose_.z * 180 / M_PI <<endl;
        
        robot_pose_.x = x;
        robot_pose_.y = y;
	robot_pose_.z = angular;

        //if (! use_imu_) { // check whether or not using IMU yaw
        //    if (! odom_yaw) {
        //        robot_pose_.z = angular;
        //    }
        //    else robot_pose_.z += d_yaw_odom_;
        //}
        //else robot_pose_.z = imu_yaw_;
        
        odom_filtered_current_ = msg;
        cout <<"Current Pose = " << x << "; "<< y <<"; " << robot_pose_.z * 180 / M_PI <<endl;

        ROS_INFO("------------------------------");
        std_msgs::Bool mission_msgs;
        mission_msgs.data = mission_status_;
        pub_mission_status.publish(mission_msgs);

        if (move_signal_) {
            contrl_husky();
        }
    }
    
    void updateOdomYaw(const nav_msgs::Odometry msg) {
        double yaw = calculateYaw(msg.pose.pose.orientation);
        d_yaw_odom_ = yaw - calculateYaw(odom_current_.pose.pose.orientation);
        odom_current_ = msg;
    }

    void updateIMUYaw(const sensor_msgs::Imu msg) {
        // update the imu yaw
        imu_yaw_ = calculateYaw(msg.orientation) + magnetic_declination_ + yaw_offset_;
    }

    

    bool getGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        goal_list_.push(odom_filtered_current_);
        return true;
    }

    bool NextGoalMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        if (goal_list_.empty()) {
            move_signal_ = false;
            mission_status_ = true;
            ROS_ERROR("No goal gps in the goal list");
            return false;
        }
        move_signal_ = true;
        odom_filtered_goal_ = goal_list_.front();
        goal_list_.pop();
        ROS_INFO("----------Go to the next goal--------");
        // get x and y for goal_pose_
        //double lat_goal = goal_gps_.latitude;
        //double lon_goal = goal_gps_.longitude;
        //double lat_ori  = ori_gps_.latitude;
       // double lon_ori  = ori_gps_.longitude;

        goal_pose_.y = odom_filtered_goal_.pose.pose.position.y;
        goal_pose_.x = odom_filtered_goal_.pose.pose.position.x;

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
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    Gps_nav gps_ctrl;
    gps_ctrl.Loop();
}





