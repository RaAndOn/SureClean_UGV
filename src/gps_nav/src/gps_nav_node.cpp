
#include <ros/ros.h>
#include <iomanip>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <queue> 
#include <math.h>

using namespace std;

class Gps_nav {
private:
    ros::Subscriber sub_gps_;
    ros::Publisher pub_cmd_;
    ros::Publisher pub_status_;
    ros::ServiceServer server_goal_;
    ros::ServiceServer server_move_;
    ros::ServiceServer server_stop_;
    ros::ServiceServer server_go_;

    nav_msgs::Odometry odom_filtered_goal_;
    nav_msgs::Odometry odom_filtered_current_;
    nav_msgs::Odometry odom_current_;
    // Point x -- east, y -- north and z -- yaw (up)
    geometry_msgs::Point goal_pose_;
    geometry_msgs::Point robot_pose_;
    geometry_msgs::Twist robot_vel_;
    queue<nav_msgs::Odometry> goal_list_;
    
    double magnetic_declination_; 
    bool move_signal_;
    bool move_status_;
    bool ori_status_ ;
    float Kp_;
    float Kd_;

    float ANGULAR_THRED; 
    float LINEAR_THRED; 
    float MOVE_THRED;
    float RADIUS_EARTH;
    float MAX_SPEED;
    float MIN_SPEED;
    float MAX_ANGULAR;
    float MIN_ANGULAR;
    float DIS_RANGE;
    int   AVER_TIME;

public:
    Gps_nav() {


        ANGULAR_THRED = 0.04;
        LINEAR_THRED = 0.1;
        MOVE_THRED = 0.01;
        RADIUS_EARTH = 6378.137;
        MAX_SPEED = 1;
        MIN_SPEED = 0.4;
        MAX_ANGULAR = 0.5;
        MIN_ANGULAR = 0.15;
        DIS_RANGE = 1;
        AVER_TIME = 10;

        Kp_ = 1;
        Kd_ = 0.05;
        move_signal_ = false;
        move_status_ = false;
        ori_status_ = false;

        magnetic_declination_ = -0.16347917327;
    }
    ~Gps_nav() {}

    void Loop() {
        ros::NodeHandle n;
        pub_cmd_ = n.advertise<geometry_msgs::Twist>("/cmd_vel",0);
        pub_status_ = n.advertise<std_msgs::Bool>("/goal_achieve_status",0);
        sub_gps_ = n.subscribe("/odometry/filtered_gps",0,&Gps_nav::Main_CallBack,this);
        server_goal_ = n.advertiseService("/collect_goal",&Gps_nav::getGoal,this);
        server_move_ = n.advertiseService("/move_next_goal",&Gps_nav::NextGoalMove,this);
        server_stop_ = n.advertiseService("/emergency_stop",&Gps_nav::emergency_stop,this);
        server_go_   = n.advertiseService("/continue_mission",&Gps_nav::continue_move,this);
        ros::spin();
    }

    double UTC2Map(double lat1, double lat2, double lon1, double lon2) {
        // usefull function 
        double sign = 1;
        if (lon1 == 0 && lon2 == 0) {
            if (lat2 >= lat1) sign = 1;
            else sign = -1;
        }
        if (lat1 == 0 && lat2 == 0) {
            if (lon2 >= lon1) sign = 1;
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
        if (linear && angular) {
            status_msgs.data = true;
            move_signal_ = false;
            pub_status_.publish(status_msgs);
        } 
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
            ROS_INFO("---- Achieved linear goal ----");
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
	        ROS_INFO("---- Achieved angular goal ----");
        }
        if (fabs(d_angular) > M_PI / 6) {
            ctrl_msg.linear.x = 0;
        }
        pub_cmd_.publish(ctrl_msg);
        cout << "Control output " << ctrl_msg.linear.x << "; "<< ctrl_msg.angular.z << endl;
        status_check(check_linear,check_angular);
    }
   

    void Main_CallBack(const nav_msgs::Odometry msg) {
        // set origin
        double angular = calculateYaw(msg.pose.pose.orientation);
        cout << "Current Orientation: " << angular * 180 / M_PI << endl;
        double y = msg.pose.pose.position.y;
        double x = msg.pose.pose.position.x;
        double dy_goal = odom_filtered_goal_.pose.pose.position.y - msg.pose.pose.position.y;
        double dx_goal = odom_filtered_goal_.pose.pose.position.x - msg.pose.pose.position.x;
        goal_pose_.z = atan2(dy_goal,dx_goal);
        cout <<"Goal Pose = " <<goal_pose_.x<< "; "<< goal_pose_.y << "; " << goal_pose_.z * 180 / M_PI <<endl;
        robot_pose_.x = x;
        robot_pose_.y = y;
	    robot_pose_.z = angular;
        robot_vel_ = msg.twist.twist;
        odom_filtered_current_ = msg;
        cout <<"Current Pose = " << x << "; "<< y <<"; " << robot_pose_.z * 180 / M_PI <<endl;
        if (move_signal_) {
            contrl_husky();
        }
    }

    bool getGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        nav_msgs::Odometry goal;
        goal = odom_filtered_current_;
        bool index = 1;
        while (fabs(robot_vel_.linear.x) <= MOVE_THRED && fabs(robot_vel_.linear.y) <= MOVE_THRED && index < AVER_TIME) {
            goal.pose.pose.position.x += odom_filtered_current_.pose.pose.position.x;
            goal.pose.pose.position.y += odom_filtered_current_.pose.pose.position.y;
            index += 1;
        }
        goal.pose.pose.position.x = goal.pose.pose.position.x / index;
        goal.pose.pose.position.y = goal.pose.pose.position.y / index;
        ROS_INFO("GOAL RECEIEVED!");
        goal_list_.push(goal);
        // goal_list_.push(odom_filtered_current_);
        return true;
    }

    bool NextGoalMove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        if (goal_list_.empty()) {
            move_signal_ = false;
            ROS_ERROR("No goal gps in the goal list");
            return false;
        }
        move_signal_ = true;
        odom_filtered_goal_ = goal_list_.front();
        goal_list_.pop();
        ROS_INFO("---------- Go to the next goal --------");
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
    ros::init(argc, argv, "gps_nav_node");
    Gps_nav gps_ctrl;
    gps_ctrl.Loop();
}





