#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>


ros::ServiceClient move_next;
ros::Subscriber sub_status;
ros::Subscriber sub_mission_status;

bool achievement_status_ = false;
bool mission_status_ = false;

void getStatus(const std_msgs::Bool &msg) {
    achievement_status_ = msg.data;
}

void getMissionStatus(const std_msgs::Bool &msg) {
    mission_status_ = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_start");

    ros::NodeHandle n;
    std_srvs::Empty srv;
    move_next = n.serviceClient<std_srvs::Empty>("/move_next_goal");
    sub_status = n.subscribe("/goal_achieve_status",0,&getStatus);    
    sub_mission_status = n.subscribe("/mission_status",0,&getMissionStatus);  

    while(! mission_status_) {
        if (achievement_status_) {
            move_next.call(srv);
            ROS_INFO("-------------Moving to next goal-----------");
        }
    }
    ros::spin();

    return 0;
}