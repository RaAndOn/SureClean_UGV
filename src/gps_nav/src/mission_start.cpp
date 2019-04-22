#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>


ros::ServiceClient move_next;
ros::Subscriber sub_status;

void getStatus(const std_msgs::Bool &msg) {
    std_srvs::Empty srv;
    if (move_next.call(srv)) {
        ROS_INFO("----------Moving to the next goal-------------");
    }
    else ROS_INFO("----------Mission Complete------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_start");
    ros::NodeHandle n;
    move_next = n.serviceClient<std_srvs::Empty>("/move_next_goal");
    sub_status = n.subscribe("/goal_achieve_status",0,&getStatus);      
    ros::spin();
    return 0;
}