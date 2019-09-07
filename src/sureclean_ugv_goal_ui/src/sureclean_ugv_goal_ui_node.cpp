
#include <ros/ros.h>
#include <sureclean_ugv_goal_ui/goal_ui.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sureclean_ugv_goal_ui_node");
  ros::NodeHandle privateNH("~");
  ros::NodeHandle publicNH("");
  GoalUI goalUI(privateNH, publicNH);
  ros::Rate loop_rate(20);
  ROS_INFO("Init Goal UI node\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
