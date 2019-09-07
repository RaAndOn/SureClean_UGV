#include <sureclean_ugv_planner/coverage_planner.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle privateNH("~");
  ros::NodeHandle publicNH("");

  CoveragePlanner coveragePlanner(privateNH, publicNH);

  ros::Rate loop_rate(20);
  ROS_INFO("Init Planner node\n");
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}