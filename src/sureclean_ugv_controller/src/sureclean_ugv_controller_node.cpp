#include <ros/ros.h>
#include <sureclean_ugv_controller/point_and_shoot_controller.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle privateNH("~");
  ros::NodeHandle publicNH("");

  PointAndShootController ugvController(privateNH, publicNH);

  ros::Rate loop_rate(20);
  ROS_INFO("Init Controller node\n");
  while (ros::ok()) {
    ugvController.movementController();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
