cmake_minimum_required(VERSION 2.8.3)
project(gps_waypoint_nav)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
#  robot_localization
  tf
  actionlib
  roslib
)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#target_link_libraries(
#  ${catkin_LIBRARIES}
#  robot_localization
#)

add_executable(gps_waypoint_nav src/gps_waypoint_nav.cpp)
add_dependencies(gps_waypoint_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_waypoint_nav ${catkin_LIBRARIES} )