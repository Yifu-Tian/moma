cmake_minimum_required(VERSION 2.8.3)
project(diffusion_arm_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  remani_diffusion_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    remani_diffusion_msgs
)

catkin_install_python(PROGRAMS
  scripts/diffusion_arm_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

