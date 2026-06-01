cmake_minimum_required(VERSION 3.0.2)
project(chronos_balance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  urdf
  xacro
  gazebo_ros
)

catkin_package()

install(DIRECTORY launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
