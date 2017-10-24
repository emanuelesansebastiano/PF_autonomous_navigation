cmake_minimum_required(VERSION 2.8.3)
project(PF_autonomous_navigation)

find_package(catkin REQUIRED COMPONENTS
  pipefollowing
  std_srvs
  topic_tools 
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    std_srvs
    topic_tools
  DEPENDS
    roscpp
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


