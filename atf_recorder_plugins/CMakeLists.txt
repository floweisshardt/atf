cmake_minimum_required(VERSION 2.8.3)
project(atf_recorder_plugins)

find_package(catkin REQUIRED COMPONENTS
    rospy
    atf_msgs
    atf_recorder
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    rospy
    atf_msgs
    atf_recorder
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)