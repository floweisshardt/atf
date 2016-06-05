cmake_minimum_required(VERSION 2.8.3)
project(atf_recorder_plugins)

find_package(catkin REQUIRED COMPONENTS
    atf_msgs
    rospy
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    atf_msgs
    rospy
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
