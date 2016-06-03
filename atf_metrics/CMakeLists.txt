cmake_minimum_required(VERSION 2.8.3)
project(atf_metrics)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf
#    obstacle_distance
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    rospy
#    obstacle_distance
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
