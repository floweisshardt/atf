cmake_minimum_required(VERSION 2.8.3)
project(atf_status_server)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    std_msgs
    atf_msgs
    rosbridge_server
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
    FILES
        GetTestStatus.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
        atf_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    message_runtime
    rospy
    std_msgs
    atf_msgs
    rosbridge_server
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
    PROGRAMS
        scripts/atf_status_server_node.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
