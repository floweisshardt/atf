cmake_minimum_required(VERSION 2.8.3)
project(atf_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  atf_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
        TestStatusDetails.msg
)

add_service_files(
    FILES
        TestStatus.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    rospy
    std_msgs
    atf_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
    PROGRAMS
        scripts/atf_server.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
