cmake_minimum_required(VERSION 2.8.3)
project(atf_recorder)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    std_msgs
    atf_msgs
)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
    FILES
        RecorderCommand.srv
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
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(
    PROGRAMS
        scripts/atf_recorder.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
