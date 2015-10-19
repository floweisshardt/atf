cmake_minimum_required(VERSION 2.8.3)
project(atf_msgs)

find_package(catkin
    REQUIRED COMPONENTS
        message_generation
        geometry_msgs
        std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
        Status.msg
        Trigger.msg
        IO.msg
        Network.msg
        Resources.msg
        TestStatus.msg
        TestblockStatus.msg
        NodeResources.msg
        ObstacleDistance.msg
        ObstacleDistanceLink.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)
