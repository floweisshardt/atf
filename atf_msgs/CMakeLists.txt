cmake_minimum_required(VERSION 2.8.3)
project(atf_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
        Api.msg
        AtfResult.msg
        DataStamped.msg
        Groundtruth.msg
        IO.msg
        Interface.msg
        InterfaceItem.msg
        KeyValue.msg
        MetricResult.msg
        Network.msg
        NodeApi.msg
        NodeResources.msg
        ObstacleDistance.msg
        ObstacleDistanceLink.msg
        Resources.msg
        TestResult.msg
        TestStatus.msg
        TestblockResult.msg
        TestblockStatus.msg
        TestblockTrigger.msg
)

add_action_files(
    FILES
        TestblockTrigger.action
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
        actionlib_msgs
        message_runtime
        std_msgs
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)
