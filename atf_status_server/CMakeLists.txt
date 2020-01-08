cmake_minimum_required(VERSION 2.8.3)
project(atf_status_server)

find_package(catkin REQUIRED COMPONENTS
    atf_msgs
    message_generation
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
        atf_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    atf_msgs
    message_runtime
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

install(
    PROGRAMS
        scripts/${PROJECT_NAME}_node.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
