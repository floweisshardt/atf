cmake_minimum_required(VERSION 2.8.3)
project(atf_core)

find_package(catkin REQUIRED COMPONENTS
    rospy
    atf_msgs
    atf_recorder
    atf_metrics
    atf_recorder_plugins
    atf_status_server
#    obstacle_distance
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
    rospy
    atf_msgs
    atf_recorder
    atf_metrics
    atf_recorder_plugins
    atf_status_server
#    obstacle_distance
)

catkin_package(CFG_EXTRAS atf_core-extras.cmake)

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
        scripts/analyser.py
        scripts/analyse_all.py
        scripts/record_all.py
        scripts/merger.py
        scripts/dropbox_uploader.sh
        test/test_dropbox_uploader.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        scripts/generate_tests.py
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
