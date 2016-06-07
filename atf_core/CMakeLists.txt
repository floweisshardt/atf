cmake_minimum_required(VERSION 2.8.3)
project(atf_core)

find_package(catkin REQUIRED COMPONENTS
    atf_metrics
    atf_msgs
    atf_recorder_plugins
    atf_status_server
#    obstacle_distance
    rospy
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
        atf_metrics
        atf_msgs
        atf_recorder_plugins
        atf_status_server
    #    obstacle_distance
        rospy
    CFG_EXTRAS
        ${PROJECT_NAME}-extras.cmake
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
        scripts/analyse_all.py
        scripts/analyser.py
        scripts/cleaner.py
        scripts/dropbox_uploader.sh
        scripts/merger.py
        scripts/record_all.py
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
