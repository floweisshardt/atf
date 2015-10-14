cmake_minimum_required(VERSION 2.8.3)
project(atf_recorder_plugins)

find_package(catkin REQUIRED
    COMPONENTS
        atf_msgs
        rospy
        roscpp
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        tf
)

find_package(Boost REQUIRED
    COMPONENTS
        filesystem
        system
        thread
    REQUIRED)
find_package(fcl REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package(
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning_interface
        roscpp
        atf_msgs
    DEPENDS
        Boost
        fcl
    INCLUDE_DIRS
        include
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
)
link_directories(
    ${catkin_LIBRARY_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
)


add_executable(obstacle_distance_node src/${PROJECT_NAME}/obstacle_distance.cpp include/${PROJECT_NAME}/obstacle_distance.h)
target_link_libraries(obstacle_distance_node ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES})
