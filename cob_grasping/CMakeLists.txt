cmake_minimum_required(VERSION 2.8.3)
project(cob_grasping)

find_package(catkin REQUIRED COMPONENTS rospy)

#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS rospy)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/grasping_app.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
