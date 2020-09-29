cmake_minimum_required(VERSION 2.8.3)
project(atf_plotter)

find_package(catkin REQUIRED)

catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS
        scripts/plot.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
