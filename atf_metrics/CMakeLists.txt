cmake_minimum_required(VERSION 2.8.3)
project(atf_metrics)

find_package(catkin REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########
# noting to build, this is only python

#############
## Install ##
#############

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
