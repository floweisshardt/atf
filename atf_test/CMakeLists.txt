cmake_minimum_required(VERSION 2.8.3)
project(atf_test)

find_package(catkin REQUIRED)

catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/application.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(atf_core REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  roslaunch_add_file_check(launch)
  atf_test(atf/test_generation_config.yaml)
endif()
