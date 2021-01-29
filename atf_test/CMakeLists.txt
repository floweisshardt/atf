cmake_minimum_required(VERSION 3.0.2)
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

install(DIRECTORY atf launch test
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/manual_execution.test)
#endif()

if(CATKIN_ENABLE_TESTING)
  find_package(atf_core REQUIRED)
  atf_test(atf/test_generation_config.yaml)
  atf_test(atf/test_generation_config_extended.yaml False)
endif()
