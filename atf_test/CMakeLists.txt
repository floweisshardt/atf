cmake_minimum_required(VERSION 2.8.3)
project(atf_test)

find_package(catkin REQUIRED COMPONENTS
  atf_core
  rospy
)
catkin_package(
   CATKIN_DEPENDS rospy
)

###########
## Build ##
###########

#############
## Install ##
#############

install(PROGRAMS scripts/application.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    execute_process(
        COMMAND rosrun atf_core generate_tests.py ${PROJECT_SOURCE_DIR}/config/test_generation_config.yaml ${PROJECT_NAME}
    )

    file(GLOB TESTS_FILES_RECORDING test_generated/recording/*.test)
    file(GLOB TESTS_FILES_ANALYSING test_generated/analysing/*.test)
    foreach(TEST ${TESTS_FILES_RECORDING})
        add_rostest(${TEST})
    endforeach()
    foreach(TEST ${TESTS_FILES_ANALYSING})
        add_rostest(${TEST})
    endforeach()
endif()
