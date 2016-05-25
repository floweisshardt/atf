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
        COMMAND python ${atf_core_SCRIPTS_DIR}/generate_tests.py ${PROJECT_NAME} ${PROJECT_SOURCE_DIR}
        RESULT_VARIABLE generation_result
    )
    if(NOT "${generation_result}" STREQUAL "0")
      message(FATAL_ERROR "ATF: generating test files failed: exit_code='${generation_result}'")
    endif()

    add_custom_target(${PROJECT_NAME}_recording ALL)
    add_custom_target(${PROJECT_NAME}_analysing DEPENDS ${PROJECT_NAME}_recording)
    add_custom_target(${PROJECT_NAME}_merging DEPENDS ${PROJECT_NAME}_analysing)
    add_custom_target(${PROJECT_NAME}_uploading DEPENDS ${PROJECT_NAME}_merging)

    file(GLOB TESTS_FILES_RECORDING test_generated/recording/*.test)
    file(GLOB TESTS_FILES_ANALYSING test_generated/analysing/*.test)
    foreach(TEST ${TESTS_FILES_RECORDING})
        add_rostest(${TEST} DEPENDENCIES ${PROJECT_NAME}_recording)
    endforeach()
    foreach(TEST ${TESTS_FILES_ANALYSING})
        add_rostest(${TEST} DEPENDENCIES ${PROJECT_NAME}_analysing)
    endforeach()
    add_rostest(test_generated/merging.test DEPENDENCIES ${PROJECT_NAME}_merging)
    add_rostest(test_generated/uploading.test DEPENDENCIES ${PROJECT_NAME}_uploading)
endif()
