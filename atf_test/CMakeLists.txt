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

    file(GLOB TEST_NAMES_RECORDING RELATIVE ${PROJECT_SOURCE_DIR}/test_generated/recording ${PROJECT_SOURCE_DIR}/test_generated/recording/*.test)
    file(GLOB TEST_NAMES_ANALYSING RELATIVE ${PROJECT_SOURCE_DIR}/test_generated/analysing ${PROJECT_SOURCE_DIR}/test_generated/analysing/*.test)

    foreach(TEST_NAME ${TEST_NAMES_RECORDING})
        string(REPLACE "/" "_" TARGET_NAME ${TEST_NAME})
        set(TARGET_NAME _run_tests_${PROJECT_NAME}_rostest_test_generated_recording_${TARGET_NAME})
        list(APPEND TEST_TARGETS_RECORDING ${TARGET_NAME})
        add_rostest(test_generated/recording/${TEST_NAME} DEPENDENCIES atf_generating)
    endforeach()

    foreach(TEST_NAME ${TEST_NAMES_ANALYSING})
        string(REPLACE "/" "_" TARGET_NAME ${TEST_NAME})
        set(TARGET_NAME _run_tests_${PROJECT_NAME}_rostest_test_generated_analysing_${TARGET_NAME})
        list(APPEND TEST_TARGETS_ANALYSING ${TARGET_NAME})
        add_rostest(test_generated/analysing/${TEST_NAME} DEPENDENCIES atf_recording)
    endforeach()

    add_rostest(test_generated/merging.test DEPENDENCIES atf_analysing)
    add_rostest(test_generated/uploading.test DEPENDENCIES atf_merging)

    add_custom_target(atf_generating
        COMMAND echo "geeeeeeeeeeeeeeenerating")
    add_custom_target(atf_recording
        DEPENDS 
            atf_generating
            ${TEST_TARGETS_RECORDING}
    )
    add_custom_target(atf_analysing 
        DEPENDS
            atf_recording
            ${TEST_TARGETS_ANALYSING}
    )
    add_custom_target(atf_merging
        DEPENDS
            atf_analysing
            _run_tests_atf_test_rostest_test_generated_merging.test
    )
    add_custom_target(atf_uploading
        DEPENDS
            atf_merging
            _run_tests_atf_test_rostest_test_generated_uploading.test
    )
    add_custom_target(atf
        DEPENDS
            atf_uploading
            atf_merging
            atf_analysing
            atf_recording
            atf_generating
    )
    add_dependencies(run_tests atf)

endif()
