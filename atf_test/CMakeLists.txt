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

    file(GLOB TESTS_FILES_RECORDING test_generated/recording/*.test)
    file(GLOB TESTS_FILES_ANALYSING test_generated/analysing/*.test)

    add_custom_target(generating
        COMMAND echo "geeeeeeeeeeeeeeenerating")
    add_custom_target(recording DEPENDS 
        generating
        _run_tests_atf_test_rostest_test_generated_recording_recording_ts1_t1_1.test
        _run_tests_atf_test_rostest_test_generated_recording_recording_ts1_t1_2.test
    )
    add_custom_target(analysing DEPENDS
        recording
        _run_tests_atf_test_rostest_test_generated_analysing_analysing_ts1_t1_1.test
        _run_tests_atf_test_rostest_test_generated_analysing_analysing_ts1_t1_2.test
    )
    add_custom_target(merging DEPENDS
        analysing
        _run_tests_atf_test_rostest_test_generated_merging.test
    )
    add_custom_target(uploading DEPENDS
        merging
        _run_tests_atf_test_rostest_test_generated_uploading.test
    )
    add_custom_target(atf DEPENDS uploading merging analysing recording)
    add_dependencies(run_tests atf)



    #foreach(TEST ${TESTS_FILES_RECORDING})
    #    add_rostest(${TEST} DEPENDS recording)
    #endforeach()
    #foreach(TEST ${TESTS_FILES_ANALYSING})
    #    add_rostest(${TEST} DEPENDS analysing)
    #endforeach()
    add_rostest(test_generated/recording/recording_ts1_t1_1.test DEPENDENCIES generating)
    add_rostest(test_generated/recording/recording_ts1_t1_2.test DEPENDENCIES generating)
    add_rostest(test_generated/analysing/analysing_ts1_t1_1.test DEPENDENCIES recording)
    add_rostest(test_generated/analysing/analysing_ts1_t1_2.test DEPENDENCIES recording)
    add_rostest(test_generated/merging.test DEPENDENCIES analysing)
    add_rostest(test_generated/uploading.test DEPENDENCIES merging)
endif()
