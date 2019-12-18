@[if DEVELSPACE]@
# set path to generate_tests script in develspace
set(generate_tests_script @(CMAKE_CURRENT_SOURCE_DIR)/scripts/generate_tests.py)
@[else]@
# set path to generate_tests script in installspace
set(generate_tests_script ${atf_core_DIR}/../scripts/generate_tests.py)
@[end if]@


function(atf_test TEST_GENERATION_CONFIG_FILE)
    if(CATKIN_ENABLE_TESTING)
        message(STATUS "ATF: executing test generation macro")
        
        find_package(roslaunch REQUIRED)
        find_package(rostest REQUIRED)

        ############
        message(STATUS "ATF: generating test files")        
        execute_process(
            COMMAND python ${generate_tests_script} ${PROJECT_NAME} ${TEST_GENERATION_CONFIG_FILE} ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
            RESULT_VARIABLE generation_result
        )
        if(NOT "${generation_result}" STREQUAL "0")
          message(FATAL_ERROR "-- ATF: generating test files failed: exit_code='${generation_result}'")
        endif()

        set(TEST_GENERATED_PATH ${PROJECT_BINARY_DIR}/test_generated)
        
        ############
        message(STATUS "ATF: roslaunch checking test files")
        file(GLOB_RECURSE test_list
            LIST_DIRECTORIES false
            "${TEST_GENERATED_PATH}/*.test"
        )
        foreach(test ${test_list})
            message("roslaunch check for test file: ${test}")
            roslaunch_add_file_check(${test})
        endforeach()

        ############
        message(STATUS "ATF: cleaning")
        add_rostest(${TEST_GENERATED_PATH}/cleaning.test)
        add_custom_target(atf_${PROJECT_NAME}_cleaning
            COMMAND echo "cccccccccccccccccccccccccccccccleaning"
            DEPENDS
                _run_tests_${PROJECT_NAME}_rostest_test_generated_cleaning.test
        )

        ############
        message(STATUS "ATF: recording")
        file(GLOB TEST_NAMES_RECORDING RELATIVE ${TEST_GENERATED_PATH} ${TEST_GENERATED_PATH}/recording_*.test)
        foreach(TEST_NAME_RECORDING ${TEST_NAMES_RECORDING})
            # recording
            set(TARGET_NAME_RECORDING run_tests_${PROJECT_NAME}_rostest_test_generated_${TEST_NAME_RECORDING})
            set(_TARGET_NAME_RECORDING _run_tests_${PROJECT_NAME}_rostest_test_generated_${TEST_NAME_RECORDING})
            string(REPLACE "/" "_" TARGET_NAME_RECORDING ${TARGET_NAME_RECORDING})
            string(REPLACE "/" "_" _TARGET_NAME_RECORDING ${_TARGET_NAME_RECORDING})
            list(APPEND TARGET_NAMES_RECORDING ${TARGET_NAME_RECORDING})
            list(APPEND _TARGET_NAMES_RECORDING ${_TARGET_NAME_RECORDING})
            add_rostest(${TEST_GENERATED_PATH}/${TEST_NAME_RECORDING} DEPENDENCIES _run_tests_${PROJECT_NAME}_rostest_test_generated_cleaning.test)
        endforeach()
        add_custom_target(atf_${PROJECT_NAME}_recording
            COMMAND echo "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrecording"
            DEPENDS 
                atf_${PROJECT_NAME}_cleaning
                ${_TARGET_NAMES_RECORDING}
        )

        ############
        message(STATUS "ATF: analysing")
        add_rostest(${TEST_GENERATED_PATH}/analysing.test DEPENDENCIES ${_TARGET_NAMES_RECORDING})
        add_custom_target(atf_${PROJECT_NAME}_analysing
            COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaanalysing"
            DEPENDS
                atf_${PROJECT_NAME}_recording
                _run_tests_${PROJECT_NAME}_rostest_test_generated_analysing.test
        )
        add_rostest(${TEST_GENERATED_PATH}/uploading.test DEPENDENCIES atf_${PROJECT_NAME}_analysing)

        ############
        message(STATUS "ATF: uploading")
        add_custom_target(atf_${PROJECT_NAME}_uploading
            COMMAND echo "uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuploading"
            DEPENDS
                atf_${PROJECT_NAME}_analysing
                _run_tests_${PROJECT_NAME}_rostest_test_generated_uploading.test
        )
        ############
        message(STATUS "ATF: gathering all atf test steps")
        add_custom_target(atf_${PROJECT_NAME}
            COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaatf"
            DEPENDS
                atf_${PROJECT_NAME}_cleaning
                atf_${PROJECT_NAME}_recording
                atf_${PROJECT_NAME}_analysing
                atf_${PROJECT_NAME}_uploading
        )
        add_dependencies(run_tests atf_${PROJECT_NAME})
        add_dependencies(run_tests_${PROJECT_NAME} atf_${PROJECT_NAME})

        message(STATUS "ATF: executing test generation macro done!")
    endif()
endfunction()
