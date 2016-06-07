@[if DEVELSPACE]@
# set path to generate_tests script in develspace
set(generate_tests_script @(CMAKE_CURRENT_SOURCE_DIR)/scripts/generate_tests.py)
@[else]@
# set path to generate_tests script in installspace
set(generate_tests_script ${atf_core_DIR}/../scripts/generate_tests.py)
@[end if]@


function(atf_test)
    if(CATKIN_ENABLE_TESTING)
        message(STATUS "ATF: executing test generation macro")
        find_package(catkin REQUIRED COMPONENTS
            rostest
            roslaunch)

        execute_process(
            COMMAND python ${generate_tests_script} ${PROJECT_NAME} ${PROJECT_SOURCE_DIR}
            RESULT_VARIABLE generation_result
        )
        if(NOT "${generation_result}" STREQUAL "0")
          message(FATAL_ERROR "-- ATF: generating test files failed: exit_code='${generation_result}'")
        endif()

        set(TEST_GENERATED_PATH ${PROJECT_SOURCE_DIR}/test_generated)

        roslaunch_add_file_check(test_generated)

        add_rostest(test_generated/cleaning.test)

        file(GLOB TEST_NAMES_RECORDING RELATIVE ${TEST_GENERATED_PATH}/recording ${TEST_GENERATED_PATH}/recording/*.test)
        foreach(TEST_NAME_RECORDING ${TEST_NAMES_RECORDING})
            # recording
            set(TARGET_NAME_RECORDING run_tests_${PROJECT_NAME}_rostest_test_generated_recording_${TEST_NAME_RECORDING})
            set(_TARGET_NAME_RECORDING _run_tests_${PROJECT_NAME}_rostest_test_generated_recording_${TEST_NAME_RECORDING})
            string(REPLACE "/" "_" TARGET_NAME_RECORDING ${TARGET_NAME_RECORDING})
            string(REPLACE "/" "_" _TARGET_NAME_RECORDING ${_TARGET_NAME_RECORDING})
            list(APPEND TARGET_NAMES_RECORDING ${TARGET_NAME_RECORDING})
            list(APPEND _TARGET_NAMES_RECORDING ${_TARGET_NAME_RECORDING})
            add_rostest(test_generated/recording/${TEST_NAME_RECORDING} DEPENDENCIES _run_tests_${PROJECT_NAME}_rostest_test_generated_cleaning.test)

            # analysing
            string(REPLACE "recording_" "analysing_" TEST_NAME_ANALYSING ${TEST_NAME_RECORDING})
            set(TARGET_NAME_ANALYSING run_tests_${PROJECT_NAME}_rostest_test_generated_analysing_${TEST_NAME_ANALYSING})
            set(_TARGET_NAME_ANALYSING _run_tests_${PROJECT_NAME}_rostest_test_generated_analysing_${TEST_NAME_ANALYSING})
            string(REPLACE "/" "_" TARGET_NAME_ANALYSING ${TARGET_NAME_ANALYSING})
            string(REPLACE "/" "_" _TARGET_NAME_ANALYSING ${_TARGET_NAME_ANALYSING})
            list(APPEND TARGET_NAMES_ANALYSING ${TARGET_NAME_ANALYSING})
            list(APPEND _TARGET_NAMES_ANALYSING ${_TARGET_NAME_ANALYSING})
            add_rostest(test_generated/analysing/${TEST_NAME_ANALYSING} DEPENDENCIES ${_TARGET_NAME_RECORDING})
        endforeach()

        add_rostest(test_generated/merging.test DEPENDENCIES atf_${PROJECT_NAME}_analysing)
        add_rostest(test_generated/uploading.test DEPENDENCIES atf_${PROJECT_NAME}_merging)

        add_custom_target(atf_${PROJECT_NAME}_cleaning
            COMMAND echo "cccccccccccccccccccccccccccccccleaning"
            DEPENDS
                _run_tests_${PROJECT_NAME}_rostest_test_generated_cleaning.test
        )
        add_custom_target(atf_${PROJECT_NAME}_recording
            COMMAND echo "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrecording"
            DEPENDS 
                atf_${PROJECT_NAME}_cleaning
                ${_TARGET_NAMES_RECORDING}
        )
        add_custom_target(atf_${PROJECT_NAME}_analysing
            COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaanalysing"
            DEPENDS
                atf_${PROJECT_NAME}_recording
                ${_TARGET_NAMES_ANALYSING}
        )
        add_custom_target(atf_${PROJECT_NAME}_merging
            COMMAND echo "mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmerging"
            DEPENDS
                atf_${PROJECT_NAME}_analysing
                _run_tests_${PROJECT_NAME}_rostest_test_generated_merging.test
        )
        add_custom_target(atf_${PROJECT_NAME}_uploading
            COMMAND echo "uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuploading"
            DEPENDS
                atf_${PROJECT_NAME}_merging
                _run_tests_${PROJECT_NAME}_rostest_test_generated_uploading.test
        )
        add_custom_target(atf_${PROJECT_NAME}
            COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaatf"
            DEPENDS
                atf_${PROJECT_NAME}_cleaning
                atf_${PROJECT_NAME}_recording
                atf_${PROJECT_NAME}_analysing
                atf_${PROJECT_NAME}_merging
                atf_${PROJECT_NAME}_uploading
        )
        add_dependencies(run_tests atf_${PROJECT_NAME})
        add_dependencies(run_tests_${PROJECT_NAME} atf_${PROJECT_NAME})

        message(STATUS "ATF: executing test generation macro done!")
    endif()
endfunction()
