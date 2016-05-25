@[if DEVELSPACE]@
# set path to generate_tests script in develspace
set(generate_tests_script @(CMAKE_CURRENT_SOURCE_DIR)/scripts/generate_tests.py)
@[else]@
# set path to generate_tests script in installspace
set(generate_tests_script ${atf_core_DIR}/../scripts/generate_tests.py)
@[end if]@


function(atf_test)
    if(CATKIN_ENABLE_TESTING)
        message("--ATF: executing macro")
        find_package(rostest REQUIRED)

        execute_process(
            COMMAND python ${generate_tests_script} ${PROJECT_NAME} ${PROJECT_SOURCE_DIR}
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
            add_rostest(test_generated/recording/${TEST_NAME} DEPENDENCIES atf_${PROJECT_NAME}_generating)
        endforeach()

        foreach(TEST_NAME ${TEST_NAMES_ANALYSING})
            string(REPLACE "/" "_" TARGET_NAME ${TEST_NAME})
            set(TARGET_NAME _run_tests_${PROJECT_NAME}_rostest_test_generated_analysing_${TARGET_NAME})
            list(APPEND TEST_TARGETS_ANALYSING ${TARGET_NAME})
            add_rostest(test_generated/analysing/${TEST_NAME} DEPENDENCIES atf_${PROJECT_NAME}_recording)
        endforeach()

        add_rostest(test_generated/merging.test DEPENDENCIES atf_${PROJECT_NAME}_analysing)
        add_rostest(test_generated/uploading.test DEPENDENCIES atf_${PROJECT_NAME}_merging)

        add_custom_target(atf_${PROJECT_NAME}_generating
            COMMAND echo "gggggggggggggggggggggggggggggenerating"
        )
        add_custom_target(atf_${PROJECT_NAME}_recording
            COMMAND echo "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrecording"
            DEPENDS 
                atf_${PROJECT_NAME}_generating
                ${TEST_TARGETS_RECORDING}
        )
        add_custom_target(atf_${PROJECT_NAME}_analysing
            COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaanalysing"
            DEPENDS
                atf_${PROJECT_NAME}_recording
                ${TEST_TARGETS_ANALYSING}
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
                atf_${PROJECT_NAME}_uploading
                atf_${PROJECT_NAME}_merging
                atf_${PROJECT_NAME}_analysing
                atf_${PROJECT_NAME}_recording
                atf_${PROJECT_NAME}_generating
        )
        add_dependencies(run_tests atf_${PROJECT_NAME})

        message("--ATF: executing macro done!")
    endif()
endfunction()
