if(EXISTS "@(CMAKE_INSTALL_PREFIX)/lib/atf_core/generate_tests.py")   # atf_core uses install space
    set(generate_tests_script @(CMAKE_INSTALL_PREFIX)/lib/atf_core/generate_tests.py)
    message("ATF: install space found. generate_tests_script=${generate_tests_script}")
elseif(EXISTS "@(CMAKE_SOURCE_DIR)/scripts/generate_tests.py")   # atf_core uses devel space
    set(generate_tests_script @(CMAKE_SOURCE_DIR)/scripts/generate_tests.py)
    message("ATF: devel space found. generate_tests_script=${generate_tests_script}")
else()   # atf_core not found
    message(FATAL_ERROR "ATF: atf_core not found in install or devel space. CMAKE_SOURCE_DIR=@(CMAKE_SOURCE_DIR), CMAKE_INSTALL_PREFIX=@(CMAKE_INSTALL_PREFIX)")
endif()

function(atf_test TEST_GENERATION_CONFIG_FILE)
    set(ExtraMacroArgs ${ARGN})
    # Get the length of the list
    list(LENGTH ExtraMacroArgs NumExtraMacroArgs)

    # set default
    set(EXECUTE_TESTS True)

    # Execute the following block only if the length is > 0
    if(NumExtraMacroArgs GREATER 0)
        #foreach(ExtraArg ${ExtraMacroArgs})
        #    message( ">>> Element of list of opt args = ${ExtraArg}")
        #endforeach()
        set(EXECUTE_TESTS ${ARGV1})
    endif()

    #message("TEST_GENERATION_CONFIG_FILE = ${TEST_GENERATION_CONFIG_FILE}")
    #message( "EXECUTE_TESTS = ${EXECUTE_TESTS}")

    if(CATKIN_ENABLE_TESTING)
        message("ATF: executing test generation macro with '${TEST_GENERATION_CONFIG_FILE}' and EXECUTE_TESTS=${EXECUTE_TESTS}")
        
        find_package(roslaunch REQUIRED)
        find_package(rostest REQUIRED)

        # prints the cmake targets and its dependencies in the terminal (uncomment for debugging)
        #set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_DEBUG_MODE 1)

        ############
        message(STATUS "ATF: generating test files")
        find_package(PythonInterp)
        execute_process(
            COMMAND ${PYTHON_EXECUTABLE} ${generate_tests_script} ${PROJECT_NAME} ${TEST_GENERATION_CONFIG_FILE} ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
            RESULT_VARIABLE generation_result
        )
        if(NOT "${generation_result}" STREQUAL "0")
          message(FATAL_ERROR "-- ATF: generating test files failed: exit_code='${generation_result}'")
        endif()

        # replace directory "/" with "_"
        string(REPLACE "/" "_" TEST_GENERATION_CONFIG_FILE_REPLACED ${TEST_GENERATION_CONFIG_FILE})
        # replace *.yaml with *_yaml
        string(REPLACE "." "_" TEST_GENERATION_CONFIG_FILE_REPLACED ${TEST_GENERATION_CONFIG_FILE_REPLACED})
        set(TARGET_NAME ${PROJECT_NAME}_${TEST_GENERATION_CONFIG_FILE_REPLACED})
        set(TEST_GENERATED_PATH ${PROJECT_BINARY_DIR}/test_generated/${TEST_GENERATION_CONFIG_FILE_REPLACED})
        
        if(EXECUTE_TESTS)
            ############ roslaunch checks
            message(STATUS "ATF: roslaunch checking test files")
            file(GLOB_RECURSE test_list
                LIST_DIRECTORIES false
                "${TEST_GENERATED_PATH}/*.test"
            )
            foreach(test ${test_list})
                message("roslaunch check for test file: ${test}")
                roslaunch_add_file_check(${test})
            endforeach()

            ############ cleaning
            message(STATUS "ATF: cleaning")
            add_rostest(${TEST_GENERATED_PATH}/cleaning.test)
            add_custom_target(atf_${TARGET_NAME}_cleaning
                COMMAND echo "cccccccccccccccccccccccccccccccleaning"
                DEPENDS
                    _run_tests_${PROJECT_NAME}_rostest_test_generated_${TEST_GENERATION_CONFIG_FILE_REPLACED}_cleaning.test
            )

            ############ recording
            message(STATUS "ATF: recording")
            file(GLOB TEST_NAMES_RECORDING RELATIVE ${TEST_GENERATED_PATH} ${TEST_GENERATED_PATH}/recording_*.test)
            foreach(TEST_NAME_RECORDING ${TEST_NAMES_RECORDING})
                set(TARGET_NAME_RECORDING _run_tests_${PROJECT_NAME}_rostest_test_generated_${TEST_GENERATION_CONFIG_FILE_REPLACED}_${TEST_NAME_RECORDING})
                list(APPEND TARGET_NAMES_RECORDING ${TARGET_NAME_RECORDING})
                add_rostest(${TEST_GENERATED_PATH}/${TEST_NAME_RECORDING} DEPENDENCIES atf_${TARGET_NAME}_cleaning)
            endforeach()
            add_custom_target(atf_${TARGET_NAME}_recording
                COMMAND echo "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrecording"
                DEPENDS 
                    atf_${TARGET_NAME}_cleaning
                    ${TARGET_NAMES_RECORDING}
            )

            ############ analysing
            message(STATUS "ATF: analysing")
            add_rostest(${TEST_GENERATED_PATH}/analysing.test DEPENDENCIES atf_${TARGET_NAME}_recording)
            add_custom_target(atf_${TARGET_NAME}_analysing
                COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaanalysing"
                DEPENDS
                    atf_${TARGET_NAME}_recording
                    _run_tests_${PROJECT_NAME}_rostest_test_generated_${TEST_GENERATION_CONFIG_FILE_REPLACED}_analysing.test
            )
            add_rostest(${TEST_GENERATED_PATH}/uploading.test DEPENDENCIES atf_${TARGET_NAME}_analysing)

            ############ uploading
            message(STATUS "ATF: uploading")
            add_custom_target(atf_${TARGET_NAME}_uploading
                COMMAND echo "uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuploading"
                DEPENDS
                    atf_${TARGET_NAME}_analysing
                    _run_tests_${PROJECT_NAME}_rostest_test_generated_${TEST_GENERATION_CONFIG_FILE_REPLACED}_uploading.test
            )
            ############ all
            message(STATUS "ATF: gathering all atf test steps")
            add_custom_target(atf_${TARGET_NAME}
                COMMAND echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaatf"
                DEPENDS
                    atf_${TARGET_NAME}_cleaning
                    atf_${TARGET_NAME}_recording
                    atf_${TARGET_NAME}_analysing
                    atf_${TARGET_NAME}_uploading
            )

        add_dependencies(run_tests                 atf_${TARGET_NAME})
        add_dependencies(run_tests_${PROJECT_NAME} atf_${TARGET_NAME})

        endif()

        message(STATUS "ATF: executing test generation macro done!")
    endif()
endfunction()
