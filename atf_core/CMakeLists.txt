cmake_minimum_required(VERSION 3.0.2)
project(atf_core)

find_package(catkin REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CFG_EXTRAS
        ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(
    PROGRAMS
        scripts/analyser.py
        scripts/cleaner.py
        scripts/generate_tests.py
        scripts/record_tests.py
        scripts/sm_test.py
        test/test_dropbox_uploader.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    PROGRAMS
        scripts/dropbox_uploader.sh
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
