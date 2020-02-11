cmake_minimum_required(VERSION 2.8.3)
project(atf_presenter)
find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/chromium.sh
  scripts/firefox.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
