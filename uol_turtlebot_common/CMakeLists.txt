cmake_minimum_required(VERSION 2.8.3)
project(uol_turtlebot_common)
find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/republish_cmd_vel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
