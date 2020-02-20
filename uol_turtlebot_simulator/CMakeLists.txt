cmake_minimum_required(VERSION 2.8.3)
project(uol_turtlebot_simulator)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

add_subdirectory(gzmaze)
if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(90.gazebo_models_uol SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  message(FATAL_ERROR "Only UNIX builds supported.")
#  catkin_add_env_hooks(90.gazebo_models_oru SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

#############
## Install ##
#############

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds FILES_MATCHING PATTERN "*.world")
install(DIRECTORY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds FILES_MATCHING PATTERN "*.mz")
install(DIRECTORY ./models/color_wall_marker ./gzmaze/mouse ./gzmaze/maze_base DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models/)
