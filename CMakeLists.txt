cmake_minimum_required(VERSION 2.8.3)
project(vsnt_teleop)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  mavros_msgs
  rospy
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs mavros_msgs
    DEPENDS
)

add_executable(vsnt_teleop src/vsnt_teleop.cpp)
target_link_libraries(vsnt_teleop ${catkin_LIBRARIES})
add_dependencies(vsnt_teleop ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/mode_controller_joystick.py
  scripts/mode_controller_ship_console.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS vsnt_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
