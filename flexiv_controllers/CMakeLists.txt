cmake_minimum_required(VERSION 3.5)
project(flexiv_controllers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  flexiv_msgs
  hardware_interface
  joint_limits_interface
  pluginlib
  realtime_tools
  roscpp
  urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    controller_interface
    controller_manager
    flexiv_msgs
    hardware_interface
    joint_limits_interface
    pluginlib
    realtime_tools
    roscpp
    urdf
  LIBRARIES
    ${PROJECT_NAME}
)

# Build
add_library(${PROJECT_NAME}
  src/joint_impedance_controller.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

# INSTALL
# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install plugins
install(FILES flexiv_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
