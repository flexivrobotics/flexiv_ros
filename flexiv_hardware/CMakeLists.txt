cmake_minimum_required(VERSION 3.5)
project(flexiv_hardware)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_FOR_ARM64 "Link to RDK library for arm64 processor, otherwise link to x64" OFF)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  flexiv_msgs
  hardware_interface
  joint_limits_interface
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    controller_manager
    flexiv_msgs
    hardware_interface
    joint_limits_interface
    pluginlib
    realtime_tools
    roscpp
    std_msgs
    urdf
  LIBRARIES
    flexiv_hardware_interface
)

add_library(FlexivRdk SHARED IMPORTED)
if (${BUILD_FOR_ARM64})
  set_target_properties(FlexivRdk PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/rdk/lib/libFlexivRdk.arm64-darwin.a)
else()
  set_target_properties(FlexivRdk PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/rdk/lib/libFlexivRdk.x86_64-linux-gnu.a)
endif()

add_library(flexiv_hardware_interface
  src/flexiv_hardware_interface.cpp
)
target_include_directories(
  flexiv_hardware_interface
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rdk/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(flexiv_hardware_interface
  PRIVATE
  FlexivRdk
  ${catkin_LIBRARIES}
)


add_executable(flexiv_hardware_interface_node
  src/flexiv_hardware_interface.cpp
  src/flexiv_hardware_interface_node.cpp
)
target_include_directories(
  flexiv_hardware_interface_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rdk/include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(flexiv_hardware_interface_node
  PRIVATE
  FlexivRdk
  ${catkin_LIBRARIES}
)

# INSTALL
install(
  TARGETS
    flexiv_hardware_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executable
install(
  TARGETS
    flexiv_hardware_interface_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
