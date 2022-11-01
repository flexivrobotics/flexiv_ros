cmake_minimum_required(VERSION 3.0.2)
project(flexiv_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CartesianState.msg
  CartesianVelocity.msg
  ExternalForce.msg
  JointPosVel.msg
  Mode.msg
  RobotStates.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_runtime
)
