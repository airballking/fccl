cmake_minimum_required(VERSION 2.8.3)
project(fccl_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  geometry_msgs)

add_message_files(DIRECTORY msg FILES
  Feature.msg
  Constraint.msg
  KinematicChain.msg)

add_action_files(DIRECTORY action FILES
  SingleArmMotion.action)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs)

catkin_package(CATKIN_DEPENDS 
  actionlib_msgs 
  message_runtime
  std_msgs
  geometry_msgs)
