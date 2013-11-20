cmake_minimum_required(VERSION 2.8.3)
project(sf_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  )

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
  OutputEntityStamped.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)