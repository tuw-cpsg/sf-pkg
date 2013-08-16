cmake_minimum_required(VERSION 2.8.3)
project(sf_single)

## Find catkin and any catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)

## Declare the estimation library
add_library(estimation 
  src/estimation/InputValue.cpp
  src/estimation/OutputValue.cpp
  src/estimation/MovingMedian.cpp
  src/estimation/MovingAverage.cpp

  include/estimation/InputValue.h
  include/estimation/OutputValue.h
  include/estimation/IEstimationMethod.h
  include/estimation/methods.h
  include/estimation/MovingMedian.h
  include/estimation/MovingAverage.h
  )
# find_library(estimation 
#   NAMES estimation libestimation #name
#   lib/estimation #path
#   )
# if(estimation_FOUND)
#   message(STATUS "estimation lib found")
# else(estimation_FOUND)
#   message(FATAL_ERROR "estimation lib not found")
# endif(estimation_FOUND)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   InputValue.msg
#   OutputValue.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES estimation
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS Boost
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  )

## cpp executables and libs
add_executable(sf_single src/sf_single.cpp)
target_link_libraries(sf_single 
  estimation 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sf_estimation_node sf_estimation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sf_estimation_node
#   ${catkin_LIBRARIES}
# )