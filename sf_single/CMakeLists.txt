cmake_minimum_required(VERSION 2.8.3)
project(sf_single)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

## Declare the estimation library
add_library(estimation
  src/estimation/InputValue.cpp
  src/estimation/Input.cpp
  src/estimation/OutputValue.cpp
  src/estimation/Output.cpp
  src/estimation/EstimatorFactory.cpp
  src/estimation/IEstimator.cpp
  src/estimation/MovingMedian.cpp
  src/estimation/MovingAverage.cpp
  src/estimation/AbstractKalmanFilter.cpp
  src/estimation/KalmanFilter.cpp
  src/estimation/ExtendedKalmanFilter.cpp
  src/estimation/UnscentedTransform.cpp
  src/estimation/UnscentedKalmanFilter.cpp

  include/estimation/EstimatorInterface.h
  include/estimation/InputValue.h
  include/estimation/Input.h
  include/estimation/OutputValue.h
  include/estimation/Output.h
  include/estimation/EstimatorFactory.h
  include/estimation/IEstimator.h
  include/estimation/methods.h
  include/estimation/MovingMedian.h
  include/estimation/MovingAverage.h
  include/estimation/AbstractKalmanFilter.h
  include/estimation/KalmanFilter.h
  include/estimation/ExtendedKalmanFilter.h
  include/estimation/ITransformer.h
  include/estimation/UnscentedTransform.h
  include/estimation/UnscentedKalmanFilter.h
  )

add_library(configuration STATIC
  src/configuration/configuration.cpp

  include/configuration/validation_MovingMedian.h
  include/configuration/validation_MovingAverage.h
  include/configuration/validation_KalmanFilter.h
  include/configuration/validation_ExtendedKalmanFilter.h
  include/configuration/validation_UnscentedKalmanFilter.h
  include/configuration/validation.h

  include/configuration/pre_config.h
  include/configuration/post_config.h
  include/configuration/configuration.h
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
add_message_files(
  FILES
  OutputEntityStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
  LIBRARIES estimation misc
  CATKIN_DEPENDS message_runtime
  DEPENDS Boost Eigen
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  cfg
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  )

## cpp executables and libs
add_executable(sf_single src/sf_single.cpp)
target_link_libraries(sf_single 
  estimation 
  configuration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )
add_dependencies(sf_single sf_single_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sf_estimation_node
#   ${catkin_LIBRARIES}
# )

##########
## Test ##
##########
## Specify tests to build
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest 
    tests/utest_main.cpp
    tests/utest_EstimatorFactory.cpp
    tests/utest_KalmanFilter.cpp
    tests/utest_ExtendedKalmanFilter.cpp
    tests/utest_UnscentedTransform.cpp
    tests/utest_UnscentedKalmanFilter.cpp
    )
  target_link_libraries(utest
    estimation
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
endif()