cmake_minimum_required(VERSION 2.8.3)

## Variables.
set(CONFIG_PATH ../${CONFIG_PATH})	# optional from parent CMakeLists.txt
set(NAME ${PROJECT_NAME})

## Find catkin and any catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp sf_msgs)
find_package(sf_estimation REQUIRED)

## System dependencies are found with CMake's conventions.
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

## Declare the configuration library.
add_library(${NAME}_configuration STATIC
  # sources
  src/configuration/configuration.cpp

  # validation
  include/configuration/validation_MovingMedian.h
  include/configuration/validation_MovingAverage.h
  include/configuration/validation_KalmanFilter.h
  include/configuration/validation_ExtendedKalmanFilter.h
  include/configuration/validation_UnscentedKalmanFilter.h
  include/configuration/validation_ParticleFilterSIR.h
  include/configuration/validation.h

  # code generation
  include/configuration/pre_config.h
  include/configuration/post_config.h
  include/configuration/configuration.h

  # user's configuration
  ${CONFIG_PATH}/config.h
  )

###########
## Build ##
###########
## Specify additional locations of header files. Your package
## locations should be listed before other locations.
include_directories(
  include
  ${CONFIG_PATH}

  ${sf_estimation_INCLUDE_DIRS}

  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  )

## Create the executable.
add_executable(${NAME} src/sf_filter.cpp)

## Link the libraries.
target_link_libraries(${NAME}  
  ${NAME}_configuration
  ${sf_estimation_LIBRARIES}
  ${catkin_LIBRARIES}
  )

## Build sf_msgs before this filter.
add_dependencies(${NAME} sf_msgs_gencpp)