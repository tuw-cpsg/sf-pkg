cmake_minimum_required(VERSION 2.8.3)
project(sf_estimation)

## Find catkin - needed for build.
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

## Specify catkin-specific information to the build system which in
## turn is used to generate pkg-config and CMake files.
## INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
## LIBRARIES - The exported libraries from the project
## CATKIN_DEPENDS - Other catkin projects that this project depends on
## DEPENDS - Non-catkin CMake projects that this project depends on
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS Boost Eigen
  )

## Declare the estimation library
add_library(${PROJECT_NAME} SHARED
  # estimation
  src/estimation/Value.cpp
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
  src/estimation/AbstractParticleFilter.cpp
  src/estimation/ParticleFilterSIR.cpp
  src/estimation/ConfidenceWeightedAveraging.cpp

  include/estimation/EstimatorInterface.h
  include/estimation/Value.h
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
  include/estimation/AbstractParticleFilter.h
  include/estimation/ParticleFilterSIR.h
  include/estimation/ConfidenceWeightedAveraging.h

  # probabilities
  src/probability/pdfs.cpp
  src/probability/sampling.cpp
  
  include/probability/pdfs.h
  include/probability/sampling.h
  )

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  )

##########
## Test ##
##########
## Specify tests to build
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest 
    tests/utest_main.cpp
    tests/utest_OutputValue.cpp
    tests/utest_InputValue.cpp
    tests/utest_EstimatorInterface.cpp
    tests/utest_EstimatorFactory.cpp
    tests/utest_KalmanFilter.cpp
    tests/utest_ExtendedKalmanFilter.cpp
    tests/utest_UnscentedTransform.cpp
    tests/utest_UnscentedKalmanFilter.cpp
    tests/utest_pdfs.cpp
    tests/utest_sampling.cpp
    tests/utest_ParticleFilterSIR.cpp
    tests/utest_ConfidenceWeightedAveraging.cpp
    tests/utest_log.cpp
    )
  target_link_libraries(utest
    sf_estimation
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
endif()