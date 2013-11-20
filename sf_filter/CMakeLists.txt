cmake_minimum_required(VERSION 2.8.3)

## MODIFY. Sets the name of this project. Must be a unique name, since
## it will be used for generating the package name. This name must
## equal the name specified in package.xml.
project(sf_filter)

## Specify the path to the config file. In this template the
## configuration header is put directly into this folder.
set(CONFIG_PATH ./)

## MODIFY. Find catkin and any catkin packages.
find_package(catkin REQUIRED COMPONENTS
  # add the messages you want to be able to receive below:
  geometry_msgs
  )

## DO NOT MODIFY. Sets catkin specific properties, configures this
## package.
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENS
  DEPENDS
  )

## Build sources in subdirectory 'filter'. If you didn't change the
## directory structure, there is no need to change the line. Be
## careful if you do so.
add_subdirectory(filter)
