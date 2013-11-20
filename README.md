Single Sensor Fusion Package for ROS
====================================

Implementation of estimation methods for low-level sensor fusion in
C++ libraries with a generic ROS node using this framework.

* Author: Denise Ratasich 
* Start: June 2013 
* Keywords: ROS, Sensor Fusion, Data Fusion, State Estimation, Filtering

Implemented methods/algorithms are:

* Weighted Moving Averaging
* Moving Median
* Kalman Filter
* Extended Kalman Filter
* Unscented Kalman Filter
* Sampling Importance Resampling


Installation
------------

These packages depend on Eigen3, so when not installed (you will get
an error when runngin catkin_make if so), get it from
http://eigen.tuxfamily.org/index.php?title=Main_Page. It is sufficient
to copy the directory 'Eigen/' from this archive to
/usr/include/eigen3, i.e. there is no installation necessary.


Directory Structure
-------------------

* sf_estimation: the low-level sensor fusion framework implementing
  the state estimation algorithms or filters respectively.
* sf_msgs: package containing the messages a sf_filter node can
  publish.
* sf_filter: the source for the ROS node with configuration. Copy this
  folder to create another concrete filter.


Usage
-----

As mentioned above *sf_filter* contains a concrete filter which can be
used as a template. Copy this folder to create your own one. The
directory structure of a concrete filter is:

* filter: a directory containing the source for the concrete filter,
  in particular the configuration. Do not modify or move this folder.
* config.h: the filter's configuration header. Change type and
  parameters of your filter here.
* package.xml: the package.xml file for catkin which defines
  properties and dependencies of the package, i.e. of the ROS
  node. Change here the name of the package and add your additional
  message dependencies, e.g. geometry_msgs.
* CMakeLists.txt: the cmake configuration file. You have to modify at
  least the project name, which must equal the specified package name
  in package.xml.


Generating Documentation
------------------------

A configuration file for generating the documentation with doxygen is
provided in the *doc*-folder(s).

1. Change to the *doc*-directory.  

2. Open a terminal window and generate the documentation (doxygen is
   required) with following command: 

   ''doxygen Doxyfile''

The documentation is then located in the generated *html*-folder. To
view the main page open index.html in a browser.