Single Sensor Fusion Package for ROS
====================================

Implementation of estimation methods for low-level sensor fusion in
C++ libraries with a generic ROS node using this framework.

* Author: Denise Ratasich 
* Start: June 2013 
* Keywords: ROS, Sensor Fusion, Data Fusion, Estimation, Filtering,
  Smoothing

Implemented methods/algorithms are:
* Weighted Moving Averaging
* Moving Median
* Kalman Filter
* Extended Kalman Filter
* Unscented Kalman Filter
* Particle Filter


Usage
-----

The ROS node can be configured with a header file and arguments to
implement an estimator for a specific topic.

The header file, called *config.h* is located in the folder
*cfg*. Currently it is a symbolic link to an example configuration
header (relinking is easier than renaming your header-files everytime
you want to change the configuration). However, change *config.h* to
configure the ROS node to be a specific estimator.


Generating Documentation
------------------------

A configuration file for generating the documentation with doxygen is
provided in the *doc*-folder.

1. Change to the *doc*-directory.  

2. Open a terminal window and generate the documentation (doxygen is
   required) with following command: 

   ''doxygen Doxyfile''

The documentation is then located in the generated *html*-folder. To
view the main page open index.html in a browser.