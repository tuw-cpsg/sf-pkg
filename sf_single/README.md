Single Sensor Fusion Package for ROS
====================================

Implementation of estimation methods for low-level sensor fusion in
C++ libraries with an generic ROS node using this framework.

* Author: Denise Ratasich 
* Start: June 2013 
* Keywords: ROS, Sensor Fusion, Data Fusion, Estimation

Implemented methods/algorithms are:
* Kalman Filter
* Particle Filter
* Simple/Weighted Moving Averaging
* Simple Moving Median
* ...

Usage
-----

The ROS node can be configured with a header file and arguments to
implement an estimator for a specific topic.


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