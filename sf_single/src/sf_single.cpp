/**
 * @file 
 * @author Denise Ratasich
 * @date 26.07.2013
 *
 * @brief ROS node for using a single sensor fusion method, in
 * particular estimation.
 */


// configuration
#include "configuration/configuration.h"


// cpp includes
#include <iostream>

// ROS includes
#include <ros/ros.h>

// estimation framework includes
#include "estimation/EstimatorFactory.h"
#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/OutputValue.h"
#include "estimation/methods.h"

/** @brief Sample publisher (global because publishing is done in
    callback function); */
ros::Publisher pub_signalFused;

/** @brief Estimator, which does the filtering of a signal. */
estimation::IEstimator* estimator;

/**
 * @brief Called when a message is received.
 */
template <class T>
void received(const T& msg)	// msg is a boost::shared_ptr<T const>!
{ 
  try 
  { 
    // Estimation
    estimation::Input in(estimation::InputValue(msg->TOPIC_FIELD));
    estimation::Output out = estimator->estimate(in);

    // Create fused message.
    std_msgs::Float64 sampleFused;
    sampleFused.data = out.getValue();

    // Send fused message.
    pub_signalFused.publish(sampleFused);
    ROS_DEBUG("sample | sample-fused: %.2f | %.2f", msg->TOPIC_FIELD, sampleFused.data);
  } 
  catch (std::exception& e) 
  {
    ROS_ERROR_STREAM(e.what());
  }
}

/**
 * @brief This node implements a specified sensor fusion method.
 *
 * The method and its parameters can be passed as arguments or in a
 * configuration file. According to these settings an estimation class
 * is instantiated which fuses the subscribed input value. With every
 * new input value the estimate is updated and published.
 */
int main(int argc, char **argv)
{
  try 
  {
    // --------------------------------------------
    // Initialization
    // --------------------------------------------
    // Initialize for ROS.
    ros::init(argc, argv, "sf_single");

    // ROS specific arguments are handled and removed by ros::init(..),
    // f.e. remapping arguments added by roslaunch.

    // Create an estimator according to the configuration header file.
    try {
      estimation::EstimatorFactory eFactory;
      initEstimatorFactory(eFactory);
      estimator = eFactory.create();
      ROS_DEBUG("Estimator initialized.");
    } catch(std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      return 1;
    }
 
    // Create main access point to communicate with the ROS system.
    ros::NodeHandle n;
    
    // Tell ROS that we want to subscribe to the sensor fusion input
    // (the entity to estimate) and publish a fused version of it (the
    // estimate).
    ros::Subscriber sub_signal = n.subscribe<TOPIC_TYPE>(TOPIC_NAME, 20, received);
    ROS_INFO("Subscribing to '%s'.", TOPIC_NAME);
    pub_signalFused = n.advertise<std_msgs::Float64>("signal_fused", 100);
    ROS_INFO("Publishing result to 'signal_fused'.");

    // --------------------------------------------
    // Running application.
    // --------------------------------------------
    // Check repeatedly whether there are samples to estimate.
    ros::spin();	// blocking (until roscore sends "kill")

    // --------------------------------------------
    // Clean up.
    // --------------------------------------------
    delete estimator;

    // Close connections.
    sub_signal.shutdown();
    pub_signalFused.shutdown();
    ROS_INFO("Closed connections. Exit.");
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Unhandled Exception reached the top of main: " 
		     << e.what() << ", application will now exit." << std::endl); 
    return 1; 
  }

  return 0;
}
