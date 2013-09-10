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
#include <sstream>

// ROS includes
#include <ros/ros.h>

// estimation framework includes
#include "estimation/EstimatorFactory.h"
#include "estimation/IEstimator.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/Output.h"

/** @brief Collects the messages together. */
struct TopicState {
  /** @brief True when a new message has been received. */
  bool received;
  /** @brief The latest message received, converted into an InputValue
   * for an estimator. **/
  estimation::InputValue value;
};

/** @brief Collects the messages of subscribed topics. */
struct TopicState topics[TOPICS_NUM];

// Expands to the callback functions of the topics.
RECEIVES(topics)

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
    estimation::IEstimator* estimator;
    try {
      estimation::EstimatorFactory eFactory;
      // Pass definitions of the configuration header to the factory.
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
    // (given in the configuration header) and publish a fused version
    // of it (the estimate).
    ros::Subscriber subscribers[TOPICS_NUM];
    SUBSCRIBE(subscribers, n);
    ros::Publisher publishers[TOPICS_NUM];
    for (int i = 0; i < TOPICS_NUM; i++) {
      publishers[i] = n.advertise<std_msgs::Float64>("state_" + std::to_string(i) + "_fused", 100);
      ROS_INFO_STREAM("Publishing result to 'state_" << i << "_fused'.");
    }
    // Create messages for publishing.
    std_msgs::Float64 sampleFused[TOPICS_NUM];

    // --------------------------------------------
    // Running application.
    // --------------------------------------------
    // Check repeatedly whether there are samples to estimate.
    while (ros::ok()) 
    {
      // Check if all samples for an estimation cycle were received.
      bool all_recv = true;
      for (int i = 0; i < TOPICS_NUM; i++) 
	all_recv &= topics[i].received;

      if (all_recv)
      {
	try
	{
	  // Debug: Print jitter of each message/value in ms.
	  std::stringstream ss;
	  ss << "Jitter: ";
	  for (int i = 0; i < TOPICS_NUM; i++)
	    ss << topics[i].value.getJitter() << " ";
	  ROS_DEBUG_STREAM(ss.str());

	  // Collect inputs together.
	  estimation::Input in;
	  for (int i = 0; i < TOPICS_NUM; i++)
	    in.add(topics[i].value);

	  // Estimate.
	  estimation::Output out = estimator->estimate(in);

	  // Set output message(s) and publish.
	  for (int i = 0; i < out.size(); i++) {
	    sampleFused[i].data = out[i].getValue();
	    publishers[i].publish(sampleFused[i]);
	  }
	}
	catch (std::exception& e)
	{
	  ROS_ERROR_STREAM(e.what());
	}

	// Samples/messages are processed, so reset all receive flags.
	for (int i = 0; i < TOPICS_NUM; i++) 
	  topics[i].received = false;
      }
      
      // Handle callbacks (check for next samples).
      ros::spinOnce();
    }

    // --------------------------------------------
    // Clean up.
    // --------------------------------------------
    delete estimator;

    // Close connections.
    //sub_signal.shutdown();
    //pub_signalFused.shutdown();
    ROS_INFO("Exit.");
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Unhandled Exception reached the top of main: " 
		     << e.what() << ", application will now exit." << std::endl); 
    return 1; 
  }

  return 0;
}
