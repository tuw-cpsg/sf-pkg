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

#include "sf_single/OutputEntityStamped.h"

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
struct TopicState topics[TOPICS_IN_NUM];

// Expands to the callback functions of the topics.
RECEIVES(topics)

/**
 * @brief This node implements a specified sensor fusion method.
 *
 * According to the settings in the configuration header an estimation
 * class is instantiated which fuses the subscribed input
 * value(s). With every new (set of) input value(s) the estimate is
 * updated and published.
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
    // of it, i.e. the estimate(s).
    ros::Subscriber subscribers[TOPICS_IN_NUM];
    SUBSCRIBE(subscribers, n);
    ros::Publisher publishers[TOPICS_OUT_NUM];
    PUBLISH_INIT(publishers, n);
    // Create messages for publishing.
    sf_single::OutputEntityStamped sampleFused[TOPICS_OUT_NUM];

    // --------------------------------------------
    // Running application.
    // --------------------------------------------
    // Check repeatedly whether there are samples to estimate.
    while (ros::ok()) 
    {
      // Check if all samples for an estimation cycle were received.
      bool all_recv = true;
      for (int i = 0; i < TOPICS_IN_NUM; i++) 
	all_recv &= topics[i].received;

      if (all_recv)
      {
	try
	{
	  // Debug: Print jitter of each message/value in ms.
	  std::stringstream ss;
	  ss << "Jitter: ";
	  for (int i = 0; i < TOPICS_IN_NUM; i++)
	    ss << topics[i].value.getJitter() << " ";
	  ROS_DEBUG_STREAM(ss.str());

	  // Collect inputs together.
	  estimation::Input in;
	  for (int i = 0; i < TOPICS_IN_NUM; i++)
	    in.add(topics[i].value);

	  // Estimate.
	  estimation::Output out = estimator->estimate(in);

	  // Set output message(s) and publish.
	  PUBLISH(publishers, out, sampleFused);
	}
	catch (std::exception& e)
	{
	  ROS_ERROR_STREAM(e.what());
	}

	// Samples/messages are processed, so reset all receive flags.
	for (int i = 0; i < TOPICS_IN_NUM; i++) 
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
