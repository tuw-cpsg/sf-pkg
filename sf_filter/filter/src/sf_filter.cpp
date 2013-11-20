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

#include "sf_msgs/OutputEntityStamped.h"

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

/** @brief Collects the messages of subscribed topics_meas. */
struct TopicState topics_meas[TOPICS_IN_MEAS_NUM];

// Expands to the callback functions of the topics representing
// measurements.
RECEIVES(TOPICS_IN, topics_meas, received_meas_)

// Check if control input topics are defined (control inputs are
// optional and not available in simple estimators like the Moving
// Median). If defined, these topics must be subscribed too.
#ifdef TOPICS_IN_CTRL
/** @brief Collects the messages of subscribed topics_ctrl. */
struct TopicState topics_ctrl[TOPICS_IN_CTRL_NUM];

// Expands to the callback functions of the topics representing the
// control input.
RECEIVES(TOPICS_IN_CTRL, topics_ctrl, received_ctrl_)
#endif

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
    ros::init(argc, argv, "sf_filter");

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
    ros::NodeHandle n("~");
    
    // Tell ROS that we want to subscribe to the inputs (given in the
    // configuration header) and publish a fused version of it,
    // i.e. the estimate(s).
    // Measurement inputs.
    ros::Subscriber subscribers_meas[TOPICS_IN_MEAS_NUM];
    SUBSCRIBE(TOPICS_IN, subscribers_meas, n, received_meas_);

    // Control inputs are optional.
#ifdef TOPICS_IN_CTRL
    ros::Subscriber subscribers_ctrl[TOPICS_IN_CTRL_NUM];
    SUBSCRIBE(TOPICS_IN_CTRL, subscribers_ctrl, n, received_ctrl_);
#endif

    // Estimates to output.
    ros::Publisher publishers[TOPICS_OUT_NUM];
    PUBLISH_INIT(publishers, n);
    // Create messages for publishing.
    sf_msgs::OutputEntityStamped sampleFused[TOPICS_OUT_NUM];

    // --------------------------------------------
    // Running application.
    // --------------------------------------------
    // Check repeatedly whether there are samples to estimate.
    while (ros::ok()) 
    {
      // Control inputs are optional.
#ifdef TOPICS_IN_CTRL
      // Check if all control input variables were received.
      bool all_ctrl_recv = true;
      for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++)
	all_ctrl_recv &= topics_ctrl[i].received;
      
      if (all_ctrl_recv)
      {
	// Debug: Print jitter of each message/value in ms.
	std::stringstream ss;
	ss << "Ctrl-Jitter: ";
	for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++)
	  ss << topics_ctrl[i].value.getJitter() << " ";
	ROS_DEBUG_STREAM(ss.str());

	// Collect inputs together.
	estimation::Input in_ctrl;
	for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++)
	  in_ctrl.add(topics_ctrl[i].value);

	// Change control input of estimator.
	estimator->setControlInput(in_ctrl);

	// Control messages are processed, so reset all receive flags.
	for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++) 
	  topics_ctrl[i].received = false;
      }
#endif

      // Check if all samples for an estimation cycle were received.
      bool all_meas_recv = true;
      for (int i = 0; i < TOPICS_IN_MEAS_NUM; i++) 
	all_meas_recv &= topics_meas[i].received;

      if (all_meas_recv)
      {
	try
	{
	  // Debug: Print jitter of each message/value in ms.
	  std::stringstream ss;
	  ss << "Meas-Jitter: ";
	  for (int i = 0; i < TOPICS_IN_MEAS_NUM; i++)
	    ss << topics_meas[i].value.getJitter() << " ";
	  ROS_DEBUG_STREAM(ss.str());

	  // Collect inputs together.
	  estimation::Input in_meas;
	  for (int i = 0; i < TOPICS_IN_MEAS_NUM; i++)
	    in_meas.add(topics_meas[i].value);

	  // Estimate.
	  estimation::Output out = estimator->estimate(in_meas);

	  // Set output message(s) and publish.
	  PUBLISH(publishers, out, sampleFused);
	}
	catch (std::exception& e)
	{
	  ROS_ERROR_STREAM(e.what());
	}

	// Samples/messages are processed, so reset all receive flags.
	for (int i = 0; i < TOPICS_IN_MEAS_NUM; i++) 
	  topics_meas[i].received = false;
      }
      
      // Handle callbacks (check for next samples).
      ros::spinOnce();
    }

    // --------------------------------------------
    // Clean up.
    // --------------------------------------------
    delete estimator;

    // Close connections.
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
