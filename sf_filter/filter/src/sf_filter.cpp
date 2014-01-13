/**
 * @file 
 * @author Denise Ratasich
 * @date 26.07.2013
 *
 * @brief ROS node for using a generic low-level sensor fusion method.
 */


// configuration
#include "configuration/configuration.h"


// cpp includes
#include <iostream>
#include <sstream>
#include <vector>

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
  /** @brief True when at least one message has been received. */
  bool received;
  /** @brief The timestamp of the last message received. **/
  ros::Time stamp;
  /** @brief The values of the messages received in a filter
   * period. **/
  std::vector<double> values;
  /** @brief The variance of the messages received in a filter
   * period. **/
  std::vector<double> variances;
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
      ROS_INFO_STREAM((*estimator) << " created.");
    } catch(std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      return 1;
    }

    // Get further parameters from the configuration header file.
    ros::Duration period;
    period.fromSec((double) getEstimatePeriod() / 1000);
    ROS_INFO("Estimate every %.0f ms.", period.toSec()*1000);
 
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
    // Estimate according the specified filter period (configuration).
    ros::Time lastEstimation = ros::Time::now();
    while (ros::ok()) 
    {
      try
      {
	// Control inputs are optional. 
#ifdef TOPICS_IN_CTRL
	// A received control value is immediately passed to the
	// estimator. So check if at least one control input has been
	// received.
	bool ctrl_recv = false;
	for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++)
	  ctrl_recv |= topics_ctrl[i].received;
      
	if (ctrl_recv)
	{
	  // Collect inputs together.
	  estimation::Input in_ctrl;
	  for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++)
	  {
	    estimation::InputValue in_ctrl_val;
	    if (topics_ctrl[i].received)
	    {
	      double value = topics_ctrl[i].values.back();				// take most actual value
	      double jitter = (ros::Time::now() - topics_ctrl[i].stamp).toSec()*1000;	// and its jitter
	      //ROS_DEBUG("ctrl-jitter[%d] (ms): %.2f", i, jitter);
	    
	      // initialize the InputValue
	      in_ctrl_val.setValue(value);
	      in_ctrl_val.setJitter(jitter);
	    }
	    in_ctrl.add(in_ctrl_val);
	  }

	  // Change control input of estimator.
	  estimator->setControlInput(in_ctrl);
	  ROS_DEBUG_STREAM("Passed control input " << in_ctrl << " to estimator.");

	  // Control messages are processed, so reset all receive flags
	  // and delete values.
	  for (int i = 0; i < TOPICS_IN_CTRL_NUM; i++) 
	  {
	    topics_ctrl[i].received = false;
	    topics_ctrl[i].values.clear();
	  }
	}
#endif

	// Check if its time to do the estimation.
	if ((ros::Time::now() - lastEstimation) >= period)
	{
	  // Collect inputs together.
	  estimation::Input in_meas;
	  for (int i = 0; i < TOPICS_IN_MEAS_NUM; i++)
	  {
	    estimation::InputValue in_meas_val;

	    if (topics_meas[i].received)
	    {
	      // At least one message of a topic has been received.

	      // Average the message values and variances (when more
	      // than one message is received from a topic in a filter
	      // period).
	      double value = 0;
	      for (int j = 0; j < topics_meas[i].values.size(); j++)
		value += topics_meas[i].values[j];
	      value = value / topics_meas[i].values.size();	// size always > 0 when received is true

	      double variance = 0;
	      if (topics_meas[i].variances.size() > 0)		// variance may be missing when no additional field is specified
	      {
		for (int j = 0; j < topics_meas[i].variances.size(); j++)
		  variance += topics_meas[i].variances[j];
		variance = variance / topics_meas[i].variances.size();
	      }

	      double jitter = (ros::Time::now() - topics_meas[i].stamp).toSec()*1000;

	      // Initialize the InputValue.
	      in_meas_val.setValue(value);
	      in_meas_val.setVariance(variance);
	      in_meas_val.setJitter(jitter);
	    }

	    in_meas.add(in_meas_val);
	  }

	  // Estimate.
	  ROS_DEBUG_STREAM("estimate | in: " << in_meas);
	  lastEstimation = ros::Time::now();
	  estimation::Output out = estimator->estimate(in_meas);
	  ROS_DEBUG("estimate | duration (ms): %.1f", (ros::Time::now() - lastEstimation).toSec()*1000);
	  ROS_DEBUG_STREAM("estimate | out: " << out);

	  // Set output message(s) and publish.
	  PUBLISH(publishers, out, sampleFused);

	  // Samples/messages are processed, so reset all receive flags
	  // and delete values.
	  for (int i = 0; i < TOPICS_IN_MEAS_NUM; i++) 
	  {
	    topics_meas[i].received = false;
	    topics_meas[i].values.clear();
	    topics_meas[i].variances.clear();
	  }
	}
      }
      catch (std::exception& e)
      {
	ROS_ERROR_STREAM(e.what());
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
