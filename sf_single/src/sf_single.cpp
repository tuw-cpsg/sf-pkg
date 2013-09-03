/**
 * @file 
 * @author Denise Ratasich
 * @date 26.07.2013
 *
 * @brief ROS node for using a single sensor fusion method, in
 * particular estimation.
 */

// cpp includes
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// estimation framework includes
#include "estimation/IEstimationMethod.h"
#include "estimation/Input.h"
#include "estimation/OutputValue.h"
#include "estimation/methods.h"

// configuration
#include "Configurator.h"
#include "config.h"
void initConfigurator(Configurator& configurator);

#define MSG_BUFFER_SIZE			1000

/** @brief Sample publisher (global because publishing is done in
    callback function); */
ros::Publisher pub_signalFused;

/** @brief Estimator, which does the filtering of a signal. */
estimation::IEstimationMethod* estimator;

/**
 * @brief Called when a sample is received.
 */
void sampleReceived(const std_msgs::Float64::ConstPtr& msg)
{
  // Message objects.
  std_msgs::Float64 sample = *msg;
  std_msgs::Float64 sampleFused;
    
  try {
    // Estimation
    estimation::Input in(estimation::InputValue(sample.data));
    estimation::Output out = estimator->estimate(in);

    // Fill the message with data.
    sampleFused.data = out.getValue();
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(e.what());
  }

  // Send message.
  pub_signalFused.publish(sampleFused);
  ROS_DEBUG("sample | sample-fused: %.2f | %.2f", sample.data, sampleFused.data);
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
      Configurator configurator;
      initConfigurator(configurator);
      estimator = configurator.getInitializedEstimator();
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
    ros::Subscriber sub_signal = n.subscribe<std_msgs::Float64>("signal", 20, sampleReceived);
    //ros::Subscriber entity_sub = n.subscribe<estimation_msgs::InputValue>("input", 10, dataReceived);
    pub_signalFused = n.advertise<std_msgs::Float64>("signal_fused", MSG_BUFFER_SIZE);

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
    ROS_DEBUG("Closed connections.");
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Unhandled Exception reached the top of main: " 
		     << e.what() << ", application will now exit." << std::endl); 
    return 1; 
  }

  return 0;
}

// Configuration
void initConfigurator(Configurator& configurator)
{
#ifdef METHOD
  configurator.addParam("method", std::string(METHOD));
#endif
#ifdef WINDOW_SIZE
  configurator.addParam("window-size", WINDOW_SIZE);
#endif
#ifdef WEIGHTING_COEFFICIENTS
  Configurator::vector wc = { WEIGHTING_COEFFICIENTS };
  configurator.addParam("weighting-coefficients", wc);
#endif
#ifdef STATE_TRANSITION_MODEL
  Configurator::matrix stm = { STATE_TRANSITION_MODEL };
  configurator.addParam("state-transition-model", stm);
#endif
#ifdef PROCESS_NOISE_COVARIANCE
  Configurator::vector pnc = { PROCESS_NOISE_COVARIANCE };
  configurator.addParam("process-noise-covariance", pnc);
#endif
#ifdef OBSERVATION_MODEL
  Configurator::matrix om = { OBSERVATION_MODEL };
  configurator.addParam("observation-model", om);
#endif
#ifdef MEASUREMENT_NOISE_COVARIANCE
  Configurator::matrix mnc = { MEASUREMENT_NOISE_COVARIANCE };
  configurator.addParam("measurement-noise-covariance", mnc);
#endif
#ifdef CONTROL_INPUT_MODEL 
  Configurator::matrix cim = { CONTROL_INPUT_MODEL };
  configurator.addParam("control-input-model", cim);
#endif
#ifdef CONTROL_INPUT
  Configurator::vector ci = { CONTROL_INPUT };
  configurator.addParam("control-input", ci);
#endif
#ifdef INITIAL_STATE
  Configurator::vector is = { INITIAL_STATE };
  configurator.addParam("initial-state", is);
#endif
#ifdef INITIAL_ERROR_COVARIANCE
  Configurator::matrix iec = { INITIAL_ERROR_COVARIANCE };
  configurator.addParam("initial-error-covariance", iec);
#endif
}
