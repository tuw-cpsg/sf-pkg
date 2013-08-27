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
#include <fstream>
#include <sstream>
#include <string>

#include "boost/program_options.hpp"
#include "boost/filesystem.hpp" 
namespace po = boost::program_options; 

// ROS includes
#include "ros/ros.h"
#include "std_msgs/Float64.h"

// estimation framework includes
#include "estimation/IEstimationMethod.h"
#include "estimation/Input.h"
#include "estimation/InputValue.h"
#include "estimation/OutputValue.h"
#include "estimation/methods.h"

#define MSG_BUFFER_SIZE			1000

// Forward declarations.
po::variables_map getArgumentMap(int argc, char** argv);
estimation::IEstimationMethod* getEstimator(boost::program_options::variables_map vm);

// Message objects.
std_msgs::Float64 sample;
std_msgs::Float64 sampleFused;

/** Sample publisher (global because publishing is done in callback
    function); */
ros::Publisher pub_signalFused;

/** Estimator, which does the filtering of a signal. */
estimation::IEstimationMethod* estimator;

/**
 * Called when a sample is received.
 */
void sampleReceived(const std_msgs::Float64::ConstPtr& msg)
{
  sample = *msg;
    
  // Estimation
  estimation::Input in(estimation::InputValue(sample.data));
  estimation::Output out = estimator->estimate(in);

  // Fill the message with data.
  sampleFused.data = out.getValue();

  // Send message.
  pub_signalFused.publish(sampleFused);
  ROS_DEBUG("sample | sample-fused: %.2f | %.2f", sample.data, sampleFused.data);
}

/**
 * This node implements a specified sensor fusion method. 
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

    // Check arguments and create an estimator according to it.
    try {
      // Get arguments/options in a map.
      po::variables_map vm = getArgumentMap(argc, argv);
      if (vm.empty())	// No arguments for estimator (e.g. help message printed).
	return 0;	// Exit application.

      // Get estimator according to the arguments.
      estimator = getEstimator(vm);
      ROS_DEBUG("Estimator initialized.");
    } catch(po::error& e) { 
      ROS_ERROR_STREAM(e.what() << std::endl << std::endl
		       << "Use --help or -h to print a help message."
		       << std::endl); 
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

    /*
    // for testing: create a "noisy signal", pass it to an estimator
    // and print the results ("x y \n" for gnuplot)
    std::string line;
    while (std::getline(std::cin, line))
    {
      double x;

      std::stringstream ss(line);
      ss >> x;
      
      estimation::InputValue in(x,0);
      estimation::OutputValue out = estimator->estimate(in);
      
      std::cout << out.getValue() << std::endl;
    }
    */

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

// ===================================================================
// models and validators for multitoken options
// ===================================================================

// needed because multiple tokens cannot be parsed like arg = 1 2 3 in
// the config file, with a custom validator arg = "1 2 3" and above
// example should be possible

/**
 * Structure for validation of double tokens of a multitoken option.
 *
 * Use as option type for boost::program_options when multiple tokens
 * should be allowed, e.g. coefficients. Needed because multitokens
 * don't work out of the box with a configuration file, although they
 * work when parsing from the console.
 */
struct multi_double {
  std::vector<double> vector;
};

/**
 * Validates double tokens of a multitoken option.
 *
 * Called by boost::program_options when converting the option
 * arguments from string to specified types.
 */
void validate(boost::any& v, const std::vector<std::string>& values,
	      multi_double* target_type, int) 
{
  multi_double md;

  /*
  std::cout << "----------" << std::endl
	    << "| values: " << std::endl;
  for (int i = 0; i < values.size(); i++)
    std::cout << values[i] << std::endl;
  std::cout << "----------" << std::endl;
  */
  if (values.size() == 0)
    throw po::validation_error(po::validation_error::invalid_option_value);
  else {
    // Extract tokens from values (string vector) and populate multi_double.
    for(std::vector<std::string>::const_iterator it = values.begin();
	it != values.end();
	it++) {
      std::stringstream ss(*it);

      copy(std::istream_iterator<double>(ss), std::istream_iterator<double>(),
	   back_inserter(md.vector));

      if(!ss.eof())
	throw po::validation_error(po::validation_error::invalid_option_value);
    }
  }

  v = md;
}
// ===================================================================

/**
 * Checks the arguments for validity, i.e. if the right parameters for
 * the specified methods are set an estimator is returned.
 *
 * @param vm Variable map which holds the arguments.
 * @return An instance of the specified estimation method.
 */
estimation::IEstimationMethod* getEstimator(po::variables_map vm) 
{
  std::string method = vm["method"].as<std::string>();

  // --------------------------------------------------------------------------------
  // Moving Median
  // --------------------------------------------------------------------------------
  if (method.compare("MovingMedian") == 0)
  {
    ROS_INFO("MovingMedian.");
    estimation::MovingMedian* mm = new estimation::MovingMedian();

    // required: none
    // optional: -w
    if (vm.count("window-size"))
    {
      unsigned int ws = vm["window-size"].as<unsigned int>();

      // debug
      std::stringstream ss;
      ss << ws;
      std::cout << "window-size = " << ss.str() << std::endl;
      
      mm->setWindowSize(ws);
    }

    return mm;
  }

  // --------------------------------------------------------------------------------
  // Moving Average
  // --------------------------------------------------------------------------------
  if (method.compare("MovingAverage") == 0)
  {
    ROS_INFO("MovingAverage.");
    estimation::MovingAverage* ma = new estimation::MovingAverage();

    // required: none
    // optional: -w -b
    unsigned int ws = ma->getWindowSize();	// default window size
    if (vm.count("window-size"))
    {
      ws = vm["window-size"].as<unsigned int>();

      // debug
      std::stringstream ss;
      ss << ws;
      std::cout << "window-size = " << ss.str() << std::endl;

      ma->setWindowSize(ws);
    }

    if (vm.count("weighting-coefficients"))
    {
      std::vector<double> wc = vm["weighting-coefficients"].as< multi_double >().vector;

      // debug
      std::stringstream ss;
      ss << "#weighting-coefficients: " << wc.size() << std::endl
	 << "weighting-coefficients = ";
      for (int i = 0; i < wc.size(); i++) 
	ss << wc[i] << " ";
      std::cout << ss.str() << std::endl;

      if (wc.size() == ws) {
	// only b_k coefficients are passed
	ma->setWeightingCoefficientsIn(&wc[0], ws);
      } else if (wc.size() == 2*ws-1) {
	// a_k coefficients also passed through arguments
	// first #window-size numbers
	ma->setWeightingCoefficientsOut(&wc[0], ws-1);
	ma->setWeightingCoefficientsIn(&wc[ws-1], ws);
      } else {
	throw po::error("Invalid number of "
			"weighting-coefficients "
			"(must equal "
			"1x or 2x-1 window size).");
      }
    }

    return ma;
  }

  throw po::error("Unknown method! "
		  "See help for available methods.");
}

/**
 * Collects the arguments into a map.
 *
 * @param argc The number of arguments given in the command line.
 * @param argv Pointer to the arguments.
 * @return The arguments in a map.
 */
po::variables_map getArgumentMap(int argc, char** argv) 
{
  std::string appName = boost::filesystem::basename(argv[0]);

  // Define general options.
  po::options_description desc_general("Basic Options");
  desc_general.add_options()
    ("configuration-file,c", po::value<std::string>(),
     "Use configuration file for arguments.")
    ("help,h", "Print help message.")    
    ;

  // Define sensor fusion specific options.
  po::options_description desc_sf("Single Sensor Fusion Options:"); 
  desc_sf.add_options() 
    // no short options allowed!
    //("help", "Print this help message.")
    ("method", po::value<std::string>()->required(), 
     "The estimation method.\n\nExample values are:\n"
     "MovingMedian, MovingAverage, KalmanFilter, etc.\n\n"
     "According to the method the parameters must be specified. "
     "In the following list the required and optional parameters "
     "for a particular method are listed:\n"
     "  MovingMedian: window-size\n"
     "  MovingAverage: window-size, weighting-coefficients\n"
     "\nParameters which are specified but not required for a"
     "method will be ignored.")
    ("window-size", po::value<unsigned int>(),
     "The number of data values to use for estimation. "
     "arg must be a positive integer. Default value is 3.") 
    ("weighting-coefficients", 
     po::value<multi_double>()->multitoken(), 
     "The weighting coefficients for a moving filter. "
     "The number of coefficients must equal the 1x window size or 2x "
     "window size - 1, e.g. window-size=3, weighting-coefficients=\"1 3 1\" "
     "corresponds to an FIR filter, i.e. the only the input is weighted. "
     "Default values will be set to apply an FIR filter with hamming "
     "window. 2x window-size - 1 causes the moving average "
     "filter to be in autoregressive mode, i.e. also (old) output "
     "values are weighted and added to a new one (e.g. window-size=3, "
     "weighting-coefficients=\"1 0 3 2 1\" where the first two "
     "coefficients are used for the old output values and the last "
     "three coefficients for the input values).")
    ;
 
  po::variables_map vm; 
  
  // Parse and store arguments into variable map.
  // Parse general options.
  po::store(po::command_line_parser(argc, argv)
	    .options(desc_general).run(), 
	    vm); // throws on error

  // Help option.
  if ( vm.count("help") ) { 
    std::cout << "This node implements a specified sensor fusion method." 
	      << std::endl << std::endl 
	      << "The method and its parameters can be passed as "
	      << "arguments or in a configuration file. According to "
	      << "these settings an estimation class is instantiated "
	      << "which fuses the subscribed input value. With every "
	      << "new input value the estimate is updated and published." 
	      << std::endl << std::endl 
	      << desc_general << std::endl
	      << desc_sf << std::endl; 
    vm.clear();
    return vm; 
  } 

  // A configuration file provides the arguments?
  if ( vm.count("configuration-file") ) {
    // Parse sensor fusion options from configuration file.
    std::string configFilepath = vm["configuration-file"].as< std::string >();
    std::ifstream configFile;
    configFile.open(configFilepath.c_str(), std::ios::in);
    if (!configFile.is_open()) {
      throw po::error("Cannot open configuration file.");
    }

    vm.clear();
    po::store(po::parse_config_file(configFile, desc_sf), 
	      vm);

    po::notify(vm);	// throws on error, so do after help in case there
			// are any problems
  } else {
    // Parse sensor fusion options from argument list.
    po::store(po::command_line_parser(argc, argv)
	      .options(desc_sf)
	      .style(po::command_line_style::allow_long | 
		     po::command_line_style::long_allow_adjacent | 
		     po::command_line_style::long_allow_next)
	      .run(), 
	      vm);

    po::notify(vm);	// throws on error, so do after help in case there
			// are any problems
  }

  return vm;
}
