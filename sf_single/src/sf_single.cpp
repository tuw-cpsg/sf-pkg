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
#include <sstream>
#include <string>

#include "boost/program_options.hpp"
#include "boost/filesystem.hpp" 

// ROS includes
#include "ros/ros.h"

// estimation framework includes
#include "estimation/IEstimationMethod.h"
#include "estimation/InputValue.h"
#include "estimation/OutputValue.h"
#include "estimation/methods.h"

#define MSG_BUFFER_SIZE			1000

// Forward declarations.
int createAndInitEstimator(int argc, char** argv, estimation::IEstimationMethod** estimator);

// error codes
const size_t SUCCESS = 0;  
const size_t ERROR_UNHANDLED_EXCEPTION = 1; 
const size_t ERROR_IN_COMMAND_LINE = 2;
const size_t PRINTED_HELP = 3;

/** Name of the estimation method. */
std::string method;

// Instantiate the message object.
//estimation_msgs::InputValue in;

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
    // Initialize for ROS.
    //ros::init(argc, argv, "sf_single");

    // ROS specific arguments are handled and removed by ros::init(..),
    // f.e. remapping arguments added by roslaunch.

    estimation::IEstimationMethod* estimator;

    // Check arguments and create an estimator according to it.
    int parseResult = createAndInitEstimator(argc, argv, &estimator);
    if (parseResult == PRINTED_HELP)	
      return SUCCESS;		// printed help is not really an error
    else if (parseResult != 0)	// something else wrong with arguments?
      return parseResult;	// return error code

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
    
/*
    // Create main access point to communicate with the ROS system.
    ros::NodeHandle n;
    
    // Tell ROS that we want to subscribe to the sensor fusion input
    // (the entity to estimate) and publish a fused version of it (the
    // estimate).
    ros::Subscriber entity_sub = n.subscribe<estimation_msgs::InputValue>("input", 10, dataReceived);
    ros::Publisher fusedEntity_pub = n.advertise<estimation_msgs::OutputValue>("estimate", MSG_BUFFER_SIZE);

    while (ros::ok())
    {
      // TODO

      // Handle callbacks if any.
      ros::spinOnce();
    }

    // Close connections.
    entity_sub.shutdown();
    fusedEntity_pub.shutdown();
    ROS_INFO("Closed connection.");*/
  }
  catch (std::exception& e)
  {
    std::cerr << "Unhandled Exception reached the top of main: " 
              << e.what() << ", application will now exit" << std::endl; 
    return ERROR_UNHANDLED_EXCEPTION; 
  }

  return SUCCESS;
}

/*
void dataReceived(const estimation_msgs::InputValue::ConstPtr& msg)
{
    in = *msg;
}
*/

/**
 * Checks the arguments for validity, i.e. if the right parameters for
 * the specified methods are set an estimator is returned.
 *
 * @param vm Variable map which holds the arguments.
 * @return An instance of the specified estimation method.
 */
estimation::IEstimationMethod* getEstimator(boost::program_options::variables_map vm) 
{
  // --------------------------------------------------------------------------------
  // Moving Median
  // --------------------------------------------------------------------------------
  if (method.compare("MovingMedian") == 0)
  {
    std::cout << "method = MovingMedian" << std::endl;
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
    std::cout << "method = MovingAverage" << std::endl;
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
      std::vector<double> wc = vm["weighting-coefficients"].as< std::vector<double> >();

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
	throw boost::program_options::error("Invalid number of "
					    "weighting-coefficients "
					    "(must equal "
					    "1x or 2x-1 window size).");
      }
    }

    return ma;
  }

  throw boost::program_options::error("Unknown method! "
				      "See help for available methods.");
}

/**
 * Parse the arguments passed to specify estimation method and its
 * needed parameters.
 *
 * @param argc The number of arguments given in the command line.
 * @param argv Pointer to the arguments.
 * @param config Parameters and estimation method will be stored there.
 * @return Error code.
 */
int createAndInitEstimator(int argc, char** argv, estimation::IEstimationMethod** estimator) 
{
  std::string appName = boost::filesystem::basename(argv[0]);
 
  // Define and parse the program options.
  namespace po = boost::program_options; 
  po::options_description desc("Options:"); 
  desc.add_options() 
    // no short options allowed!
    ("help", "Print this help message.")
    ("method", po::value<std::string>(&method)->required(), 
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
     po::value< std::vector<double> >()->multitoken(), 
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
     "three coefficients for the input values).");

    /*
      // allowing short options
    ("help,h", "Print this help message.")
    ("method,m", po::value<std::string>(&method)->required(), 
     "The estimation method.\n\nExample values are:\n"
     "MovingMedian, MovingAverage, KalmanFilter, etc.\n\n"
     "According to the method the parameters must be specified. "
     "In the following list the required and optional parameters "
     "for a particular method are listed:\n"
     "  MovingMedian: windowSize\n"
     "  MovingAverage: windowSize, weighting-coefficients\n"
     "\nParameters which are specified but not required for a"
     "method will be ignored.")
    ("windowSize,w", po::value<unsigned int>(),
     "The number of data values to use for estimation. "
     "arg must be a positive integer. Default value is 3.") 
    ("weighting-coefficients,b", 
     po::value< std::vector<float> >()->multitoken(), 
     "The weighting coefficients b_k for moving filters. "
     "The number of coefficients must equal the window size "
     "(b_k, where k=1..windowSize), e.g. windowSize=3, "
     "weighting-coefficients=\"1 3 1\". "
     "Default values will be set to apply a hamming window."); 
    */
 
  po::variables_map vm; 
 
  try 
  { 
    // Parse and store arguments into variable map.
/*
    po::store(po::command_line_parser(argc, argv).options(desc).run(), 
	      vm); // throws on error
*/
    po::store(po::command_line_parser(argc, argv)
	      .options(desc)
	      .style(po::command_line_style::allow_long | 
		     po::command_line_style::long_allow_adjacent | 
		     po::command_line_style::long_allow_next)
	      .run(), vm);

    // Help option.
    if ( vm.count("help")  ) 
    { 
      std::cout << "This node implements a specified sensor fusion method." 
		<< std::endl << std::endl 
		<< "The method and its parameters can be passed as "
		<< "arguments or in a configuration file. According to "
		<< "these settings an estimation class is instantiated "
		<< "which fuses the subscribed input value. With every "
		<< "new input value the estimate is updated and published." 
		<< std::endl << std::endl << desc << std::endl; 
      return PRINTED_HELP; 
    } 
 
    po::notify(vm); // throws on error, so do after help in case there
		    // are any problems

    *estimator = getEstimator(vm);	// throws on error
  }
  catch(po::error& e) 
  { 
    std::cerr << "ERROR: " << e.what()
	      << std::endl << std::endl << desc << std::endl; 
    return ERROR_IN_COMMAND_LINE; 
  } 

  return SUCCESS;
}
