/**
 * @file 
 * @author Denise Ratasich
 * @date 09.09.2013
 *
 * @brief Prepares configuration for the ROS node.
 *
 * Abbreviations or macros respectively for the ROS node are generated
 * here with the Boost preprocessor library.
 */

#ifndef __CONFIGURATION_POSTCONFIG_H__
#define __CONFIGURATION_POSTCONFIG_H__

// boost preprocessor includes
#include <boost/preprocessor/tuple/elem.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/seq/elem.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/cat.hpp>

#include "estimation/InputValue.h"

// ----------------------------------------------------------------
// abbreviations for using configuration
// ----------------------------------------------------------------

/** @brief Expands to the number of topics defined in the
 * configuration header. */
#define TOPICS_NUM	BOOST_PP_SEQ_SIZE(TOPICS)

/** @brief Expands to the name of a topic T. */
#define TOPIC_NAME(T)		BOOST_PP_TUPLE_ELEM(3, 0, T)
/** @brief Expands to the name of a topic T as string. */
#define TOPIC_NAME_STR(T)	BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(3, 0, T))
/** @brief Expands to the field to estimate of a topic T. */
#define TOPIC_FIELD(T)		BOOST_PP_TUPLE_ELEM(3, 1, T)
/** @brief Expands to the type of a topic T. */
#define TOPIC_TYPE(T)		BOOST_PP_TUPLE_ELEM(3, 2, T)

/** @brief Initializes an element of the given array of subscribers
 * according to a topic. */
#define SUBSCRIBE_TO_TOPIC(r, data, i, elem)			\
  BOOST_PP_SEQ_ELEM(0, data)[i] =				\
    BOOST_PP_SEQ_ELEM(1, data).subscribe<TOPIC_TYPE(elem)>	\
    (								\
     TOPIC_NAME_STR(elem),					\
     20,							\
     BOOST_PP_CAT(received_,i)			\
    );								\
  ROS_INFO("Subscribing to '%s'.", TOPIC_NAME_STR(elem));	\
  /**/
/** 
 * @brief Initializes the subscribers.
 *
 * The topics are defined as a sequence of tuples. Each tuple is
 * processed to create a new subscriber which is stored in the given
 * array \c subscribers. */
#define SUBSCRIBE(subscribers, handle)					\
  BOOST_PP_SEQ_FOR_EACH_I(SUBSCRIBE_TO_TOPIC, (subscribers)(handle), TOPICS) \
  /**/

/** 
 * @brief Creates the callback function for a received message of a
 * specific topic.
 *
 * The received message is stored into data (must be of type struct
 * TopicState) for further processing.
 *
 * @param r Unnecessary, needed by Boost for repetitions.
 * @param data Must be an array of struct TopicState (defined in
 * sf_single.cpp) which saves the important parts of the message
 * needed for estimation.
 * @param i Number of the topic (position in the sequence TOPICS
 * defined by the user).
 * @param elem The topic tuple from TOPICS, i.e. name, type, field of
 * message to receive.
 */
#define RECEIVE_TOPIC(r, data, i, elem)					\
  void BOOST_PP_CAT(received_,i)(const TOPIC_TYPE(elem)::ConstPtr& msg) \
  {									\
    data[i].value.setValue(msg->TOPIC_FIELD(elem));			\
    data[i].received = true;						\
    ROS_DEBUG("RECV '%s': %.2f.",					\
	      TOPIC_NAME_STR(elem),					\
	      msg->TOPIC_FIELD(elem));					\
  }									\
  /**/
/** @brief Generates the methods for receiving the defined topics. */
#define RECEIVES(curTopicState)					\
  BOOST_PP_SEQ_FOR_EACH_I(RECEIVE_TOPIC, curTopicState, TOPICS)	\
  /**/

#endif
