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
#include <boost/preprocessor/repetition/for.hpp>
#include <boost/preprocessor/arithmetic/inc.hpp>
#include <boost/preprocessor/arithmetic/dec.hpp>
#include <boost/preprocessor/comparison/not_equal.hpp>

#include "estimation/InputValue.h"


// ----------------------------------------------------------------
// abbreviations for checking SIZES
// ----------------------------------------------------------------

#if METHOD == EXTENDED_KALMAN_FILTER  ||  METHOD == UNSCENTED_KALMAN_FILTER
#define STATE_SIZE		BOOST_PP_SEQ_SIZE(STATE_TRANSITION_MODEL)
#define MEASUREMENT_SIZE	BOOST_PP_SEQ_SIZE(OBSERVATION_MODEL)
#endif

/** 
 * @brief Expands to the number of elements in a sequence.
 */
#define VECTOR_SIZE(vector)	\
  BOOST_PP_SEQ_SIZE(vector)	\
  /**/

/** 
 * @brief Expands to the number of elements in a sequence, i.e. for a
 * sequence-of-sequences its the number of rows.
 */
#define MATRIX_ROWS(matrix)	\
  BOOST_PP_SEQ_SIZE(matrix)	\
  /**/

/** 
 * @brief Expands to the number of elements of the first sequence of a
 * sequence, i.e. for a sequence-of-sequences its the number of
 * columns.
 */
#define MATRIX_COLS(matrix)				\
  BOOST_PP_SEQ_SIZE(BOOST_PP_SEQ_ELEM(0,matrix))	\
  /**/

// ----------------------------------------------------------------
// abbreviations for handling INPUTS/OUTPUTS
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
 * array \c subscribers. 
 */
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

/** 
 * @brief Generates the methods for receiving the defined topics.
 */
#define RECEIVES(curTopicState)					\
  BOOST_PP_SEQ_FOR_EACH_I(RECEIVE_TOPIC, curTopicState, TOPICS)	\
  /**/


// ----------------------------------------------------------------
// abbreviations for ASSIGNING a SEQUENCE to a VECTOR
// ----------------------------------------------------------------
/** 
 * @brief Expands to an assignment (e.g. x[0] = 10). 
 *
 * @param r Unnecessary, needed by Boost for repetitions.
 * @param data The name of the vector which should be assigned to.
 * @param i The counter, the position of the element in the vector
 * \c data which should be assigned to.
 * @param elem The value to assign, i.e. the i'th element of a
 * sequence.
 */
#define CODE_LINE_ASSIGN_ELEMENT_TO_VECTOR(r, data, i, elem)	\
  data[i] = elem;						\
  /**/

/**
 * @brief Expands to assignments of a defined vector (sequence of
 * double values) to an Eigen vector.
 *
 * @param vector The vector to be assigned to, i.e. the array which
 * should be filled with the values.
 * @param values_seq The sequence of values to assign to the vector.
 */
#define CODE_ASSIGN_VALUES_TO_VECTOR(vector, values_seq)		\
  BOOST_PP_SEQ_FOR_EACH_I(CODE_LINE_ASSIGN_ELEMENT_TO_VECTOR, vector, values_seq) 

/** 
 * @brief Creates assignments of formulas given in a sequence to a
 * vector.
 *
 * @param vector The vector to be assigned to, i.e. the array which
 * should be filled with the values.
 * @param formulas_seq The sequence of formulas to assign to the
 * vector.
 */
#define CODE_ASSIGN_FORMULAS_TO_VECTOR(vector, formulas_seq)		\
  BOOST_PP_SEQ_FOR_EACH_I(CODE_LINE_ASSIGN_ELEMENT_TO_VECTOR, vector, formulas_seq) 


// ----------------------------------------------------------------
// abbreviations for ASSIGNING a SEQUENCE-of-SEQUENCES to a MATRIX
// ----------------------------------------------------------------

/** @brief Expands to the element in a sequence-of-sequences (a
 * matrix), specified by row and col. */
#define MATRIX_ELEM(formulas, row, col) \
  BOOST_PP_SEQ_ELEM(col, BOOST_PP_SEQ_ELEM(row, formulas))

// macros for second (inner) loop ---------------------------------
#define PRED_2(r, state)				\
  BOOST_PP_NOT_EQUAL(BOOST_PP_TUPLE_ELEM(6, 2, state),	\
		     BOOST_PP_TUPLE_ELEM(6, 3, state))	\
  /**/

#define OP_2(r, state)						\
  (								\
   BOOST_PP_TUPLE_ELEM(6, 0, state),				\
   BOOST_PP_TUPLE_ELEM(6, 1, state),				\
   BOOST_PP_INC( BOOST_PP_TUPLE_ELEM(6, 2, state) ),		\
   BOOST_PP_TUPLE_ELEM(6, 3, state),				\
   BOOST_PP_TUPLE_ELEM(6, 4, state),				\
   BOOST_PP_TUPLE_ELEM(6, 5, state)				\
  )								\
  /**/

#define MACRO_2(r, state)						\
  BOOST_PP_TUPLE_ELEM(6, 4, state)(BOOST_PP_TUPLE_ELEM(6, 0, state),BOOST_PP_TUPLE_ELEM(6, 2, state)) \
  = MATRIX_ELEM( BOOST_PP_TUPLE_ELEM(6, 5, state),			\
		 BOOST_PP_TUPLE_ELEM(6, 0, state),			\
		 BOOST_PP_TUPLE_ELEM(6, 2, state));			\
  /**/

// macros for first (outer) loop ----------------------------------
#define PRED(r, state)						\
  BOOST_PP_NOT_EQUAL( BOOST_PP_TUPLE_ELEM(6, 0, state),		\
		      BOOST_PP_TUPLE_ELEM(6, 1, state) )	\
  /**/

#define OP(r, state)						\
  (								\
   BOOST_PP_INC( BOOST_PP_TUPLE_ELEM(6, 0, state) ),		\
   BOOST_PP_TUPLE_ELEM(6, 1, state),				\
   BOOST_PP_TUPLE_ELEM(6, 2, state),				\
   BOOST_PP_TUPLE_ELEM(6, 3, state),				\
   BOOST_PP_TUPLE_ELEM(6, 4, state),				\
   BOOST_PP_TUPLE_ELEM(6, 5, state)				\
  )								\
  /**/

#define MACRO(r, state)							\
  BOOST_PP_FOR_ ## r( (BOOST_PP_TUPLE_ELEM(6, 0, state),		\
		       BOOST_PP_TUPLE_ELEM(6, 1, state),		\
		       0,						\
		       BOOST_PP_TUPLE_ELEM(6, 3, state),		\
		       BOOST_PP_TUPLE_ELEM(6, 4, state),		\
		       BOOST_PP_TUPLE_ELEM(6, 5, state)),		\
		      PRED_2, OP_2, MACRO_2 )				\
   /**/

/**
 * @brief Expands to assignments of a defined matrix
 * (sequence-of-sequences) to an Eigen matrix.
 * 
 * Initializes an Eigen matrix with values of a macro (e.g. from the
 * configuration header).
 */
#define CODE_ASSIGN_FORMULAS_TO_MATRIX(matrix, formulas)	\
  BOOST_PP_FOR( (0,						\
		 MATRIX_ROWS(formulas),				\
		 0,						\
		 MATRIX_COLS(formulas),				\
		 matrix,					\
		 formulas),					\
		PRED, OP, MACRO )				\
  /**/

#define CODE_ASSIGN_VALUES_TO_MATRIX(matrix, values)	\
  CODE_ASSIGN_FORMULAS_TO_MATRIX(matrix, values)	\
  /**/

#endif // __CONFIGURATION_POSTCONFIG_H__
