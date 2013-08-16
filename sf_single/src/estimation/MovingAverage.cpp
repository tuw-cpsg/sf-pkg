/**
 * @file 
 * @author Denise Ratasich
 * @date 16.08.2013
 *
 * @brief Implementation of a (weighted) moving average filter (FIR
 * and IIR).
 */

#include "estimation/MovingAverage.h"
#include <iostream>
using namespace std;

namespace estimation 
{

  MovingAverage::MovingAverage ()
  {
    // set parameters to default values
    windowSize = 3;

    b = new double[windowSize];
    b[0] = 1; b[1] = 3; b[2] = 1;

    isIIR = false;
    a = new double[windowSize-1];
    a[0] = a[1] = 0;
  }

  MovingAverage::~MovingAverage () 
  {
    delete[] a;
    delete[] b;
  }
	
  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  unsigned int MovingAverage::getWindowSize (void)
  {
    return windowSize;
  }

  void MovingAverage::setWindowSize (unsigned int windowSize)
  {
    if (windowSize > 0  &&  this->windowSize != windowSize)
      this->windowSize = windowSize;

    // free memory of arrays with old window size
    delete[] a;
    delete[] b;

    // create arrays for coefficients and initialize
    this->b = new double[windowSize];
    this->a = new double[windowSize-1];
    for (int i = 0; i < windowSize-1; i++) {
      b[i] = 1;		// TODO replace with hamming!
      a[i] = 0;
    }
    b[windowSize-1] = 1;
  }

  void MovingAverage::setWeightingCoefficientsIn (double *b, unsigned int size)
  {
    if (size != windowSize)
      return;		// TODO replace with throw exception
    
    // calculate normalization factor
    double norm = 0;
    for (int i = 0; i < windowSize; i++)
      norm += b[i];

    // save normalized coefficients
    for (int i = 0; i < windowSize; i++)
      this->b[i] = b[i] / norm;
  }

  void MovingAverage::setWeightingCoefficientsOut (double *a, unsigned int size)
  {
    if (size != windowSize)
      return;		// TODO replace with throw exception

    // calculate normalization factor
    double norm = 0;
    for (int i = 0; i < windowSize-1; i++)
      norm += a[i];

    if (norm > 0)	// there is at least one coefficient != 0
      isIIR = true;

    // save normalized coefficients
    for (int i = 0; i < windowSize-1; i++)
      this->a[i] = a[i] / norm;
  }

  // -----------------------------------------
  // IEstimationMethod implementation
  // -----------------------------------------
  OutputValue MovingAverage::estimate (InputValue next)
  {
    // push the data into the queue
    in.push_front(next);
    
    // check if queue has enough elements to estimate the average with
    // the specified windowSize
    if (in.size() >= windowSize) {
      // apply filter formula
      // y[n] = sum_k a_k y[n-k] + sum_k b_k x[n-k]
      double y = b[0] * in[0].getValue();
      for (int i = 1; i < windowSize; i++) {
	// in[0] ... front/new, in[windowSize-1] ... back/old
	// cout << i << ": " << y 
	//      << " + (a=" << a[i-1] << " * out=" << out[i-1].getValue() << ")"
	//      << " + (b=" << b[i] << " * in=" << in[i].getValue() << ")" 
	//      << endl;
	y += a[i-1] * out[i-1].getValue() + b[i] * in[i].getValue();
      }
      // normalization
      if (isIIR)
	y /= 2;

      // prepare output value (new estimate)
      OutputValue outY;
      outY.setValue(y);
      outY.setVariance(0);
      outY.setJitter(in.front().getJitter());
      out.push_front(outY);

      // delete oldest element for the next estimation
      in.pop_back();
      out.pop_back();
    } else {
      // set output = input for the first #window-size elements
      OutputValue nextOut;
      nextOut.setValue(next.getValue());
      nextOut.setVariance(0);
      nextOut.setJitter(next.getJitter());
      out.push_front(nextOut);
    }
    
    return out.front();
  }

  OutputValue MovingAverage::getLastEstimate(void) 
  {
    return out.front();
  }
}
