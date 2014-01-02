/**
 * @file 
 * @author Denise Ratasich
 * @date 26.12.2013
 *
 * @brief Implementation of confidence-weighted averaging.
 */

#include <stdexcept>
#include "estimation/ConfidenceWeightedAveraging.h"

namespace estimation 
{
  ConfidenceWeightedAveraging::ConfidenceWeightedAveraging ()
  {
    // initialize and set output, size won't be changed!
    OutputValue def;
    out.add(def);

    ignoreZeroVarianceValues = false;
  }

  ConfidenceWeightedAveraging::~ConfidenceWeightedAveraging () 
  {
    // nothing to do
  }

  // -----------------------------------------
  // getters and setters
  // -----------------------------------------
  void ConfidenceWeightedAveraging::setIgnoreZeroVarianceValues ()
  {
    ignoreZeroVarianceValues = true;
  }

  void ConfidenceWeightedAveraging::clearIgnoreZeroVarianceValues ()
  {
    ignoreZeroVarianceValues = false;
  }

  // -----------------------------------------
  // IEstimator implementation
  // -----------------------------------------
  Output ConfidenceWeightedAveraging::estimate (Input next)
  {
    int numZeroVariance = 0, numMissing = 0;

    try 
    {
      // calculate the number of input values with zero variance
      for (int i = 0; i < next.size(); i++)
      {
	if (next[i].getVariance() == 0)
	  numZeroVariance++;

	if (next[i].isMissing())
	  numMissing++;
      }

      if (!ignoreZeroVarianceValues  &&  numZeroVariance > 0)
	throw std::runtime_error("At least one input with zero variance.");

      if (numMissing == next.size())
	throw std::runtime_error("All inputs are missing values.");

      // Averaging -------------------------------------------------
      double value = 0;
      double variance = 0;

      // choose averaging method
      if (ignoreZeroVarianceValues  &&  numZeroVariance == next.size())
      {
	// Simple Averaging
	for (int i = 0; i < next.size(); i++)
	{
	  if (!next[i].isMissing())
	    value += next[i].getValue();
	}
	value /= next.size() - numMissing;

	// variance = E[(x - mu)^2] = 1/n sum( (x-mu)^2 )
	for (int i = 0; i < next.size(); i++)
	{
	  if (!next[i].isMissing())
	    variance += (next[i].getValue() - value)*(next[i].getValue() - value);
	}
	variance /= next.size() - numMissing;
      }
      else
      {
	// Confidence-Weighted Averaging (with values where variance
	// is unequal to zero)

	// calc sum of reciprocal variances (needed for fused variance
	// and weight)
	double sumRecVar = 0;
	for (int i = 0; i < next.size(); i++)
	{
	  double var = next[i].getVariance();
	  if (var != 0  &&  !next[i].isMissing())
	    sumRecVar += 1/var;
	}

	// average the values
	for (int i = 0; i < next.size(); i++)
	{
	  double var = next[i].getVariance();
	  if (var != 0  &&  !next[i].isMissing())
	    value += next[i].getValue() * 1/var;
	}   
	value = value / sumRecVar;
	variance = 1 / sumRecVar;
      }
      // -----------------------------------------------------------

      // convert estimate into the Output object
      out[0].setValue(value);			// setValue first!
      out[0].setVariance(variance);
      return out;
    } 
    catch(std::exception& e) 
    {
      std::string additionalInfo = "ConfidenceWeightedAveraging: Estimation failed. ";
      throw estimator_error(additionalInfo + e.what());
    }
  }

  Output ConfidenceWeightedAveraging::getLastEstimate (void) const
  {
    return out;
  }

  void ConfidenceWeightedAveraging::serialize(std::ostream& os) const
  {
    os << "ConfidenceWeightedAveraging" << std::endl;
  }
}
