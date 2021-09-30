/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SigmaLogNormal.h
 *  @author JD
 *  @author Gerry
 **/

#pragma once

#include <gtsam/base/Vector.h>

#include "SlnParameters.h"

namespace art_skills {

class WeightedSigmaLognormal {
 public:
  /**
  * Compute the location of the sigma log normal curve at time t for some sigma
  * log normal parameters.
  * @param params The sigma log normal curve parameters
  * @param t The time to get the point at
  * @return The xy point of the sigma log normal curve at time t
  */
  gtsam::Vector2 querySigmaLogNormal(const SlnParameters &params, double t) {
    // find sigma
    // find mu
    // find t0 (time of the impulse command to SL)
    // find time relative to start of stroke
    // find weights
    dparams = derivedparams(params)
    // find theta between ctrl points
    // find D between ctrl points
    // find displacements
    // get position
    // TODO(JD): implement this
    return gtsam::Vector2::Zero();
  }

 private:

  # include math.h

  using namespace std;

  /**
  * Compute the parameters derived from the input parameters:
  * sigma:
  * mu:
  * d:
  * theta:
  * t0:
  * @param params The sigma log normal curve parameters
  * @param t The time to get the point at
  * @return The xy point of the sigma log normal curve at time t
  */
  derivedparams(params, strokenumber, t){
    sigma = sqrt(-log(1 - params.ac))
    mu = 3*sigma - log((-1 + exp(6 * sigma))/params.period)
    // TODO: find a way to keep track of which t0 this is
    t0 = 0
    if (strokenumber > 0) {
      t0 = params.delta_t*params.period
    }
    t0 = t0 - exp(mu_0 - 3*sigma_0) // TODO: find way to call back to first mu and sigma
    t_stroke = std::max(t - t0, std::numeric_limits<double>::epsilon())
    weight = 0.5*(1 + std::erf(log(t_stroke) - mu) / (sigma*sqrt(2)))

  }

}




}  // namespace art_skills
