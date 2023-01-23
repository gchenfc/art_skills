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
  * TODO: need to access ac(0), ctrl_point(n-1), and all delta_t and T
  * @param t The time to get the point at
  * @return The xy point of the sigma log normal curve at time t
  */
  gtsam::Vector2 querySigmaLogNormal(const SlnParameters &params, double t) {
    params = params(t) // TODO: is this indexing okay?
    params_secondary = [[ac(0), ctrl_point.xy(n-1), delta_t, T]]
    // TODO: move ac(0) into params passed into getweight
    weight = getweight(params)
    // TODO: create vector of [[ctrl_point(n-1)], [ctrl_point]]
    stroke_points = [[ctrl_point.xy(n-1), ctrl_point.xy(n)]]
    D, theta = getvector(stroke_points) 
    // find theta between ctrl points
    // find D between ctrl points
    // find displacement
    // get position
    point = pointposition(params, weight)
    // TODO(JD): implement this
    return gtsam::Vector2::Zero();
  }

 private:

  # include math.h

  using namespace std;

  /**
  * Find the weighted displacement after computing the parameters derived from
  * the input parameters: 
  * sigma: 
  * mu:
  * d:
  * theta:
  * t0:
  * @param params The sigma log normal curve parameters
  * @param t The time to get the point at
  * @return The xy point of the sigma log normal curve at time t
  */
  getweight(params, strokenumber, t){
    t0 = 0
    // TODO: find way to call back to first mu and sigma
    sigma_0 = sqrt(-log(1 - params.ac[0]));
    mu_0 = 3*sigma_0 - log((-1 + exp(6 * sigma_0))/params.period[0])
    if (strokenumber == 0) {
      sigma = sigma_0
      mu = mu_0
    }
    else{ // If strokenumber > 0
      sigma = sqrt(-log(1 - params.ac))
      mu = 3*sigma - log((-1 + exp(6 * sigma))/params.period)
      t0 = params.delta_t*params.period
    }
    t0 = t0 - exp(mu_0 - 3*sigma_0)
    t_stroke = std::max(t - t0, std::numeric_limits<double>::epsilon())
    weight = 0.5*(1 + std::erf(log(t_stroke) - mu) / (sigma*sqrt(2)))
    return weight
  }

  get

}




}  // namespace art_skills
