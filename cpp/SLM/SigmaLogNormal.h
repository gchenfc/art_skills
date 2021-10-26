/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SigmaLogNormal.h
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 **/

#pragma once

#include <gtsam/base/Vector.h>

#include "SlnParameters.h"

namespace art_skills {

class SigmaLognormal {
 public:
  /**
  * Compute a point in a sigma lognormal curve at time t given SL parameters
  * @param params The sigma log normal curve parameters
  * @param t The time to get the point at
  * @return The xy point of the sigma log normal curve at time t
  */
  // TODO: how are the num_strokes being set/extracted?
  gtsam::Vector2 querySigmaLogNormal(const SlnParameters &params, double t) {
    point = position(params, t, dt, num_strokes)
    // TODO(JD): implement this
    return gtsam::Vector2::Zero();
  }

 private:

  #include math.h
  using namespace std;
  double pi = 3.14159265358979323846;

  /**
  * Define lognormal curve of a stroke
  * @param params The SLN curve parameters
  * @param t The time the trajectory is being evaluated at
  * @return The xy point in a stroke at trajectory time t
  */
  logimpulse(sigma, mu, t0, t){
    delta = -(1/(sigma*sqrt(2*pi)*(t - t0)))*exp(((log(t - t0) - mu)^2)/(2*sigma^2));
    return delta;
  }

  /**
  * Define the velocity profile of a stroke
  * @param D The stroke amplitude
  * @param delta the location along the lognormal curve at time t
  * @return The speed at time t
  */
  velocity(D, delta){
    vel = D*delta;
    return vel;
  }

  /**
  * Define the curvilinear evolution of a stroke
  * @param params The SLN curve parameters
  * @param t The time the trajectory is being evaluated at
  * @return The direction/angle in a stroke at time t
  */
  direction(theta1, theta2, mu, sigma, t0, t){
    phi = theta1 + (theta2 - theta1)/2 * (1 + erf((log(t - t0) - mu)/(sigma*sqrt(2))));
    return phi;
  }

  /**
  * Define position along the trajectory
  * @param params The SLN curve parameters
  * @param t The time the trajectory is being evaluated at
  * @param dt The timestep used for integration/sampling
  * @param numstrokes The number of strokes in the trajectory
  * @return The xy position in a stroke at time t
  */
  position(params, t, dt, num_strokes){
    gtsam::Vector2 xy = xy_0; // initialize to starting point
    inst_t = 0;
    for(i=1; i<=(t - 0); i++){
      inst_t = 0 + i*dt
      // TODO: how to sum from stroke 1 to stroke n? 
      //       (how to access specific part of vector? Maybe row?)
      vel = velocity(params.stroke.D, logimpulse(params.stroke.sigma, 
                                                 params.stroke.mu, 
                                                 params.stroke.t0, inst_t));
      phi = direction(params.stroke.theta1, params.stroke.theta2, 
                      params.stroke.mu, params.stroke.sigma, 
                      params.stroke.t0, inst_t);
      xy = xy + dt*(vel*(cos(phi), sin(phi)));
    }
    return xy;
  }
}

}  // namespace art_skills
