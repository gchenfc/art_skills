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
#include "math.h"

namespace art_skills {

class SigmaLogNormal {
 public:
  /**
   * Compute a point in a sigma lognormal curve at time t given SL parameters
   * @param params The sigma log normal curve parameters
   * @param t The time to get the point at
   * @return The xy point of the sigma log normal curve at time t
   */
  // TODO: how are the num_strokes being set/extracted?
  static gtsam::Vector2 queryposition(const SlnStroke &strokeparams, double t,
                               double dt) {
    return position(strokeparams, t, dt);
  }

  /**
   * Define lognormal curve of a stroke
   * @param params The SLN curve parameters
   * @param t The time the trajectory is being evaluated at
   * @return The xy point in a stroke at trajectory time t
   */
  static double logimpulse(double t0, double sigma, double mu, double t) {
    return -(1 / (sigma * sqrt(2 * M_PI) * (t - t0))) *
           exp((std::pow((log(t - t0) - mu), 2)) / (2 * sigma * sigma));
  }

  /**
   * Define the velocity profile of a stroke
   * @param D The stroke amplitude
   * @param delta the location along the lognormal curve at time t
   * @return The speed at time t
   */
  static double velocity(double D, double lambda) {
    return D * lambda;
    ;
  }

  /**
   * Define the curvilinear evolution of a stroke
   * @param params The SLN curve parameters
   * @param t The time the trajectory is being evaluated at
   * @return The direction/angle in a stroke at time t
   */
  static double direction(double theta1, double theta2, double mu, double sigma,
                          double t0, double t) {
    return theta1 + (theta2 - theta1) / 2 *
                        (1 + erf((log(t - t0) - mu) / (sigma * sqrt(2))));
  }

  /**
   * Define position along the trajectory
   * @param params The SLN curve parameters
   * @param t The time the trajectory is being evaluated at
   * @param dt The timestep used for integration/sampling
   * @return The xy position in a stroke at time t
   */
  static gtsam::Vector2 position(const SlnStroke &strokeparameters, double t,
                                 double dt) {
    gtsam::Vector2 xy = strokeparameters.xy;  // initialize to starting point
    double inst_t = 0;

    // Integrate
    for (int i = 1; i <= (t - 0); i++) {
      inst_t = 0 + i * dt;
      double vel =
          velocity(strokeparameters.D,
                   logimpulse(strokeparameters.sigma, strokeparameters.mu,
                              strokeparameters.t0, inst_t));
      double phi = direction(strokeparameters.theta1, strokeparameters.theta2,
                             strokeparameters.mu, strokeparameters.sigma,
                             strokeparameters.t0, inst_t);
      xy = xy + dt * (vel * gtsam::Vector2(cos(phi), sin(phi)));
    }
    return xy;
  }
};

}  // namespace art_skills
