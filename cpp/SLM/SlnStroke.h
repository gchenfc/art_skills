/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SlnStroke.h
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/base/Vector.h>

#include <vector>

namespace art_skills {

/**
 * SlnStroke defines a single stroke, the length of which is defined by D along
 * the trajectory. The D of each stroke cannot overlap that of another
 * Theta1 and 2 describe the angular evolution of the stroke
 * Sigma and mu describe the shape of the lognormal
 * t0 defines the start time of the stroke along the trajectory
 */
class SlnStroke {
  gtsam::Vector2 xy;  // point coordinate
  double t0;          // start of impulse in global time
  double D;           // amplitude of stroke (||P[i] - P[i-1]||)
  double theta1;      // initial angular deviation
  double theta2;      // initial angular deviation
  double sigma;       // logresponse time
  double mu;          // logtime delay

 public:
  /// Construct from individual parameters
  SlnStroke(const gtsam::Vector2& xy, double t0, double D, double theta1,
            double theta2, double sigma, double mu)
      : xy(xy),
        t0(t0),
        D(D),
        theta1(theta1),
        theta2(theta2),
        sigma(sigma),
        mu(mu) {}

  /// Construct from initial point and 6-vector of parameters
  SlnStroke(const gtsam::Vector2& xy, const gtsam::Vector6& p)
      : xy(xy),
        t0(p(0)),
        D(p(1)),
        theta1(p(2)),
        theta2(p(3)),
        sigma(p(4)),
        mu(p(5)) {}

  /**
   * Compiutes lognormal curve of a stroke, i.e., the impulse.
   * @param t The time the trajectory is being evaluated at
   * @return The xy point in a stroke at trajectory time t
   */
  double log_impulse(double t) const {
    double lambda = 1 / (sigma * sqrt(2 * M_PI) * (t - t0)) *
                    exp(-(std::pow(log(t - t0) - mu, 2)) / (2 * sigma * sigma));
    return lambda;
    // return 1 / (sigma * sqrt(2 * M_PI) * (t - t0)) *
    //        exp(-(std::pow(log(t - t0) - mu, 2)) / (2 * sigma * sigma));
  }

  /**
   * Define the speed profile of a stroke.
   * @param delta the location along the lognormal curve at time t
   * @return The speed at time t
   */
  double speed(double lambda) const { return D * lambda; }

  /**
   * Define the curvilinear evolution of a stroke, i.e., it's direction.
   * @param t The time the trajectory is being evaluated at
   * @return The direction/angle in a stroke at time t
   */
  double direction(double t) const {
    return theta1 + (theta2 - theta1) / 2 *
                        (1 + erf((log(t - t0) - mu) / (sigma * sqrt(2))));
  }

  /**
   * Compute position at time t along the stroke
   * @param t The time the trajectory is being evaluated at
   * @param dt The timestep used for integration/sampling
   * @return The xy position in a stroke at time t
   */
  // TODO: Fix the way time is handled, t0 should be taken into account at the
  // end
  gtsam::Vector2 position(double t, double dt) const {
    gtsam::Vector2 xy = this->xy;  // initialize to starting point
    double inst_t = 0;
    // Integrate
    for (int i = 1; (dt * i) <= (t - 0); i++) {
      inst_t = 0 + i * dt;
      const double lambda = log_impulse(inst_t);
      const double s = speed(lambda);
      const double phi = direction(inst_t);
      std::cout << "\n t =  " << inst_t << " | lambda = " << lambda
                << " | v = " << s << " | phi = " << phi << "\n";
      xy = xy + dt * (s * gtsam::Vector2(cos(phi), sin(phi)));
      std::cout << "xy \n" << xy << "\n ...";
      // xy = xy + gtsam::Vector2(s, s);
      // xy = gtsam::Vector2(s, i);
    }

    return xy;
  }
};

}  // namespace art_skills
