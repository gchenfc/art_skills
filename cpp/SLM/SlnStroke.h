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

#include <gtsam/geometry/Point2.h>

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
  gtsam::Point2 xy_;  // point coordinate
  gtsam::Vector6 p_;
  double t0_;         // start of impulse in global time
  double D_;          // amplitude of stroke (||P[i] - P[i-1]||)
  double theta1_;     // initial angular deviation
  double theta2_;     // initial angular deviation
  double sigma_;      // logresponse time
  double mu_;         // logtime delay

 public:
  // Construct from individual parameters
  SlnStroke(const gtsam::Point2& xy, double t0, double D, double theta1,
            double theta2, double sigma, double mu)
      : xy_(xy),
        t0_(t0),
        D_(D),
        theta1_(theta1),
        theta2_(theta2),
        sigma_(sigma),
        mu_(mu) {}

  // Construct from initial point and 6-vector of parameters
  SlnStroke(const gtsam::Point2& xy, const gtsam::Vector6& p)
      : xy_(xy),
        t0_(p(0)),
        D_(p(1)),
        theta1_(p(2)),
        theta2_(p(3)),
        sigma_(p(4)),
        mu_(p(5)) {}

  /**
   * Compiutes lognormal curve of a stroke, i.e., the impulse.
   * @param t The time the trajectory is being evaluated at
   * @return The xy point in a stroke at trajectory time t
   */
  double log_impulse(double t) const {
    double lambda = 1 / (sigma_ * sqrt(2 * M_PI) * (t)) *
                    exp(-(std::pow(log(t) - mu_, 2)) / (2 * sigma_ * sigma_));
    return lambda;
  }

  /**
   * Define the speed profile of a stroke.
   * @param lambda the location along the lognormal curve at time t
   * @return The speed at time t
   */
  double speed(double lambda) const { return D_ * lambda; }

  /**
   * Define the curvilinear evolution of a stroke, i.e., it's direction.
   * @param t The time the trajectory is being evaluated at
   * @return The direction/angle in a stroke at time t
   */
  double direction(double t) const {
    return theta1_ + (theta2_ - theta1_) / 2 *(1 + erf((log(t) - mu_) / (sigma_ * sqrt(2))));
  }

  /**
   * Compute position at time t along the stroke
   * @param t The time the trajectory is being evaluated at
   * @param dt The timestep used for integration/sampling
   * @return The xy position in a stroke at time t
   */
  gtsam::Point2 position(double t, double dt) const {
    gtsam::Point2 xy = this->xy_;  // initialize to starting point
    double inst_t = 0;
    // Integrate
    for (size_t k = 1; (dt * k) <= (t - t0_); k++) {
      inst_t = k * dt; // stroke-wise instantaneous time, no need to consider t0 here
      const double lambda = log_impulse(inst_t);
      const double s = speed(lambda);
      const double phi = direction(inst_t);
      xy = xy + dt * (s * gtsam::Point2(cos(phi), sin(phi)));
    }

    return xy;
  }
};

}  // namespace art_skills
