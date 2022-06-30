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
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include "expressions.h"

#include <vector>

namespace art_skills {

/**
 * SlnStrokeExpression defines a single stroke, the length of which is defined
 * by D along the trajectory. The D of each stroke cannot overlap that of
 * another Theta1 and 2 describe the angular evolution of the stroke Sigma and
 * mu describe the shape of the lognormal t0 defines the start time of the
 * stroke along the trajectory
 */
class SlnStrokeExpression {
  using Double_ = gtsam::Double_;
  using Vector2_ = gtsam::Vector2_;
  using Vector2 = gtsam::Vector2;
  using Vector6_ = gtsam::Vector6_;
  Double_ t0;      // start of impulse in global time
  Double_ D;       // amplitude of stroke (||P[i] - P[i-1]||)
  Double_ theta1;  // initial angular deviation
  Double_ theta2;  // initial angular deviation
  Double_ sigma;   // logresponse time
  Double_ mu;      // logtime delay

 public:
  /// Construct from individual parameters
  SlnStrokeExpression(const Double_& t0, const Double_& D,
                      const Double_& theta1, const Double_& theta2,
                      const Double_& sigma, const Double_& mu)
      : t0(t0), D(D), theta1(theta1), theta2(theta2), sigma(sigma), mu(mu) {}

  /// Construct from initial point and 6-vector of parameters
  SlnStrokeExpression(const Vector6_& p)
      : t0(indexVector<0>(p)),
        D(indexVector<1>(p)),
        theta1(indexVector<2>(p)),
        theta2(indexVector<3>(p)),
        sigma(indexVector<4>(p)),
        mu(indexVector<5>(p)) {}

  static SlnStrokeExpression CreateSlnStrokeExpressionReparameterized(
      const Double_& t0, const Double_& logD, const Double_& theta1,
      const Double_& theta2, const Double_& logSigma, const Double_& mu) {
    return SlnStrokeExpression(t0, exp(logD), theta1, theta2, exp(logSigma),
                               mu);
  }
  static SlnStrokeExpression CreateSlnStrokeExpressionReparameterized(
      const Vector6_& params) {
    return CreateSlnStrokeExpressionReparameterized(
        indexVector<0>(params), indexVector<1>(params), indexVector<2>(params),
        indexVector<3>(params), indexVector<4>(params), indexVector<5>(params));
  }

  /**
   * Compiutes lognormal curve of a stroke, i.e., the impulse.
   * @param t The time the trajectory is being evaluated at
   * @return The xy point in a stroke at trajectory time t
   */
  Double_ log_impulse(Double_ t) const {
    Double_ lambda =
        Double_(1.0) / (t * sigma * k_sqrt2pi) *
        power(Double_(k_e), Double_(-1.0) * ((log(t) - mu) * (log(t) - mu)) /
                                (Double_(2.0) * sigma * sigma));
    return lambda;
  }

  /**
   * Define the speed profile of a stroke.
   * @param lambda the location along the lognormal curve at time t
   * @return The speed at time t
   */
  Double_ speed(Double_ lambda) const {
    return D * lambda;
  }  // * defined for compose operation

  /**
   * Define the curvilinear evolution of a stroke, i.e., it's direction.
   * @param t The time the trajectory is being evaluated at
   * @return The direction/angle in a stroke at time t
   */
  Double_ direction(Double_ t) const {
    return theta1 + 0.5 * (theta2 - theta1) *
                        (Double_(1.0) +
                         erf((log(t) - mu) / (sigma * Double_(sqrt(2.0)))));
  }  // don't need expression of "t", since t is not a parameter (no 1/dt)

  /**
   * Compute displacment at time t along the stroke
   * @param inst_t The time the trajectory is being evaluated at
   * @param dt The timestep used for integration/sampling
   * @return The xy position in a stroke at time t
   */
  Vector2_ displacement(Double_ inst_t, double dt) const {
    const Double_ lambda = log_impulse(inst_t);
    const Double_ s = speed(lambda);
    const Double_ phi = direction(inst_t);
    return dt * (s * createVector(cos(phi), sin(phi)));
  }

  /**
   * Creates a factor that enforces the relationship between the "current" point
   * and the "next" point given the stroke parameters, by enforcing x2 = x1 +
   * v2*dt, where v2 is calculated using the stroke parameters.
   * @param timestep The time step index of the "current" point.
   * @param dt The timestep used for integration/sampling
   * @param noise The noise model to use (defaults to Isotropic sigma=0.001).
   * @return The factor enforcing the displacement relationship.
   */
  gtsam::NonlinearFactor::shared_ptr pos_integration_factor(
      size_t timestep, double dt,
      gtsam::noiseModel::Base::shared_ptr noise =
          gtsam::noiseModel::Isotropic::Sigma(2, 0.001)) const {
    auto dx = displacement(Double_((timestep + 1) * dt) - t0, dt);
    // creating vector 2 of symbol xcurrent at timestep (int)
    Vector2_ xcurrent(gtsam::symbol('x', timestep));
    Vector2_ xnext(gtsam::symbol('x', timestep + 1));
    Vector2_ error = xcurrent + dx - xnext;
    // measurement should be 0, which is ensuring error is 0
    // Create an expression factor and return in the form of a shared pointer.
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        boost::make_shared<gtsam::ExpressionFactor<Vector2>>(
            noise, Vector2::Zero(), error));
  }

  /** @defgroup query_functions Functions to call the SLN function with known
   * values
   *  @{
   */

  /** Converts absolute time to strokewise time. */
  Double_ time(double t, bool is_absolute_time) const {
    return is_absolute_time ? Double_(t) - t0 : Double_(t);
  }

  /** Returns the displacement at a given time. */
  gtsam::Vector2 displacement(
      double t, double dt, bool is_absolute_time = true,
      const gtsam::Values& values = gtsam::Values()) const {
    return displacement(time(t, is_absolute_time), dt).value(values);
  }

  /** Returns the speed at a given time. */
  double speed(double t, bool is_absolute_time = true,
               const gtsam::Values& values = gtsam::Values()) const {
    Double_ expr = speed(log_impulse(time(t, is_absolute_time)));
    return expr.value(values);
  }

  // TODO(gerry): add the other functions

  /** @} */  // end of query_functions
};

}  // namespace art_skills
