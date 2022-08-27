/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SlnStrokeExpressions2.h
 *  @author Gerry Chen
 *  @author Juan-Diego Florez
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/base/utilities.h>
#include "expressions.h"

#include <vector>

namespace art_skills {

/**
 * SlnStrokeExpression2 defines a single stroke, the length of which is defined
 * by D along the trajectory. The D of each stroke cannot overlap that of
 * another. Theta1 and 2 describe the angular evolution of the stroke. Sigma and
 * mu describe the shape of the lognormal. t0 defines the start time of the
 * stroke along the trajectory
 */
class SlnStrokeExpression2 {
 protected:
  using Double_ = gtsam::Double_;
  using Vector2_ = gtsam::Vector2_;
  using Vector2 = gtsam::Vector2;
  using Vector6_ = gtsam::Vector6_;
  Double_ t0;      // start of impulse in global time
  Double_ D;       // total (arc) length of stroke
  Double_ theta1;  // initial angular deviation
  Double_ theta2;  // initial angular deviation
  Double_ sigma;   // logresponse time
  Double_ mu;      // logtime delay

 public:
  /// Construct from individual parameters
  SlnStrokeExpression2(const Double_& t0, const Double_& D,
                       const Double_& theta1, const Double_& theta2,
                       const Double_& sigma, const Double_& mu)
      : t0(t0), D(D), theta1(theta1), theta2(theta2), sigma(sigma), mu(mu) {}

  /// Construct from initial point and 6-vector of parameters
  SlnStrokeExpression2(const Vector6_& p)
      : t0(indexVector<0>(p)),
        D(indexVector<1>(p)),
        theta1(indexVector<2>(p)),
        theta2(indexVector<3>(p)),
        sigma(indexVector<4>(p)),
        mu(indexVector<5>(p)) {}

  static SlnStrokeExpression2 CreateSlnStrokeExpressionReparameterized(
      const Double_& t0, const Double_& logD, const Double_& theta1,
      const Double_& theta2, const Double_& logSigma, const Double_& mu) {
    return SlnStrokeExpression2(t0, exp(logD), theta1, theta2, exp(logSigma),
                                mu);
  }
  static SlnStrokeExpression2 CreateSlnStrokeExpressionReparameterized(
      const Vector6_& params) {
    return CreateSlnStrokeExpressionReparameterized(
        indexVector<0>(params), indexVector<1>(params), indexVector<2>(params),
        indexVector<3>(params), indexVector<4>(params), indexVector<5>(params));
  }

  Double_ s(const Double_& t) const {
    return 0.5 * (Double_(1.) + erf((log(t - t0) - mu) / (sqrt(2) * sigma)));
  }

  Double_ theta(const Double_& s) const {
    return theta1 + (theta2 - theta1) * s;
  }

  Vector2_ pos(const Double_& t, const Vector2_& x0 = Vector2_(Vector2::Zero()),
               double eps = 1e-8) const {
    Double_ s_ = s(t);
    Double_ s_dth = s_ * (theta2 - theta1);  // = theta(s) - theta1
    Double_ cth = cos(theta1), sth = sin(theta1);
    Double_ x_unrot = s_ * sinc(s_dth, eps), y_unrot = s_ * cosc(s_dth, eps);
    return D * createVector(cth * x_unrot - sth * y_unrot,  //
                            sth * x_unrot + cth * y_unrot) +
           x0;
  }

  Double_ speed(const Double_& t) const {
    Double_ normalization_constant = 1. / (sqrt(2 * M_PI) * sigma * (t - t0));
    Double_ z = (log(t - t0) - mu) / sigma;
    return D * normalization_constant * exp(-0.5 * z * z);
  }

  /** @defgroup query_functions Functions to call the SLN function with known
   * values
   *  @{
   */

  /** Converts absolute time to strokewise time. */
  Double_ time(double t, bool is_absolute_time = true) const {
    return is_absolute_time ? Double_(t) : Double_(t) + t0;
  }

  /** Returns the displacement at a given time. */
  gtsam::Vector2 pos(double t, bool is_absolute_time = true,
                     const gtsam::Vector2& x0 = gtsam::Vector2::Zero(),
                     const gtsam::Values& values = gtsam::Values()) const {
    return pos(time(t, is_absolute_time), x0).value(values);
  }

  /** Returns the speed at a given time. */
  double speed(double t, bool is_absolute_time = true,
               const gtsam::Values& values = gtsam::Values()) const {
    if ((time(t, is_absolute_time) - t0).value(values) < 1e-8) {
      return 0;
    }
    return speed(time(t, is_absolute_time)).value(values);
  }

  // TODO(gerry): add the other functions

  /** @} */  // end of query_functions

  /** @defgroup Functions needed for Testable traits
   *  @{
   */

  void print(const std::string& name, const gtsam::KeyFormatter& keyFormatter =
                                          gtsam::DefaultKeyFormatter) const {
    std::cout << name << ": SlnStroke with parameters:\n";
    // TODO(gerry): does Expression print have keyFormatter param?
    t0.print("\tt0: ");
    D.print("\tD: ");
    theta1.print("\ttheta1: ");
    theta2.print("\ttheta2: ");
    sigma.print("\tsigma: ");
    mu.print("\tmu: ");
  }

  bool equals(const SlnStrokeExpression2& expected, double tol) const {
    // TODO(gerry): figure out how to check Expression equality properly
    auto expr_print = [](const Double_& expr) -> std::string {
      gtsam::RedirectCout tmp;
      expr.print("");
      return tmp.str();
    };
    auto expr_equal = [&expr_print](const Double_& expr1, const Double_& expr2,
                                    double tol) {
      std::cout << "expr1: " << expr_print(expr1) << '\n'
                << "expr2: " << expr_print(expr2) << std::endl;
      return expr_print(expr1) == expr_print(expr2);
    };
    return expr_equal(t0, expected.t0, tol) && expr_equal(D, expected.D, tol) &&
           expr_equal(theta1, expected.theta1, tol) &&
           expr_equal(theta2, expected.theta2, tol) &&
           expr_equal(sigma, expected.sigma, tol) &&
           expr_equal(mu, expected.mu, tol);
    throw std::runtime_error("Not implemented");
  }

  /** @} */
};

}  // namespace art_skills

/// traits
template <>
struct gtsam::traits<art_skills::SlnStrokeExpression2>
    : public Testable<art_skills::SlnStrokeExpression2> {};
