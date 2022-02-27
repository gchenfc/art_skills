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

#include <vector>

namespace art_skills {

// Function to index into a vector for expression variables
// Getting jacobian of indexing into the vector, across all vars should be an
// identity matrix (stacked)
template <int I>
inline double indexVector(const gtsam::Vector6& s,
                          gtsam::OptionalJacobian<1, 6> Hs) {
  if (Hs) {
    gtsam::Vector6 mat = gtsam::Vector6::Zero();
    mat[I] = 1;
    *Hs = mat;
  }
  return s[I];
}
// Expression version of indexing into vect, using above function.
template <int I>
inline gtsam::Double_ indexVectorExpression(const gtsam::Vector6_& v) {
  return gtsam::Double_(&indexVector<I>, v);
}

// Function to populate a vector for expression variables
inline gtsam::Vector2 createVector(const double& a, const double& b,
                                   gtsam::OptionalJacobian<2, 1> Ha,
                                   gtsam::OptionalJacobian<2, 1> Hb) {
  if (Ha) {
    *Ha = gtsam::Matrix21::Zero();
    *Ha << 1, 0;
  }
  if (Hb) {
    *Hb = gtsam::Matrix21::Zero();
    *Hb << 0, 1;
  }
  return gtsam::Vector2(a, b);
}
// Expression version of scalar product, using above function.
inline gtsam::Vector2_ createVectorExpression(const gtsam::Double_ a,
                                              const gtsam::Double_ b) {
  return gtsam::Vector2_(&createVector, a, b);
}

// Function to "divide" expressions
inline double Divide(const double& n, const double& d,
                     gtsam::OptionalJacobian<1, 1> Hn,
                     gtsam::OptionalJacobian<1, 1> Hd) {
  if (Hn) {
    *Hn = gtsam::Vector1(1 / d);
  }
  if (Hd) {
    *Hd = gtsam::Vector1(-n / d / d);
  }
  return n / d;
}
// Expression version of scalar product, using above function.
inline gtsam::Double_ operator/(const gtsam::Double_ n,
                                const gtsam::Double_ d) {
  return gtsam::Double_(&Divide, n, d);
}

// Function to "product" 2 expressions
inline gtsam::Vector2 prodE(const double& n, const gtsam::Vector2& d,
                            gtsam::OptionalJacobian<2, 1> Hn,
                            gtsam::OptionalJacobian<2, 2> Hd) {
  if (Hn) {
    *Hn = d;
  }
  if (Hd) {
    *Hd = gtsam::I_2x2 * n;
  }
  return n * d;
}
// Expression version of scalar product, using above function.
inline gtsam::Vector2_ operator*(const gtsam::Double_ n,
                                 const gtsam::Vector2_ d) {
  return gtsam::Vector2_(&prodE, n, d);
}

// Function to use exponents in expressions
inline double power(const double& x, const double& p,
                    gtsam::OptionalJacobian<1, 1> Hx,
                    gtsam::OptionalJacobian<1, 1> Hp) {
  if (Hx) {
    *Hx = gtsam::Vector1(p * std::pow(x, p - 1));
  }
  if (Hp) {
    *Hp = gtsam::Vector1(std::pow(x, p) * log(x));
  }
  return std::pow(x, p);
}
// Expression version of scalar product, using above function.
inline gtsam::Double_ powerExpression(const gtsam::Double_ x,
                                      const gtsam::Double_ p) {
  return gtsam::Double_(&power, x, p);
}

// Function to use log in expressions
inline double logE(const double& x, gtsam::OptionalJacobian<1, 1> Hx) {
  if (Hx) {
    *Hx = gtsam::Vector1(1 / x);
  }
  return std::log(x);
}
// Expression version of scalar product, using above function.
inline gtsam::Double_ logExpression(const gtsam::Double_ x) {
  return gtsam::Double_(&logE, x);
}

// Function to use cos in expressions
inline double cosE(const double& x, gtsam::OptionalJacobian<1, 1> Hx) {
  if (Hx) {
    *Hx = gtsam::Vector1(-std::sin(x));
  }
  return std::cos(x);
}
// Expression version of cos(x), using above function.
inline gtsam::Double_ cosExpression(const gtsam::Double_ x) {
  return gtsam::Double_(&cosE, x);
}

// Function to use sin in expressions
inline double sinE(const double& x, gtsam::OptionalJacobian<1, 1> Hx) {
  if (Hx) {
    *Hx = gtsam::Vector1(std::cos(x));
  }
  return std::sin(x);
}
// Expression version of sin(x), using above function.
inline gtsam::Double_ sinExpression(const gtsam::Double_ x) {
  return gtsam::Double_(&sinE, x);
}

// Function to use exponents in expressions
inline double erfn(const double& z, gtsam::OptionalJacobian<1, 1> Hz) {
  if (Hz) {
    *Hz = gtsam::Vector1(2 / sqrt(M_PI) * exp(-(z * z)));
  }
  return erf(z);
}
// Expression version of scalar product, using above function.
inline gtsam::Double_ erfExpression(const gtsam::Double_ z) {
  return gtsam::Double_(&erfn, z);
}

static const double k_e = std::exp(1.0);
static const gtsam::Double_(k_sqrt2pi) =
    powerExpression(gtsam::Double_(2.0) * gtsam::Double_(M_PI), gtsam::Double_(0.5));

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
  Vector2_ xy;     // point coordinate
  Double_ t0;      // start of impulse in global time
  Double_ D;       // amplitude of stroke (||P[i] - P[i-1]||)
  Double_ theta1;  // initial angular deviation
  Double_ theta2;  // initial angular deviation
  Double_ sigma;   // logresponse time
  Double_ mu;      // logtime delay

 public:
  /// Construct from individual parameters
  SlnStrokeExpression(const gtsam::Vector2_& xy, Double_ t0, Double_ D,
                      Double_ theta1, Double_ theta2, Double_ sigma, Double_ mu)
      : xy(xy),
        t0(t0),
        D(D),
        theta1(theta1),
        theta2(theta2),
        sigma(sigma),
        mu(mu) {}

  /// Construct from initial point and 6-vector of parameters
  SlnStrokeExpression(const Vector2_& xy, const Vector6_& p)
      : xy(xy),
        t0(indexVectorExpression<0>(p)),
        D(indexVectorExpression<1>(p)),
        theta1(indexVectorExpression<2>(p)),
        theta2(indexVectorExpression<3>(p)),
        sigma(indexVectorExpression<4>(p)),
        mu(indexVectorExpression<5>(p)) {}

  /**
   * Compiutes lognormal curve of a stroke, i.e., the impulse.
   * @param t The time the trajectory is being evaluated at
   * @return The xy point in a stroke at trajectory time t
   */
  Double_ log_impulse(Double_ t) const {
    Double_ lambda =
        Double_(1.0) / (t * sigma * k_sqrt2pi) *
        powerExpression(Double_(k_e), Double_(-1.0) *
                                          ((logExpression(t) - mu) *
                                           (logExpression(t) - mu)) /
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
    return theta1 +
           0.5 * (theta2 - theta1) *
               (Double_(1.0) + erfExpression((logExpression(t) - mu) /
                                             (sigma * Double_(sqrt(2.0)))));
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
    return dt *
           (s * createVectorExpression(cosExpression(phi), sinExpression(phi)));
  }

  /**
   * create factor for each displacement point
   * @param inst_t time in the stroke
   * @param t0 start time of the stroke (t is trajectory time)
   * @param dt The timestep used for integration/sampling
   * @return The xy position in a stroke at time t
   */
  gtsam::ExpressionFactor<Vector2> pos_integration_factor(size_t timestep,
                                                          double dt) const {
    auto dx = displacement(Double_((timestep + 1) * dt) - t0, dt);
    // creating vector 2 of symbol xcurrent at timestep (int)
    Vector2_ xcurrent(gtsam::symbol('x', timestep));
    Vector2_ xnext(gtsam::symbol('x', timestep + 1));
    Vector2_ error = xcurrent + dx - xnext;
    // measurement should be 0, which is ensuring error is 0
    // return
    // gtsam::ExpressionFactor<Vector2>(gtsam::noiseModel::Constrained::All(2),
    // Vector2::Zero(), error);
    return gtsam::ExpressionFactor<Vector2>(
        gtsam::noiseModel::Isotropic::Sigma(2, 0.001*sqrt(dt)), Vector2::Zero(), error);
  }
};

}  // namespace art_skills
