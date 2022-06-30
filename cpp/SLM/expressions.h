/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  expressions.h
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/expressions.h>

namespace art_skills {

namespace internal {

// Function to index into a vector for expression variables
// Getting jacobian of indexing into the vector, across all vars should be an
// identity matrix (stacked)
template <int I, int N>
inline double indexVector(const Eigen::Matrix<double, N, 1>& s,
                          gtsam::OptionalJacobian<1, N> Hs) {
  if (Hs) {
    Eigen::Matrix<double, N, 1> mat = Eigen::Matrix<double, N, 1>::Zero();
    mat[I] = 1;
    *Hs = mat;
  }
  return s[I];
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
// Function to use log in expressions
inline double logE(const double& x, gtsam::OptionalJacobian<1, 1> Hx) {
  static constexpr double kLogThreshold = 1e-8;
  if (x < kLogThreshold) {  // approximate as a very steep line when x <= 0
    if (Hx) *Hx = gtsam::Vector1(1 / kLogThreshold);
    return std::log(kLogThreshold) + (1 / kLogThreshold) * (x - kLogThreshold);
  }

  if (Hx) *Hx = gtsam::Vector1(1 / x);
  return std::log(x);
}
// Function to use sin in expressions
inline double sinE(const double& x, gtsam::OptionalJacobian<1, 1> Hx) {
  if (Hx) {
    *Hx = gtsam::Vector1(std::cos(x));
  }
  return std::sin(x);
}
// Function to use cos in expressions
inline double cosE(const double& x, gtsam::OptionalJacobian<1, 1> Hx) {
  if (Hx) {
    *Hx = gtsam::Vector1(-std::sin(x));
  }
  return std::cos(x);
}
// Function to use exponents in expressions
inline double erf(const double& z, gtsam::OptionalJacobian<1, 1> Hz) {
  if (Hz) {
    *Hz = gtsam::Vector1(2 / sqrt(M_PI) * std::exp(-(z * z)));
  }
  return std::erf(z);
}
}  // namespace internal

template <int N>
using VectorN_ = gtsam::Expression<Eigen::Matrix<double, N, 1>>;

/// Expression version of indexing into vect
template <int I, int N>
inline gtsam::Double_ indexVector(const VectorN_<N>& v) {
  return gtsam::Double_(&internal::indexVector<I, N>, v);
}

/// Expression version of scalar product
inline gtsam::Vector2_ createVector(const gtsam::Double_ a,
                                    const gtsam::Double_ b) {
  return gtsam::Vector2_(&internal::createVector, a, b);
}

/// Expression version of scalar product
inline gtsam::Double_ operator/(const gtsam::Double_ n,
                                const gtsam::Double_ d) {
  return gtsam::Double_(&internal::Divide, n, d);
}

/// Expression version of scalar product
inline gtsam::Vector2_ operator*(const gtsam::Double_ n,
                                 const gtsam::Vector2_ d) {
  return gtsam::Vector2_(&internal::prodE, n, d);
}

/// Expression version of negation
template <typename T>
inline gtsam::Expression<T> operator-(const gtsam::Expression<T> x) {
  return gtsam::Expression<T>(
      [](T x, typename gtsam::MakeOptionalJacobian<T, T>::type H) {
        if (H)
          *H = -Eigen::Matrix<double, gtsam::traits<T>::dimension,
                              gtsam::traits<T>::dimension>::Identity();
        return -x;
      },
      x);
}

/// Expression version of scalar product
inline gtsam::Double_ power(const gtsam::Double_ x, const gtsam::Double_ p) {
  return gtsam::Double_(&internal::power, x, p);
}

/// Expression version of scalar product
inline gtsam::Double_ log(const gtsam::Double_ x) {
  return gtsam::Double_(&internal::logE, x);
}

/// Expression version of scalar exponentiation
inline gtsam::Double_ exp(const gtsam::Double_ x) {
  return gtsam::Double_(
      [](double x, gtsam::OptionalJacobian<1, 1> Hx) {
        double exp_x = std::exp(x);
        if (Hx) *Hx = gtsam::Vector1(exp_x);
        return exp_x;
      },
      x);
}

/// Expression version of cos(x)
inline gtsam::Double_ cos(const gtsam::Double_ x) {
  return gtsam::Double_(&internal::cosE, x);
}

/// Expression version of sin(x)
inline gtsam::Double_ sin(const gtsam::Double_ x) {
  return gtsam::Double_(&internal::sinE, x);
}

/// Expression version of scalar product
inline gtsam::Double_ erf(const gtsam::Double_ z) {
  return gtsam::Double_(&internal::erf, z);
}

static const double k_e = std::exp(1.0);
static const gtsam::Double_(k_sqrt2pi) = power(gtsam::Double_(2.0) *
                                                   gtsam::Double_(M_PI),
                                               gtsam::Double_(0.5));

}  // namespace art_skills
