/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSigmaLogNormal.cpp
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 *  @author Frank Dellaert
 **/

// using wsl params, see if we return the correct position at time t
#include <CppUnitLite/TestHarness.h>

#include <iostream>

#include "../SlnStroke.h"

using namespace std;
using namespace gtsam;
using namespace art_skills;

namespace example {
const Vector2 xy = Vector2::Zero();
const double t0 = -0.17290046;
const double theta1 = 0.;
const double theta2 = 0.;
const double D = 50;
const double sigma = 0.22648023;
const double mu = -1.07559856;
const SlnStroke stroke(xy, t0, D, theta1, theta2, sigma, mu);
}  // namespace example

// namespace example
TEST(Sln, speed) {
  using example::stroke;
  double t = 0.01;
  double lambda = stroke.log_impulse(t);
  EXPECT_DOUBLES_EQUAL(0.21847935659224005, lambda, 1e-6);
  EXPECT_DOUBLES_EQUAL(10.9239678296, stroke.speed(lambda), 1e-5);
}

TEST(Sln, direction) {
  using example::stroke;
  double t = 0.01;
  EXPECT_DOUBLES_EQUAL(0., stroke.direction(t), 1e-6);
}

TEST(Sln, position) {
  using example::stroke;
  double t1 = 0.01;
  double dt = 0.01;
  EXPECT_DOUBLES_EQUAL(0.10923970609042487, stroke.position(t1, dt)[0], 1e-12);
}

TEST(Sln, position2) {
  Vector2 p2;
  p2 << 50., 0.;
  const double t0 = 0.02709954;
  const double D = 50;
  const double theta1 = 0;
  const double theta2 = 0;
  const double sigma = 0.22648023;
  const double mu = -1.07559856;
  const SlnStroke stroke(p2, t0, D, theta1, theta2, sigma, mu);

  // 0.14819898530609998 is the value obtained from wSL method, which uses the
  // same lambda term, but uses an indirect integration method to approximate
  // the speed and position, which varies from the direct approach from the
  // SL method. The below check is done using a value calculated externally,
  // using the lambda value from Sl and wSL:
  double t2 = 0.03;
  double dt = 0.01;
  EXPECT_DOUBLES_EQUAL(0.10923970609042487, stroke.position(t2, dt)[0], 1e-12);
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
