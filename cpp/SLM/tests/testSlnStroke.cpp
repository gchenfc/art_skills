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
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point2.h>

#include <iostream>

#include "../SlnStroke.h"

using namespace std;
using namespace gtsam;
using namespace art_skills;

// All the hardcoded values below are obtained from wSL method, using
// generate_wSL.ipynb, which uses the same lambda term, but uses an indirect
// integration method to approximate the speed and position, which varies from
// the direct approach from the SL method.

namespace example {
const Point2 xy(0, 0);
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
  double t = 0.01- example::t0;
  double lambda = stroke.log_impulse(t);
  EXPECT_DOUBLES_EQUAL(0.21847935659224005, lambda, 1e-6);
  EXPECT_DOUBLES_EQUAL(10.9239678296, stroke.speed(lambda), 1e-5);
}

TEST(Sln, direction) {
  using example::stroke;
  double t = 0.01 - example::t0;
  EXPECT_DOUBLES_EQUAL(0., stroke.direction(t), 1e-6);
}

// TODO: update these values/check they are correct
TEST(Sln, position) {
  using example::stroke;
  double t1 = 0.01;
  double dt = 0.01;
  // using approximation, no 1:1 trajectory to stroke mapping
  EXPECT(gtsam::assert_equal(gtsam::Point2(0.14819899, 0),
                             stroke.position(t1, dt)));
}

TEST(Sln, position2) {
  Point2 p2(50, 0);
  const double t0 = 0.02709954;
  const double D = 50;
  const double theta1 = 0;
  const double theta2 = 0;
  const double sigma = 0.22648023;
  const double mu = -1.07559856;
  const SlnStroke stroke(p2, t0, D, theta1, theta2, sigma, mu);

  double t2 = 0.4; // using approximation, no 1:1 trajectory to stroke mapping
  double dt = 0.01;
  EXPECT(gtsam::assert_equal(gtsam::Point2(82.103009, 0),
                             stroke.position(t2, dt)));
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
