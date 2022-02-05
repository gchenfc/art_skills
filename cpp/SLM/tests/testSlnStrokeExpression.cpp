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

#include "../SlnStrokeExpression.h"
#include <gtsam/nonlinear/expressionTesting.h>

using namespace std;
using namespace gtsam;
using namespace art_skills;

// All the hardcoded values below are obtained from wSL method, using
// generate_wSL.ipynb, which uses the same lambda term, but uses an indirect
// integration method to approximate the speed and position, which varies from
// the direct approach from the SL method.

namespace example {
const Vector2_ xy(Vector2(0.0, 0.0));
const Double_ t0 = -0.17290046;
const Double_ theta1 = 0.;
const Double_ theta2 = 0.;
const Double_ D = 50.0;
const Double_ sigma = 0.22648023;
const Double_ mu = -1.07559856;
const SlnStrokeExpression stroke(xy, t0, D, theta1, theta2, sigma, mu);
const SlnStrokeExpression stroke2(0, Key(1), Key(2), Key(3), Key(4), Key(5), Key(6));

}  // namespace example

// namespace example
TEST(Sln, speed) {
  using example::stroke;
  Double_ t = Double_(0.01) - example::t0;
  Double_ lambda = stroke.log_impulse(t);
  EXPECT_DOUBLES_EQUAL(0.21847935659224005, lambda.value(Values()),
                       1e-6);
  EXPECT_DOUBLES_EQUAL(10.9239678296,
                       stroke.speed(lambda).value(Values()), 1e-5);
}

TEST(Sln, direction) {
  using example::stroke;
  Double_ t = Double_(0.01) - example::t0;
  EXPECT_DOUBLES_EQUAL(0., stroke.direction(t).value(Values()), 1e-6);
}

// TODO: update these values/check they are correct
// TEST(Sln, position) {
//   using example::stroke;
//   double t1 = 0.01;

//   // using approximation, no 1:1 trajectory to stroke mapping
//   EXPECT(gtsam::assert_equal(gtsam::Vector2(0.14819899, 0.0),
//                              stroke.position(t1).value(Values())));
// }

TEST(Sln, jacobian) {
  using example::stroke2;
  double t1 = 0.01;
  Values values;
  values.insert(0, example::xy.value(Values()));
  values.insert(1, example::t0.value(Values()));
  values.insert(2, example::D.value(Values()));
  values.insert(3, example::theta1.value(Values()));
  values.insert(4, example::theta2.value(Values()));
  values.insert(5, example::sigma.value(Values()));
  values.insert(6, example::mu.value(Values()));

  Vector2_ pos = stroke2.position<53>(t1);

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(pos, values, 1e-6, 1e-3);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
