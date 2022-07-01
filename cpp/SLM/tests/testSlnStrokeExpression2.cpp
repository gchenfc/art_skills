/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSlnStrokeExpression2.cpp
 *  @author Gerry Chen
 *  @author Juan-Diego Florez
 *  @author Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point2.h>

#include <iostream>

#include "../SlnStrokeExpression2.h"
#include <gtsam/nonlinear/expressionTesting.h>

using namespace std;
using namespace gtsam;

template <typename T>
T get(Expression<T> e) {
  return e.value(Values());
}

// Used for testing (to access private variables)
class SlnStrokeExpression2 : public art_skills::SlnStrokeExpression2 {
 public:
  using art_skills::SlnStrokeExpression2::SlnStrokeExpression2;
  SlnStrokeExpression2(const art_skills::SlnStrokeExpression2& other)
      : art_skills::SlnStrokeExpression2(other) {}
  bool equals(const SlnStrokeExpression2& expr2, double tol = 1e-9) const {
    return equal(get(t0), get(expr2.t0), tol) &&
           equal(get(D), get(expr2.D), tol) &&
           equal(get(theta1), get(expr2.theta1), tol) &&
           equal(get(theta2), get(expr2.theta2), tol) &&
           equal(get(sigma), get(expr2.sigma), tol) &&
           equal(get(mu), get(expr2.mu), tol);
  }
};
template <>
struct gtsam::traits<SlnStrokeExpression2>
    : public Testable<SlnStrokeExpression2> {};

// Define constants that will be used throughout the tests
const Vector2_ xy(Vector2(0.0, 0.0));
const Double_ t0 = -0.17290046, D = 50.0, theta1 = 0., theta2 = 10.,
              sigma = 0.22648023, mu = -1.07559856;
const Vector6_ params = (Vector6() << get(t0), get(D), get(theta1), get(theta2),
                         get(sigma), get(mu))
                            .finished();
const SlnStrokeExpression2 stroke(t0, D, theta1, theta2, sigma, mu);
const SlnStrokeExpression2 stroke2(Key(1), Key(2), Key(3), Key(4), Key(5),
                                   Key(6));
// end constants

TEST(SlnStrokeExpression2, constructors) {
  EXPECT(equal(stroke, SlnStrokeExpression2(params)));
  EXPECT(
      equal(stroke,
            SlnStrokeExpression2(
                SlnStrokeExpression2::CreateSlnStrokeExpressionReparameterized(
                    t0, art_skills::log(D), theta1, theta2,
                    art_skills::log(sigma), mu))));
}

TEST(Sln, s) {
  EXPECT_DOUBLES_EQUAL(1.0, get(stroke.s(99999999.)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.5, get(stroke.s(0.1681930692)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, get(stroke.s(-0.17290046)), 1e-9);
}

TEST(Sln, theta) {
  EXPECT_DOUBLES_EQUAL(0., get(stroke.theta(0.0)), 1e-9);
  EXPECT_DOUBLES_EQUAL(5., get(stroke.theta(0.5)), 1e-9);
  EXPECT_DOUBLES_EQUAL(10., get(stroke.theta(1.0)), 1e-9);
  // Double_ lambda = stroke.log_impulse(t);
  // EXPECT_DOUBLES_EQUAL(0.21847935659224005, get(lambda), 1e-6);
  // EXPECT_DOUBLES_EQUAL(10.9239678296, get(stroke.speed(lambda)), 1e-5);
}

TEST(Sln, pos) {
  EXPECT(equal(Vector2::Zero(), get(stroke.pos(Double_(-0.17290046))), 1e-5));
  EXPECT(equal(5.0 * Vector2(std::sin(5.0), 1 - std::cos(5.0)),
               get(stroke.pos(Double_(0.1681930692))), 1e-5));
  EXPECT(equal(5.0 * Vector2(std::sin(10.0), 1 - std::cos(10.0)),
               get(stroke.pos(Double_(999999999999.0))), 1e-5));
  Vector2 x0 = Vector2(1.1, 2.1);
  EXPECT(equal(Vector2::Zero() + x0, get(stroke.pos(Double_(-0.17290046), x0)),
               1e-5));
  EXPECT(equal(5.0 * Vector2(std::sin(5.0), 1 - std::cos(5.0)) + x0,
               get(stroke.pos(Double_(0.1681930692), x0)), 1e-5));
  EXPECT(equal(5.0 * Vector2(std::sin(10.0), 1 - std::cos(10.0)) + x0,
               get(stroke.pos(Double_(999999999999.0), x0)), 1e-5));
}

TEST(Sln, time) {
  EXPECT_DOUBLES_EQUAL(0.0, get(stroke.time(0.0)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.17290046, get(stroke.time(0.0, false)), 1e-9);
  EXPECT_DOUBLES_EQUAL(-0.17290046, get(stroke.time(-0.17290046)), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, get(stroke.time(0.17290046, false)), 1e-9);
}

TEST(Sln, pos_double) {
  EXPECT(equal(Vector2::Zero(), stroke.pos(-0.17290046), 1e-5));
  EXPECT(equal(5.0 * Vector2(std::sin(5.0), 1 - std::cos(5.0)),
               stroke.pos(0.1681930692), 1e-5));
  EXPECT(equal(5.0 * Vector2(std::sin(10.0), 1 - std::cos(10.0)),
               stroke.pos(999999999999), 1e-5));
}

TEST(Sln, speed_double) {
  EXPECT_DOUBLES_EQUAL(0, stroke.speed(-0.17290046), 1e-5);
  EXPECT_DOUBLES_EQUAL(0, stroke.speed(-0.17291), 1e-5);
  EXPECT_DOUBLES_EQUAL(5.1642384613, stroke.speed(0.1681930692), 1e-5);
  EXPECT_DOUBLES_EQUAL(0, stroke.speed(9999.0), 1e-5);
}

TEST(Sln, jacobian) {
  Values values;
  values.insert(0, get(xy));
  values.insert(1, get(t0));
  values.insert(2, get(D));
  values.insert(3, get(theta1));
  values.insert(4, get(theta2));
  values.insert(5, get(sigma));
  values.insert(6, get(mu));
  values.insert(7, 0.189);

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(stroke2.pos(Double_(Key(7))), values,
                                      1e-6, 1e-3);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
