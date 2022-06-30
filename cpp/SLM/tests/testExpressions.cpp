/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testExpressions.cpp
 *  @brief test the Expression operations in expressions.h for correctness
 *  @author Gerry Chen
 *  @author Juan-Diego Florez
 *  @author Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/expressionTesting.h>

#include <iostream>

#include "../expressions.h"

using namespace std;
using namespace gtsam;
using namespace art_skills;

// Constants for testing
const Double_ a = 2.0, b = 3.0, a_(Key(1)), b_(Key(2));
const Vector2_ x(Vector2(4.0, 5.0)), x_(Key(3));
const Vector6_ p((Vector6() << 6., 7., 8., 9., 10., 11.).finished()),
    p_(Key(4));

// begin helpers
template <typename T>
T get(Expression<T> e) {
  return e.value(Values());
}

Values values() {
  Values values;
  values.insert(1, get(a));
  values.insert(2, get(b));
  values.insert(3, get(x));
  values.insert(4, get(p));
  return values;
}

#define EXPECT_JACOBIANS(exp) \
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(exp, values(), 1e-5, 1e-5)
// end helpers

TEST(expressions, indexVector) {
  EXPECT_DOUBLES_EQUAL(6., get(indexVector<0>(p)), 1e-9);
  EXPECT_DOUBLES_EQUAL(7., get(indexVector<1>(p)), 1e-9);
  EXPECT_DOUBLES_EQUAL(8., get(indexVector<2>(p)), 1e-9);
  EXPECT_DOUBLES_EQUAL(9., get(indexVector<3>(p)), 1e-9);
  EXPECT_DOUBLES_EQUAL(10., get(indexVector<4>(p)), 1e-9);
  EXPECT_DOUBLES_EQUAL(11., get(indexVector<5>(p)), 1e-9);
  EXPECT_JACOBIANS(indexVector<0>(p_));
  EXPECT_JACOBIANS(indexVector<1>(p_));
  EXPECT_JACOBIANS(indexVector<2>(p_));
  EXPECT_JACOBIANS(indexVector<3>(p_));
  EXPECT_JACOBIANS(indexVector<4>(p_));
  EXPECT_JACOBIANS(indexVector<5>(p_));
}

TEST(expressions, createVector) {
  Vector2_ ab = createVector(a, b);
  EXPECT_DOUBLES_EQUAL(2., get(indexVector<0>(ab)), 1e-9);
  EXPECT_DOUBLES_EQUAL(3., get(indexVector<1>(ab)), 1e-9);
  EXPECT_JACOBIANS(createVector(a_, b_));
}

TEST(expressions, multiply_scalar_vector) {
  EXPECT(equal(Vector2(8., 10.), get(a * x), 1e-9));
  EXPECT_JACOBIANS(a_ * x_);
}

TEST(expressions, basic_double_arithmetic) {
  EXPECT_DOUBLES_EQUAL(2. / 3., get(a / b), 1e-9);
  EXPECT_JACOBIANS(a_ / b_);

  EXPECT_DOUBLES_EQUAL(8., get(power(a, b)), 1e-9);
  EXPECT_JACOBIANS(power(a_, b_));

  EXPECT_DOUBLES_EQUAL(0.6931471806, get(log(a)), 1e-9);
  EXPECT_JACOBIANS(log(a_));

  EXPECT_DOUBLES_EQUAL(7.3890560989, get(exp(a)), 1e-9);
  EXPECT_JACOBIANS(exp(a_));

  EXPECT_DOUBLES_EQUAL(-0.4161468365, get(cos(a)), 1e-9);
  EXPECT_JACOBIANS(cos(a_));

  EXPECT_DOUBLES_EQUAL(0.9092974268, get(sin(a)), 1e-9);
  EXPECT_JACOBIANS(sin(a_));

  EXPECT_DOUBLES_EQUAL(0.995322265, get(erf(a)), 1e-9);
  EXPECT_JACOBIANS(erf(a_));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
