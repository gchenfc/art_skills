/**
 * @file   testSigmaLogNormalFactor.cpp
 * @brief  Test Sigma Log Normal Factor
 * @author JD
 * @author Gerry
 * @date   2021
 */

#include "SigmaLogNormalFactor.h"

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace art_skills;

// Constructor scalar
TEST(SigmaLogNormalFactor, Constructor) {
  Key key = 1;
  int num_control_points = 3;
  double t = 0.0;
  Vector2 mocap_data_xy(1.2, 3.4);
  SharedNoiseModel model;

  SigmaLogNormalFactor factor(key, num_control_points, t, mocap_data_xy, model);
}

// Factor testing
TEST(SigmaLogNormalFactor, factorTesting) {
  // TODO(JD): write unit test

  // this is pseudocode:
  /*

  SigmaLogNormalFactor factor(key, num_control_points, t, mocap_data_xy, model);

  SlnParameters test_parameters =  // create parameters that will exactly
                                   // generate mocap_data_xy

      Vector2 expected_error(0.0, 0.0);
  EXPECT_DOUBLES_EQUAL(expected_error, factor.evaluateError(test_parameters));

  test_parameters = // intentionally wrong parameters
  expected_error = Vector2(1.0, 2.0);
  EXPECT_DOUBLES_EQUAL(expected_error, factor.evaluateError(test_parameters));

  Values values;
  values.insert(key, test_parameters);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

  */


}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
