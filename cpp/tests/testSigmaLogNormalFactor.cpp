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

/* ************************************************************************* */

// Sigma Log Normal "predict" function
TEST(SigmaLogNormalFactor, predict) {
  // pass
}

// Constructor scalar
TEST(SigmaLogNormalFactor, Constructor) {
  SharedNoiseModel model;
  SigmaLogNormalFactor factor(1, 1.0, model);
}

// Factor testing
TEST(SigmaLogNormalFactor, factorTesting) {
  // pass
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
