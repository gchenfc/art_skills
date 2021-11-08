// using wsl params, see if we return the correct position at time t
#include <CppUnitLite/TestHarness.h>
#include<iostream>
#include "../SigmaLogNormal.h"

using namespace std;
using namespace gtsam;
using namespace art_skills;

TEST(Sln, velocity) {
  double t0 = 0;
  double sigma = 0.22648023;
  double mu = -1.07559856;
  double t = 0.05;
  double lambda = SigmaLogNormal::logimpulse(sigma, mu, t0, t);
  EXPECT_DOUBLES_EQUAL(5.86302063, lambda, 1e-12);
  double D = 50;
  EXPECT_DOUBLES_EQUAL(5.86302063, SigmaLogNormal::velocity(D, lambda),
                       1e-12);
}

TEST(Sln, direction) {
  double t0 = 0;
  double theta1 = 0;
  double theta2 = 0;
  double sigma = 0.47238073;
  double mu = -2.96602237;
  double t = 0.01;

  EXPECT_DOUBLES_EQUAL(
      0, SigmaLogNormal::direction(theta1, theta2, mu, sigma, t0, t), 1e-12);
}

// TEST(Sln, position) {
//   Vector6 strokeparam;
//   strokeparam << 0.0, 29.0, 1.0, 1.0, 0.6, -3.5;
//   strokeparam =
//       gtsam::Vector7(xy = [ 0, 0 ], t0 = 0, D = 30, theta1 = 0, theta2 = 0,
//                      sigma = 0.47238073, mu = -2.96602237);
//   double t = 0.01;

//   EXPECT_DOUBLES_EQUAL(1.91996807e-09,
//                        SigmaLogNormal::position(strokeparam, t)[0], 1e-12);
// }
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
