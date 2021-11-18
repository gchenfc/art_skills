// using wsl params, see if we return the correct position at time t
#include <CppUnitLite/TestHarness.h>

#include <iostream>

#include "../SigmaLogNormal.h"

using namespace std;
using namespace gtsam;
using namespace art_skills;

TEST(Sln, velocity) {
  double t0 = -0.17290046;
  double sigma = 0.22648023;
  double mu = -1.07559856;
  double t = 0.01;
  double lambda = SigmaLogNormal::logimpulse(t0, sigma, mu, t);
  EXPECT_DOUBLES_EQUAL(0.21847935659224005, lambda, 1e-6);
  double D = 50;
  EXPECT_DOUBLES_EQUAL(10.9239678296, SigmaLogNormal::velocity(D, lambda),
                       1e-5);
}

TEST(Sln, direction) {
  double t0 = -0.17290046;
  double theta1 = 0.;
  double theta2 = 0.;
  double sigma = 0.22648023;
  double mu = -1.07559856;
  double t = 0.01;

  EXPECT_DOUBLES_EQUAL(
      0., SigmaLogNormal::direction(theta1, theta2, mu, sigma, t0, t), 1e-6);
}

TEST(Sln, position) {
  SlnStroke params;
  double dt = 0.01;

  double t1 = 0.01;
  Vector2 p1;
  p1 << 0., 0.;
  params.xy = p1;
  params.t0 = -0.17290046;
  params.D = 50;
  params.theta1 = 0;
  params.theta2 = 0;
  params.sigma = 0.22648023;
  params.mu = -1.07559856;

  EXPECT_DOUBLES_EQUAL(0.10923970609042487,
                       SigmaLogNormal::position(params, t1, dt)[0], 1e-12);

  double t2 = 0.03;                       
  Vector2 p2;
  p2 << 50., 0.;
  params.xy = p2;
  params.t0 = 0.02709954;
  params.D = 50;
  params.theta1 = 0;
  params.theta2 = 0;
  params.sigma = 0.22648023;
  params.mu = -1.07559856;

  // 0.14819898530609998 is the value obtained from wSL method, which uses the
  // same lambda term, but uses an indirect integration method to approximate
  // the velocity and position, which varies from the direct approach from the
  // SL method. The below check is done using a value calculated externally,
  // using the lambda value from Sl and wSL:
  EXPECT_DOUBLES_EQUAL(0.10923970609042487,
                       SigmaLogNormal::position(params, t2, dt)[0], 1e-12);
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
