/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSparseSlnFactor.cpp
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 *  @author Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Vector.h>

#include <vector>

#include "../SparseSlnFactor.h"

// When the factors are created, we will add them to a Factor Graph. As the
// factors we are using are nonlinear factors, we will need a Nonlinear Factor
// Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// Finally, once all of the factors have been added to our factor graph, we will
// want to solve/optimize to graph to find the best (Maximum A Posteriori) set
// of variable values. GTSAM includes several nonlinear optimizers to perform
// this step. Here we will use the common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;
using namespace art_skills;

TEST(SparseSlnFactor, WholeEnchilada) {
  // Create a factor graph for a 2 stroke trajectory
  NonlinearFactorGraph graph;

  // Create the keys we need for this simple example
  static Symbol strokeparam1('s', 1), strokeparam2('s', 2);
  static Symbol p1('p', 1), p2('p', 2);

  // If P02 is supposedto start where stroke1 ends, need a new factor that uses
  // P02 as position, modified SparseSLNFactor, position is a variable

  // Matrix for t, x, y, reference for each position factor
  Matrix53 data1;
  data1 << 0.05, 1.50799612, 0.,  //
      0.10, 8.11741321, 0.,       //
      0.15, 20.21912658, 0.,      //
      0.20, 32.6542469, 0.,       //
      0.25, 41.50480693, 0.;

  Matrix53 data2;
  data2 << 0.34, 54.4648483, 0.,  //
      0.39, 66.90712126, 0.,      //
      0.44, 80.14918063, 0.,      //
      0.49, 89.94344084, 0.,      //
      0.54, 95.53231736, 0.;

  // create a measurement for both factors (the same in this case)
  auto position_noise =
      noiseModel::Diagonal::Sigmas(Vector2(0.02, 0.02));  // 2cm std on x,y

  // TODO: for loop to create factors for each position, 5 pts per stroke
  for (int i = 1; i <= 5; i++) {
    graph.emplace_shared<SparseSlnFactor>(
        strokeparam1, p1, data1(i, 0), data1.block<1, 2>(i, 1), position_noise);
    graph.emplace_shared<SparseSlnFactor>(
        strokeparam2, p2, data2(i, 0), data2.block<1, 2>(i, 1), position_noise);
  }
  EXPECT_LONGS_EQUAL(10, graph.size());

  // Print
  //   graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  // starting with initial control point p0
  initialEstimate.insert(p1, Vector2(2, -1));
  initialEstimate.insert(p2, Vector2(25, 2));
  // Now for param initial guesses (t0, D, th1, th2, sigma, mu)

  // Syntaxes for constructing >4 sized vector:
  Vector6 sp1;
  sp1 << 0.0, 29.0, 1.0, 1.0, 0.6, -3.5;
  initialEstimate.insert(strokeparam1, sp1);

  initialEstimate.insert(
      strokeparam2, (Vector6() << 0.0, 71.0, 1.0, 1.0, 0.5, -3.0).finished());

  // regression
  EXPECT_DOUBLES_EQUAL(48028, graph[0]->error(initialEstimate), 1);

  // Print
  //   initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtParams params;
  params.setVerbosity("error");
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}