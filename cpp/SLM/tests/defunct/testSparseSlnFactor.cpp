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

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;
using namespace art_skills;

TEST(SparseSlnFactor, WholeEnchilada) {
  // Create a factor graph for a 2 stroke trajectory
  NonlinearFactorGraph graph;

  // Create the keys we need for this simple example
  static Symbol strokeparam1('s', 1);
  static Symbol p1('p', 1);

  // Matrix for t, x, y, reference for each position factor
  Matrix53 data1;
  data1 << 0.05, 1.50799612, 0.,  //
      0.10, 8.11741321, 0.,        //
      0.15, 20.21929084, 0.,       //
      0.20, 32.72157755, 0.,       //
      0.25, 42.94530815, 0.;
  // data1 << 0.05, 1.76579, 0.,  //
  //     0.10, 9.0322, 0.,        //
  //     0.15, 21.4449, 0.,       //
  //     0.20, 33.6345, 0.,       //
  //     0.25, 41.9882, 0.;

  // create a measurement for both factors (the same in this case)
  auto position_noise =
      noiseModel::Diagonal::Sigmas(Vector2(0.02, 0.02));  // 2cm std on x,y

  // For loop to create factors for each position, 5 pts per stroke
  for (int i = 1; i <= 5; i++) {
    graph.emplace_shared<SparseSlnFactor>(
        strokeparam1, p1, data1(i-1, 0), data1.block<1, 2>(i-1, 1), position_noise);
  }
  EXPECT_LONGS_EQUAL(5, graph.size());

  // Print
  graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  // starting with initial control point p0
  initialEstimate.insert(p1, Vector2(1., 4.));
  
  // Now for param initial guesses (t0, D, th1, th2, sigma, mu)
  // Syntaxes for constructing >4 sized vector:
  Vector6 sp1;

  sp1 << -0.2, 80.0, 0.5, 0., 0.3, -1.;
  initialEstimate.insert(strokeparam1, sp1);

  // regression
  // EXPECT_DOUBLES_EQUAL(1.525, graph[0]->error(initialEstimate), 1);

  // Print
  initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // Calculate and print marginal covariances for all variables
  cout.precision(2);
  Marginals marginals(graph, result);
  cout << "p1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "p2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "p3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "s1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "s2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "s3 covariance:\n" << marginals.marginalCovariance(3) << endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}