/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testAvoidIndeterminateLinSys.cpp
 *  @author Juan-Diego Florez
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Vector.h>

#include <vector>

#include "../SparseSlnFactorExpression.h"

// We will also try this using an expression factor graph:
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// Finally, once all of the factors have been added to our factor graph, we will
// want to solve/optimize to graph to find the best (Maximum A Posteriori) set
// of variable values. GTSAM includes several nonlinear optimizers to perform
// this step. Here we will use the common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// Should the LM optimizer fail, we will use the GaussNewton optimizer, as it
// will error out if the system is indeterminate.
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the
// marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;
using namespace art_skills;

TEST(ExpressionSlnFactor, ILS) {
  // 1. Create a factor graph container and add factors to it
  ExpressionFactorGraph graph;

  // Create the keys we need for this simple example
  static Symbol strokeparam1('s', 1);
  static Symbol p1('p', 1);

  // Matrix for t, x, y, reference for each position factor
  Matrix53 data1;
  // data1 << 0.05, 5.831317245188627, 4.441185357623116,  //
  //     0.1, 26.48157388186604, 22.725466421452907,       //
  //     0.15, 42.53795979151706, 40.40680069519366,       //
  //     0.2, 48.274238955666, 47.68933377015613,          //
  //     0.25, 49.66918677322847, 49.55249101225169;
  // data1 << 0.05, 1.76579, 0.,  //
  //     0.10, 9.0322, 0.,        //
  //     0.15, 21.4449, 0.,       //
  //     0.20, 33.6345, 0.,       //
  //     0.25, 41.9882, 0.;
  data1 << 0.05, 1.50799612, 0.1,  //
      0.10, 8.11741321, 0.3,       //
      0.15, 20.21929084, 0.6,      //
      0.20, 32.72157755, 0.9,      //
      0.25, 42.94530815, 1;

  // create a measurement for both factors (the same in this case)
  auto position_noise =
      noiseModel::Diagonal::Sigmas(Vector2(0.02, 0.02));  // 2cm std on x,y

  // // Place prior factor on p0:
  // graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(
  //     p1, Vector2(0.0, 0.0), noiseModel::Isotropic::Sigma(2, 1000.0));

  // graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector6>>(
  //     strokeparam1, Vector6::Zero(), noiseModel::Isotropic::Sigma(6,
  //     1000.0));

  // For loop to create factors for each position, 5 pts per stroke
  for (int i = 1; i <= 5; i++) {
    graph.addExpressionFactor();
//        graph.addExpressionFactor(x1, Pose2(0, 0, 0), priorNoise);

    // graph.emplace_shared<ExpressionSlnFactor>(
    //     strokeparam1, p1, data1(i - 1, 0), data1.block<1, 2>(i - 1, 1),
    //     position_noise);
  }
  // EXPECT_LONGS_EQUAL(6, graph.size());

  // Print
  // graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  // starting with initial control point p0
  initialEstimate.insert(p1, Vector2(1., 1.));

  // Now for param initial guesses (t0, D, th1, th2, sigma, mu)
  // Syntaxes for constructing >4 sized vector:
  Vector6 sp1;

  //  -0.104, 70.711, -0.935, -0.15, 0.226, -1.586 are reference from 0 to 50,
  //  xy
  // sp1 << -0.2, 72.0, -0.8, -0.2, 0.3, -1.1;
  sp1 << -0.2, 80.0, 0.5, 0., 0.3, -1.;
  initialEstimate.insert(strokeparam1, sp1);

  // regression
  // EXPECT_DOUBLES_EQUAL(1.525, graph[0]->error(initialEstimate), 1);

  // Print
  initialEstimate.print("Initial Estimate:\n");
  GaussianFactorGraph gfgi = *graph.linearize(initialEstimate);
  cout << gfgi.jacobian().first << endl;

  NonlinearFactorGraph graphLM = graph;
  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtParams paramsLM;
  paramsLM.setVerbosity(
      "values");  // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
  LevenbergMarquardtOptimizer optimizerLM(graphLM, initialEstimate, paramsLM);
  Values resultLM = optimizerLM.optimize();

  GaussianFactorGraph gfg = *graph.linearize(resultLM);
  cout << gfg.jacobian().first << endl;
  resultLM.print("Final Result:\n");

  // NonlinearFactorGraph graphGN = graph;
  // Optimize using Gauss Newton optimization.
  // GaussNewtonParams paramsGN;
  // paramsGN.setVerbosity("error");  // SILENT, TERMINATION, ERROR, VALUES,
  // // DELTA, LINEAR
  // GaussNewtonOptimizer optimizerGN(graphGN, initialEstimate, paramsGN);
  // Values resultGN = optimizerGN.optimize();
  // resultGN.print("Final Result:\n");

  // Calculate and print marginal covariances for all variables
  cout.precision(2);
  Marginals marginals(graph, resultLM, Marginals::Factorization::QR);
  cout << "p1 covariance:\n" << marginals.marginalCovariance(p1) << endl;
  cout << "strokeparam1 covariance:\n"
       << marginals.marginalCovariance(strokeparam1) << endl;
  // cout << "p3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  // cout << "s1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  // cout << "s2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  // cout << "s3 covariance:\n" << marginals.marginalCovariance(3) << endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

// TODO: Gerry's wisdom
// Tune param of numerical derivative?
// In numerical integration, if dt is 1 second and I do 5.5 sec of integration,
//    there is 0.5 sec unaccounted for - Check for discrete remainders or
//    account for them.
// Tweak dt EG: t as t - t0, redefince dt = t/round(t/dt)
// Do real derivative instead of numerical derivative