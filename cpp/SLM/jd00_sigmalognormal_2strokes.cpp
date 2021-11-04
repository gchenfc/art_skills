/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  sigmalognormal_3strokes.cpp
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 **/

#pragma once

#include <gtsam/base/Vector.h>

#include <vector>

#include "SparseSlnFactor.h"

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

int main(int argc, char** argv) {
  // Create a factor graph for a 2 stroke trajectory
  NonlinearFactorGraph graph;

  // Create the keys we need for this simple example
  static Symbol strokeparam1('sp', 1), strokeparam2('sp', 2);
  static Symbol p01('p0', 1), p02('p0', 2);

  // Add a prior on p01 at the origin. A prior factor consists of a mean and
  // a noise model (covariance matrix)
  Vector2 prior(0.0, 0.0);  // prior mean is at origin, P0 is at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector2(0.0001, 0.0001));            // 0.1mm std on x,y
  graph.addPrior(p01, prior, priorNoise);  // add directly to graph

  Vector2 prior(30.0,
                0.0);  // prior mean is at origin, P0 is at start of stroke 2
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector2(0.0001, 0.0001));            // 0.1mm std on x,y
  graph.addPrior(p02, prior, priorNoise);  // add directly to graph

  // Add 2 position factors, measured from trajectories.ipynb
  Vector2 position1(1.19080574e+01, 0.00000000e+00);  // 5th point
  Vector2 position2(7.24578721e+01, 0.00000000e+00);  // 10th point
  // create a measurement for both factors (the same in this case)
  auto position_noise = noiseModel::Diagonal::Sigmas(
      Vector3(0.00001, 0.00001));  // 20cm std on x,y, 0.1 rad on theta
  graph.emplace_shared<SparseSlnFactor>(strokeparam1, p01, position1,
                                        position_noise);
  graph.emplace_shared<SparseSlnFactor>(strokeparam1, p01, position2,
                                        position_noise);

  // Print
  graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  // starting with initial control point p0
  initialEstimate.insert(p01, Vector2(0.0001, -0.0001));
  initialEstimate.insert(p02, Vector2(49.9999, 0.0001));
  // Now for param initial guesses (t0, D, th1, th2, sigma, mu)
  initialEstimate.insert(strokeparam1,
                         Vector6(0.0, 29.0, 1.0, 1.0, 0.6, -3.5));
  initialEstimate.insert(strokeparam2, Vector6(0.0, 71.0, 1.0, 1.0, 0.5, -3.0));

  // Print
  initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  return 0;
}
