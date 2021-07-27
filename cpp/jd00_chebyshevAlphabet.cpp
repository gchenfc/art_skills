/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

// Generic GTSAM includes
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
// Chebyshev includes
#include <gtsam/basis/Chebyshev.h>
#include <gtsam/basis/FitBasis.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // Create linear factor graph
  NonlinearFactorGraph graph;

  // A "Key" represents a variable we're tryng to solve for
  // This variable is the chebyshev polynomial / coefficients, and we give it
  // the "id" 1
  Key coefficients_key(1);

  // Our data points that we're trying to fit to
  // x(t) = t
  const size_t m = 6;  // number of data points
  Vector t(m);
  t << -0.7, -0.4, 0.1, 0.3, 0.7, 0.9;
  Vector x(m);
  x << -0.7, -0.4, 0.1, 0.3, 0.7, 0.9;
  x = x.array() +
      5;  // make our data more interesting by translating it up by 5 units

  // Create a factor for each data point which says "try to make the polynomial
  // pass through this data point"
  for (size_t i = 0; i < m; i++) {
    // add a factor to the factor graph...
    //    * acting on our chebyshev polynomial / coefficients
    //    * with an objective that the polynomial passes through the point
    //    (t(i), x(i))
    // Refer to EvaluationFactor / Chebyshev1Basis for info on arguments
    graph.emplace_shared<EvaluationFactor<Chebyshev1Basis>>(
        coefficients_key,  // which variable (polynomial) this factor acts on
        x(i),              // y-coordinate of this data point
        noiseModel::Unit::Create(1),  // "model" is a noise model, which means
                                      // how much weight this data point carries
        3,                            // Degree of the polynomial
        t(i)                          // x-coordinate
    );
  }

  // Solve
  // First create an initial guess
  Values initial;
  initial.insert<Vector>(coefficients_key,
                         Vector::Zero(N));  // initial does not matter
  // Then optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Print out solution
  Vector coefficients = result.at<Vector>(coefficients_key);
  std::cout << "The solution is:\n\tx = ";
  for (int i = 0; i < coefficients.size(); ++i) {
    std::cout << coefficients(i) << " * t^" << i << " + ";
  }
  std::cout << std::endl;

  // // Predict x at time 1.0
  // Chebyshev1Basis::EvaluationFunctor f(N, 1.0);
  // Matrix H;
  // double actual = f(actual_c, H);
  // EXPECT_DOUBLES_EQUAL(1.0, actual, 1e-9);
}
