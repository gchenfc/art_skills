/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  test3strokes.cpp
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
  static Symbol strokeparam1('s', 1), strokeparam2('s', 2),
      strokeparam3('s', 3);
  static Symbol p1('p', 1), p2('p', 2), p3('p', 3);

  // If P02 is supposedto start where stroke1 ends, need a new factor that uses
  // P02 as position, modified SparseSLNFactor, position is a variable

  // Matrix for t, x, y, reference for each position factor
  Matrix53 data1;
  data1 << 0.03, -1.0588211175068385, 3.21476955763894,  //
      0.06, -6.040947944123917, 17.186781117060498,      //
      0.09, -16.581934532911358, 42.21314692859564,      //
      0.12, -29.080979538535388, 66.99193412646014,      //
      0.15, -34.97986028607607, 82.45346041336306;

  Matrix53 data2;
  data2 << 0.21, 11.421246711099897, 79.72398320993189,  //
      0.23, 36.30200185696294, 75.80893935604831,        //
      0.25, 57.27125395494195, 73.4258580338577,         //
      0.27, 70.05670258005732, 73.67230617679027,        //
      0.29, 71.18552295460876, 77.85712710704733;

  Matrix53 data3;
  data3 << 0.34, 29.72139849674609, 99.99954920284554,  //
      0.37, -0.7617341894927137, 110.37527498097201,    //
      0.4, -21.258934709588686, 115.87819884897615,     //
      0.43, -32.05967962148455, 118.35600811772005,     //
      0.46, -36.91095380654987, 119.37825709663488;

  // create a measurement for both factors (the same in this case)
  auto position_noise =
      noiseModel::Diagonal::Sigmas(Vector2(0.02, 0.02));  // 2cm std on x,y

  // For loop to create factors for each position, 5 pts per stroke
  for (int i = 1; i <= 5; i++) {
    graph.emplace_shared<SparseSlnFactor>(strokeparam1, p1, data1(i - 1, 0),
                                          data1.block<1, 2>(i - 1, 1),
                                          position_noise);
  }
  for (int i = 1; i <= 5; i++) {
    graph.emplace_shared<SparseSlnFactor>(strokeparam2, p2, data2(i - 1, 0),
                                          data2.block<1, 2>(i - 1, 1),
                                          position_noise);
  }
  for (int i = 1; i <= 5; i++) {
    graph.emplace_shared<SparseSlnFactor>(strokeparam3, p3, data3(i - 1, 0),
                                          data3.block<1, 2>(i - 1, 1),
                                          position_noise);
  }
  // EXPECT_LONGS_EQUAL(15, graph.size());

  // Print
  graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  // starting with initial control point p0
  initialEstimate.insert(p1, Vector2(0., 0.));
  initialEstimate.insert(p2, Vector2(-50, 100));
  initialEstimate.insert(p3, Vector2(100, 70));


  // TODO: mathematically relate theta to th1 and th2
  // Now for param initial guesses (t0, D, th1, th2, sigma, mu)
  // Syntaxes for constructing >4 sized vector:
  Vector6 sp1;
  // -0.104, 111.803, 2.034, 2.034, 0.226, -1.586;
  sp1 << -0.104, 111.803, 2.034, 2.034, 0.226, -1.586;
  // sp1 << -0.104, 111.803, 0.314, 2.348, 0.226, -1.586;
  // sp1 << -0.98, 112, 2, 1.7, 0.19, -1.54;
  initialEstimate.insert(strokeparam1, sp1);
  Vector6 sp2;
  // 0.016, 152.971, -0.197, -0.197, 0.226, -1.586;
  sp2 << 0.016, 152.971, -0.197, -0.197, 0.226, -1.586;
  // sp2 << 0.016, 152.971, -1.918, -2.116, 0.226, -1.586;
  // sp2 << 0.01, 150, -0.22, -0.16, 0.25, -1.5;
  initialEstimate.insert(strokeparam2, sp2);
  Vector6 sp3;
  // 0.136, 148.661, 2.799, 2.799, 0.226, -1.586;
  sp3 << 0.136, 148.661, 2.799, 2.799, 0.226, -1.586;
  // sp3 << 0.136, 148.661, 1.078, 3.876, 0.226, -1.586;
  // sp3 << 0.16, 150, 2.8, 2.6, 0.25, -1.68;
  initialEstimate.insert(strokeparam3, sp3);

  // Print
  initialEstimate.print("Initial Estimate:\n");

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

  // Position
  std::cout << "\nPosition" << std::endl;
  SlnStroke stroke1(result.at<Vector2>(p1), result.at<Vector6>(strokeparam1));
  std::cout << stroke1.position(0.03, 0.01) << std::endl;
  std::cout << stroke1.position(0.06, 0.01) << std::endl;
  std::cout << stroke1.position(0.09, 0.01) << std::endl;
  std::cout << stroke1.position(0.12, 0.01) << std::endl;
  std::cout << stroke1.position(0.15, 0.01) << std::endl;

  SlnStroke stroke2(result.at<Vector2>(p2), result.at<Vector6>(strokeparam2));
  std::cout << stroke2.position(0.21, 0.01) << std::endl;
  std::cout << stroke2.position(0.23, 0.01) << std::endl;
  std::cout << stroke2.position(0.25, 0.01) << std::endl;
  std::cout << stroke2.position(0.27, 0.01) << std::endl;
  std::cout << stroke2.position(0.29, 0.01) << std::endl;


  SlnStroke stroke3(result.at<Vector2>(p3), result.at<Vector6>(strokeparam3));
  std::cout << stroke3.position(0.34, 0.01) << std::endl;
  std::cout << stroke3.position(0.37, 0.01) << std::endl;
  std::cout << stroke3.position(0.40, 0.01) << std::endl;
  std::cout << stroke3.position(0.43, 0.01) << std::endl;
  std::cout << stroke3.position(0.46, 0.01) << std::endl;

  // Velocity
  std::cout << "\nVelocity" << std::endl;
  std::cout << stroke1.speed(stroke1.log_impulse(0.03)) << std::endl;
  std::cout << stroke1.speed(stroke1.log_impulse(0.06)) << std::endl;
  std::cout << stroke1.speed(stroke1.log_impulse(0.09)) << std::endl;
  std::cout << stroke1.speed(stroke1.log_impulse(0.12)) << std::endl;
  std::cout << stroke1.speed(stroke1.log_impulse(0.15)) << std::endl;

  std::cout << stroke2.speed(stroke2.log_impulse(0.21)) << std::endl;
  std::cout << stroke2.speed(stroke2.log_impulse(0.23)) << std::endl;
  std::cout << stroke2.speed(stroke2.log_impulse(0.25)) << std::endl;
  std::cout << stroke2.speed(stroke2.log_impulse(0.27)) << std::endl;
  std::cout << stroke2.speed(stroke2.log_impulse(0.29)) << std::endl;

  std::cout << stroke3.speed(stroke3.log_impulse(0.34)) << std::endl;
  std::cout << stroke3.speed(stroke3.log_impulse(0.37)) << std::endl;
  std::cout << stroke3.speed(stroke3.log_impulse(0.40)) << std::endl;
  std::cout << stroke3.speed(stroke3.log_impulse(0.43)) << std::endl;
  std::cout << stroke3.speed(stroke3.log_impulse(0.46)) << std::endl;

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