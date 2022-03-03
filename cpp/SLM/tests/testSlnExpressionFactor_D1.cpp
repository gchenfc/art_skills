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

#include "../SlnStrokeExpression.h"
#include "../SlnStroke.h"

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

  // Matrix for t, x, y, reference for each position factor
  Eigen::Matrix<double, 31, 3> data1;
  data1 << 0.000000000000000000e+00,3.254897368664543267e-01,-8.936601621630331227e-01, //
8.333333333333300258e-03,3.269525745636264746e-01,-8.900745220853243378e-01, //
1.666666666666660052e-02,3.291453697577698678e-01,-8.852161029552397808e-01, //
2.499999999999990077e-02,3.325515378590233051e-01,-8.787672389395526640e-01, //
3.333333333333320103e-02,3.359679291751946351e-01,-8.703859355971451661e-01, //
4.166666666666650476e-02,3.399923931724491144e-01,-8.602826055064219934e-01, //
4.999999999999980155e-02,3.442824786600893194e-01,-8.480163473642363670e-01, //
5.833333333333309834e-02,3.491627944160899943e-01,-8.333700759417705939e-01, //
6.666666666666640206e-02,3.539444568538996361e-01,-8.171722977308741864e-01, //
7.499999999999970579e-02,3.592159525538891618e-01,-7.986980555707734464e-01, //
8.333333333333300952e-02,3.650546213240065674e-01,-7.787279749159550235e-01, //
9.166666666666629937e-02,3.714736542000802166e-01,-7.570136703552537982e-01, //
9.999999999999960310e-02,3.778634091460778555e-01,-7.333904140887657075e-01, //
1.083333333333329068e-01,3.843281155915927449e-01,-7.087376395403870433e-01, //
1.166666666666661967e-01,3.909623919130325032e-01,-6.824655080692988429e-01, //
1.249999999999995004e-01,3.974843175627018055e-01,-6.555879713420276200e-01, //
1.333333333333328041e-01,4.039229239569012142e-01,-6.276668544291764684e-01, //
1.416666666666661079e-01,4.105347405162689256e-01,-5.992839591518326348e-01, //
1.499999999999994116e-01,4.170243715585888467e-01,-5.709676427131310517e-01, //
1.583333333333327153e-01,4.225962714348362459e-01,-5.424340160626357488e-01, //
1.666666666666660190e-01,4.286362937160109743e-01,-5.141503176252475438e-01, //
1.749999999999992950e-01,4.343193225662036472e-01,-4.862841172426772829e-01, //
1.833333333333325987e-01,4.388840687436094123e-01,-4.590381204102663704e-01, //
1.916666666666659025e-01,4.427400928215209919e-01,-4.330911887270484373e-01, //
1.999999999999992062e-01,4.457518663108581691e-01,-4.092233080346499019e-01, //
2.083333333333325099e-01,4.483554727983301014e-01,-3.883470340886382433e-01, //
2.166666666666658136e-01,4.505788138762804040e-01,-3.709451157561682866e-01, //
2.249999999999991174e-01,4.521774015580295458e-01,-3.582143602029230767e-01, //
2.333333333333323933e-01,4.540392733371415579e-01,-3.502929538788304153e-01, //
2.416666666666656971e-01,4.553027656528993994e-01,-3.465550328228148569e-01, //
2.499999999999990008e-01,4.569055513590115636e-01,-3.463931905575849957e-01; //

  int multiplier = 2;
  //int i = 0;
  int k_data1_limit = data1.rows()*multiplier;
  double dt = (data1.row(1)(0)-data1.row(0)(0))/2;


  // Create the keys we need for this simple example
  static Symbol strokeparam1('s', 1);
  static Symbol p1('x', 0);

  // create a measurement for both factors (the same in this case)
  auto position_noise =
      noiseModel::Diagonal::Sigmas(Vector2(0.02, 0.02));  // 2cm std on x,y

  SlnStrokeExpression stroke1(strokeparam1);
  for (int k = 0; k < k_data1_limit; k++) {
    graph.add(stroke1.pos_integration_factor(k, dt));
  }

  // For loop to place priors at data/measured points
  for (int k = 0; k < data1.rows(); k++) {
    // this is following c++ way to cast to a type
    // we set timestep equal to the time at row(k)/dt
    size_t timestep = static_cast<size_t>(data1.row(k)(0) / dt);
    // now we check if timestep*dt is equal to the data time in row k
    assert_equal(timestep * dt, data1.row(k)(0));
    // does the assert equal ensure that ONLY matching timesteps get a prior?
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(
        gtsam::symbol('x', timestep), data1.row(k).tail<2>(), position_noise);
        // TODO: potentially replace with 2*k to place at each actual point
  }

  // Print
  //graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  Vector6 sp1;
  sp1 << -0.2, 0.4, 10*M_PI/180, 0*M_PI/180, 0.5, -1.8;
  initialEstimate.insert(strokeparam1, sp1);

  // Maybe TODO: initialize this based on data
  for (int k = 0; k <= k_data1_limit; k++) {
    initialEstimate.insert(gtsam::symbol('x', k), Vector2(0.0, 0.0));
    // if (k % multiplier == 0){
    //   i = k/multiplier;
    // }
    // if (k < k_data1_limit){
    //   initialEstimate.insert(gtsam::symbol('x', k), Vector2(data1(i,1), data1(i,2)));
    // } else{
    //   initialEstimate.insert(gtsam::symbol('x', k), Vector2(data1(data1.rows()-1,1), data1(data1.rows()-1,2)));
    // }
  }

  NonlinearFactorGraph graphLM = graph;
  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtParams paramsLM;
  paramsLM.setMaxIterations(1000);
  paramsLM.setVerbosity(
      "ERROR");  // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
  LevenbergMarquardtOptimizer optimizerLM(graphLM, initialEstimate, paramsLM);
  Values resultLM = optimizerLM.optimize();

  resultLM.print("Final Result:\n");

  // Create the stroke
  {  // const SlnStroke stroke(xy, t0, D, theta1, theta2, sigma, mu);
    Vector6 params1 = resultLM.at<Vector6>(strokeparam1);
    Point2 xy1 = resultLM.at<Point2>(p1);
    Point2 xy1_0 = SlnStroke (Point2::Zero(), params1).position(0,dt);
    const SlnStroke stroke1(xy1-xy1_0, params1);
    // populate headers
    std::ofstream myfile1;
    myfile1.open("gtsam1_stroke1.csv");
    myfile1 << "time,x,y\n";
    for (int k = 0; k < k_data1_limit; k++) {
      double k_t = k * dt;
      Point2 pt1 = stroke1.position(k_t, dt);
      myfile1 << k_t << "," << pt1(0) << "," << pt1(1) << "\n";
    }
    myfile1.close();
  }
  //graph.saveGraph("tstSLN.dot", resultLM);

  // Calculate and print marginal covariances for all variables
  cout.precision(2);
  Marginals marginals(graph, resultLM, Marginals::Factorization::QR);
  cout << "strokeparam1 covariance:\n"
       << marginals.marginalCovariance(strokeparam1) << endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}