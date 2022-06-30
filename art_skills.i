/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file     art_skills.i
 * @brief    Wrapper interface file for Python
 * @author   Gerry Chen
 */

#include <SLM/SlnStrokeExpression.h>
#include <SLM/SlnStrokeExpression2.h>

namespace art_skills {

template <T = {double, gtsam::Point2, gtsam::Point3, gtsam::Vector6}>
class Expression {
  Expression(T value);
  Expression(gtsam::Key key);
  Expression(gtsam::Symbol symbol);
  Expression(unsigned char c, std::uint64_t j);

  std::set<gtsam::Key> keys();
  void dims(std::map<gtsam::Key, int> map);
  void print(std::string s);
  T value(gtsam::Values values);
};

// ExpressionFactor should be wrapped in GTSAM, but going to use this for now
template <T = {gtsam::Point2}>
class ExpressionFactor : gtsam::NoiseModelFactor {
  ExpressionFactor(gtsam::SharedNoiseModel noiseModel,  //
                   T measurement, gtsam::Expression<T> expression);

  T measured();
  void print(std::string s = "");
  void print(std::string s = "", gtsam::KeyFormatter keyFormatter);
  bool equals(const gtsam::NonlinearFactor& f, double tol);
};

class SlnStrokeExpression {
  SlnStrokeExpression(gtsam::Key t0, gtsam::Key D, gtsam::Key theta1,
                      gtsam::Key theta2, gtsam::Key sigma, gtsam::Key mu);
  SlnStrokeExpression(gtsam::Key p);
  SlnStrokeExpression(double t0, double D, double theta1, double theta2,
                      double sigma, double mu);
  SlnStrokeExpression(gtsam::Vector6 p);

  static SlnStrokeExpression CreateSlnStrokeExpressionReparameterized(
      gtsam::Key t0, gtsam::Key logD, gtsam::Key theta1, gtsam::Key theta2,
      gtsam::Key logSigma, gtsam::Key mu);
  static SlnStrokeExpression CreateSlnStrokeExpressionReparameterized(
      gtsam::Key p);
  static SlnStrokeExpression CreateSlnStrokeExpressionReparameterized(
      double t0, double logD, double theta1, double theta2, double logSigma,
      double mu);
  static SlnStrokeExpression CreateSlnStrokeExpressionReparameterized(
      gtsam::Vector6 p);

  gtsam::NonlinearFactor::shared_ptr pos_integration_factor(
      size_t timestep, double dt,
      gtsam::noiseModel::Base::shared_ptr noise =
          gtsam::noiseModel::Isotropic::Sigma(2, 0.001));

  gtsam::Vector2 displacement(double t, double dt, bool is_absolute_time = true,
                              const gtsam::Values& values = gtsam::Values());
  double speed(double t, bool is_absolute_time = true,
               const gtsam::Values& values = gtsam::Values());
};

class SlnStrokeExpression2 {
  SlnStrokeExpression2(gtsam::Key t0, gtsam::Key D, gtsam::Key theta1,
                       gtsam::Key theta2, gtsam::Key sigma, gtsam::Key mu);
  SlnStrokeExpression2(gtsam::Key p);
  SlnStrokeExpression2(double t0, double D, double theta1, double theta2,
                       double sigma, double mu);
  SlnStrokeExpression2(gtsam::Vector6 p);

  static SlnStrokeExpression2 CreateSlnStrokeExpressionReparameterized(
      gtsam::Key t0, gtsam::Key logD, gtsam::Key theta1, gtsam::Key theta2,
      gtsam::Key logSigma, gtsam::Key mu);
  static SlnStrokeExpression2 CreateSlnStrokeExpressionReparameterized(
      gtsam::Key p);
  static SlnStrokeExpression2 CreateSlnStrokeExpressionReparameterized(
      double t0, double logD, double theta1, double theta2, double logSigma,
      double mu);
  static SlnStrokeExpression2 CreateSlnStrokeExpressionReparameterized(
      gtsam::Vector6 p);

  gtsam::Double_ s(gtsam::Double_ t);
  gtsam::Double_ theta(gtsam::Double_ s);
  gtsam::Vector2_ pos(gtsam::Double_ t);
  gtsam::Vector2_ pos(gtsam::Double_ t, gtsam::Vector2_ x0, double eps = 1e-8);
  gtsam::Double_ speed(gtsam::Double_ t);
  gtsam::Double_ time(double t, bool is_absolute_time = true);

  gtsam::Vector2 pos(double t, bool is_absolute_time = true,
                     gtsam::Vector2 x0 = gtsam::Vector2::Zero(),
                     gtsam::Values values = gtsam::Values());
  double speed(double t, bool is_absolute_time = true,
               gtsam::Values values = gtsam::Values());
};

}  // namespace art_skills
