/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SparseSumSlnFactors.h
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 **/

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

#include "SigmaLogNormal.h"

namespace art_skills {

/**
 * A factor associated with the lognormal impulse of a stroke 'S_i'. It is assumed that
 * a good initial estimate for the parameters {mu, sigma, t0} are used, such that t0 can
 * be used to place the 'S_i' along the trajectory, where t0 is the start time of S_i.
 */
class Delta : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 public:
  using Vector = gtsam::Vector;
  using Vector2 = gtsam::Vector2;
  typedef typename boost::shared_ptr<Delta> shared_ptr;
  typedef Delta This;

 private:
  typedef NoiseModelFactor1<Vector> Base;

  int num_strokes_;
  double t_;       /** the time step corresponding to this data point */
  double y_;     /** The measured mocap data point */

 public:
  /** Constructor
   * @param parameters_key key for a SlnParameters struct (converted to a
   * vector)
   * @param t the time that this data point corresponds to
   * @param mocap_delta_y the discrete value at time t on the impulse we're fitting to
   */
  Delta(gtsam::Key parameters_key,
                       double t,
                       const double mocap_delta_y,
                       const gtsam::SharedNoiseModel& model = nullptr)
      : Base(model, parameters_key),
        t_(t),
        y_(mocap_delta_y) {}

  /** vector of errors 
   * TODO: JD, check/refactor evaluateError
  */
  Vector evaluateError(
      const Vector& x,  //
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // lambda function for querySigmaLogNormal but without `t` argument
    auto predict = [this, &x](const Vector& params_vector) {
      SlnParameters params = SlnParameters::fromVector(num_strokes_, x);
      return querySigmaLogNormal(params, t_);
    };
    Vector2 predicted_xy = predict(x);

    // TODO(Gery+JD): fix this dynamic jacobian stuff
    if (H) {
      switch (num_strokes_) {
        case 3:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 3 * 5>(
                  predict, x);
          break;
        case 4:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 4 * 5>(
                  predict, x);
          break;
        case 5:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 5 * 5>(
                  predict, x);
          break;
        case 6:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 6 * 5>(
                  predict, x);
          break;
        default:
          throw std::runtime_error(
              "TODO: fix dynamic number of control points");
      }
    }
    return predicted_xy - xy_;
  }

  const Vector2& prior() const { return x_; }

  // /** print */
  // void print(const std::string& s,
  //            const gtsam::KeyFormatter& keyFormatter =
  //                gtsam::DefaultKeyFormatter) const override {
  //   std::cout << s << "SigmaLogNormalFactor on " << keyFormatter(this->key())
  //             << "\n"  //
  //             << "  data point: " << xy_;
  //   if (this->noiseModel_)
  //     this->noiseModel_->print("  noise model: ");
  //   else
  //     std::cout << "no noise model" << std::endl;
  // }
};


/**
 * A factor associated with the velocity of a stroke 'S_i'. It takes D as a parameter and
 * is a binary factor that relates Delta and Velocity.
 */
class Velocity : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 public:
  using Vector = gtsam::Vector;
  using Vector2 = gtsam::Vector2;
  typedef typename boost::shared_ptr<Delta> shared_ptr;
  typedef Velocity This;

 private:
  typedef NoiseModelFactor1<Vector> Base;

  double t_;       /** the time step corresponding to this data point */
  Vector2 vel_;     /** The measured mocap SPEED value */

 public:
  /** Constructor
   * @param parameters_key key for a SlnParameters struct (converted to a
   * vector)
   * @param t the time that this data point corresonds to
   * @param mocap_vel an instantaneous speed in the vel profile we're fitting to
   */
  Velocity(gtsam::Key parameters_key,
                       double t,
                       const Vector2& mocap_vel,
                       const gtsam::SharedNoiseModel& model = nullptr)
      : Base(model, parameters_key),
        t_(t),
        vel_(mocap_vel) {}

  /** vector of errors 
   * TODO: JD, check/refactor evaluateError
  */  Vector evaluateError(
      const Vector& x,  //
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // lambda function for querySigmaLogNormal but without `t` argument
    auto predict = [this, &x](const Vector& params_vector) {
      SlnParameters params = SlnParameters::fromVector(num_strokes_, x);
      return querySigmaLogNormal(params, t_);
    };
    Vector2 predicted_xy = predict(x);

    // TODO(Gery+JD): fix this dynamic jacobian stuff
    if (H) {
      switch (num_strokes_) {
        case 3:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 3 * 5>(
                  predict, x);
          break;
        case 4:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 4 * 5>(
                  predict, x);
          break;
        case 5:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 5 * 5>(
                  predict, x);
          break;
        case 6:
          (*H) =
              gtsam::numericalDerivative11<Vector2, gtsam::Vector, 3 + 6 * 5>(
                  predict, x);
          break;
        default:
          throw std::runtime_error(
              "TODO: fix dynamic number of control points");
      }
    }
    return predicted_xy - xy_;
  }

  const Vector2& prior() const { return vel_; }

  // /** print */
  // void print(const std::string& s,
  //            const gtsam::KeyFormatter& keyFormatter =
  //                gtsam::DefaultKeyFormatter) const override {
  //   std::cout << s << "SigmaLogNormalFactor on " << keyFormatter(this->key())
  //             << "\n"  //
  //             << "  data point: " << xy_;
  //   if (this->noiseModel_)
  //     this->noiseModel_->print("  noise model: ");
  //   else
  //     std::cout << "no noise model" << std::endl;
  // }
};


}  // namespace art_skills
