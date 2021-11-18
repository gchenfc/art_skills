/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SparseSlnFactor.h
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
 * A factor associated with a single 2D mocap point at a given time.
 * It is assumed that the caller uses the time to select the right stroke
 * segment `S_i`, and passes in t0 and t to the constructor, where t0 is
 * the beginning time for that stroke i.
 * It is a unary factor that predicts the measurement given the parameters for
 * stroke i only, converted into a vector (not GTSAM-like, but hey).
 */
class SparseSlnFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector2> {
 public:
  using Vector = gtsam::Vector;
  using Vector2 = gtsam::Vector2;
  using Vector6 = gtsam::Vector6;
  typedef typename boost::shared_ptr<SparseSlnFactor> shared_ptr;
  typedef SparseSlnFactor This;

 private:
  using Base = NoiseModelFactor2<gtsam::Vector6, gtsam::Vector2>;
  double t_;   /** the time step corresponding to this data point */
  Vector2 xy_; /** The measured mocap data point */

 public:
  /** Constructor
   * @param parameters_key key for a SlnParameters struct (converted to a
   * vector)
   * @param t the time that this data point corresonds to
   * @param mocap_data_xy the data point we're fitting to
   */
  SparseSlnFactor(gtsam::Key parameters_key, gtsam::Key p0_key, double t,
                  const Vector2& mocap_data_xy,
                  const gtsam::SharedNoiseModel& model = nullptr)
      : Base(model, parameters_key, p0_key), t_(t), xy_(mocap_data_xy) {}

  /** vector of errors */
  Vector evaluateError(
      const Vector6& params, const Vector2& p0,  //
      boost::optional<gtsam::Matrix&> H_params = boost::none,
      boost::optional<gtsam::Matrix&> H_p0 = boost::none) const override {
    // lambda function for queryposition but without `t` argument
    auto predict = [this](const Vector6& params_vector, const Vector2& p0) {
      SlnStroke params;
      params.xy = p0;
      params.t0 = params_vector(0);
      params.D = params_vector(1);
      params.theta1 = params_vector(2);
      params.theta2 = params_vector(3);
      params.sigma = params_vector(4);
      params.mu = params_vector(5);

      return SigmaLogNormal::queryposition(params, t_, 0.01);
    };
    const Vector2 predicted_xy = predict(params, p0);

    // TODO(Gery+JD): fix this dynamic jacobian stuff
    if (H_params) {
      (*H_params) = gtsam::numericalDerivative21<Vector2, Vector6, Vector2>(
          predict, params, p0);
    }
    if (H_p0) {
      (*H_p0) = gtsam::numericalDerivative22<Vector2, Vector6, Vector2>(
          predict, params, p0);
    }
    return predicted_xy - xy_;
  }

  const Vector2& data_xy() const { return xy_; }
  double data_t() const { return t_; }

  // /** print */
  // void print(const std::string& s,
  //            const gtsam::KeyFormatter& keyFormatter =
  //                gtsam::DefaultKeyFormatter) const override {
  //   std::cout << s << "SparseSlnFactor on " << keyFormatter(this->key1())
  //             << "\n  data point: " << xy_.transpose() << "\n";
  //   if (this->noiseModel_)
  //     this->noiseModel_->print("noise model: ");
  //   else
  //     std::cout << "no noise model" << std::endl;
  // }
};

}  // namespace art_skills
