/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SigmaLogNormalFactor.h
 *  @author JD
 *  @author Gerry
 **/

#pragma once

#include <string>

#include <metis/include/metis.h>
#include <Eigen/Dense>



#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include "SigmaLogNormal.h"

namespace art_skills {

/**
 * A factor for fitting a SigmaLogNormal curve to data.  It is a unary factor
 * whose only variable is a SlnParameters data structure converted into a
 * vector.
 */
class SigmaLogNormalFactor : public NoiseModelFactor1<gtsam::Vector> {
 public:
  using Vector = gtsam::Vector;
  typedef typename boost::shared_ptr<SigmaLogNormalFactor> shared_ptr;
  typedef SigmaLogNormalFactor This;

 private:
  typedef NoiseModelFactor1<Vector> Base;

  int num_control_points_;
  double t_;          /** the time step corresponding to this data point */
  gtsam::Vector2 xy_; /** The measured mocap data point */

 public:
  /** Constructor
   * @param parameters_key key for a SlnParameters struct (converted to a
   * vector)
   * @param num_control_points the number of control points in the sigma log
   * normal parameters
   * @param t the time that this data point corresonds to
   * @param mocap_data_xy the data point we're fitting to
   */
  SigmaLogNormalFactor(Key parameters_key,  //
                       int num_control_points, double t,
                       const Vector2& mocap_data_xy,
                       const SharedNoiseModel& model = nullptr)
      : Base(model, parameters_key),
        num_control_points_(num_control_points),
        t_(t),
        xy_(mocap_data_xy) {}

  /** vector of errors */
  Vector evaluateError(
      const Vector& x,  //
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    // lambda function for querySigmaLogNormal but without `t` argument
    auto predict = [](const Vector& params_vector) {
      SlnParameters params = SlnParameters::fromVector(num_control_points_, x);
      return querySigmaLogNormal(params, t_);
    };
    gtsam::Vector2 predicted_xy = predict(x);

    // TODO(Gery+JD): fix this dynamic jacobian stuff
    if (H) {
      switch (num_control_points_) {
        case 3:
          (*H) = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector,
                                              3 + 3 * 5>(predict, x);
          break;
        case 4:
          (*H) = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector,
                                              3 + 4 * 5>(predict, x);
          break;
        case 5:
          (*H) = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector,
                                              3 + 5 * 5>(predict, x);
          break;
        case 6:
          (*H) = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Vector,
                                              3 + 6 * 5>(predict, x);
          break;
        default:
          throw std::runtime_error(
              "TODO: fix dynamic number of control points");
      }
    }
    return predicted_xy - xy_;
  }

  const gtsam::Vector2& prior() const { return xy_; }

  /** print */
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "SigmaLogNormalFactor on " << keyFormatter(this->key())
              << "\n"  //
              << "  data point: " << xy_;
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
  }
};

}  // namespace art_skills
