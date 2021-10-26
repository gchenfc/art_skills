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
class SparseSlnFactor : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 public:
  using Vector = gtsam::Vector;
  using Vector2 = gtsam::Vector2;
  typedef typename boost::shared_ptr<SparseSlnFactor> shared_ptr;
  typedef SparseSlnFactor This;

 private:
  typedef NoiseModelFactor1<Vector> Base;

  int num_strokes_;
  double t_;       /** the time step corresponding to this data point */
  Vector2 xy_;     /** The measured mocap data point */

 public:
  /** Constructor
   * @param parameters_key key for a SlnParameters struct (converted to a
   * vector)
   * @param num_strokes the number of stroke in the SLN trajectory
   * @param t the time that this data point corresonds to
   * @param mocap_data_xy the data point we're fitting to
   */
  SparseSlnFactor(gtsam::Key parameters_key,
                       int num_strokes, double t,
                       const Vector2& mocap_data_xy,
                       const gtsam::SharedNoiseModel& model = nullptr)
      : Base(model, parameters_key),
        num_strokes_(num_strokes),
        t_(t),
        xy_(mocap_data_xy) {}

  /** vector of errors */
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

  const Vector2& prior() const { return xy_; }

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
