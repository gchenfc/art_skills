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

namespace art_skills {

class SlnStrokeExpression {
  SlnStrokeExpression(gtsam::Key t0, gtsam::Key D, gtsam::Key theta1,
                      gtsam::Key theta2, gtsam::Key sigma, gtsam::Key mu);
  SlnStrokeExpression(gtsam::Key p);
  gtsam::NonlinearFactor::shared_ptr pos_integration_factor(
      size_t timestep, double dt,
      gtsam::noiseModel::Base::shared_ptr noise =
          gtsam::noiseModel::Isotropic::Sigma(2, 0.001));
};

}  // namespace art_skills
