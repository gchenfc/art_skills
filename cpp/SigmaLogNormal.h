/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SigmaLogNormal.h
 *  @author JD
 *  @author Gerry
 **/

#pragma once

#include <gtsam/base/Vector.h>

#include "SlnParameters.h"

namespace art_skills {

/**
 * Compute the location of the sigma log normal curve at time t for some sigma
 * log normal parameters.
 * @param params The sigma log normal curve parameters
 * @param t The time to get the point at
 * @return The xy point of the sigma log normal curve at time t
 */
gtsam::Vector2 querySigmaLogNormal(const SlnParameters &params, double t) {
  // TODO(JD): implement this
  return gtsam::Vector2::Zero();
}

}  // namespace art_skills
