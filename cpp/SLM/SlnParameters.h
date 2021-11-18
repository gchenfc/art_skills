/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SlnParameters.h
 *  @author Juan-Diego Florez
 *  @author Gerry Chen
 **/

#pragma once

#include <gtsam/base/Vector.h>

#include <vector>

#include "SlnStroke.h"

namespace art_skills {

/**
 * SlnParameters contains all the parameters needed to generate a SigmaLogNormal
 * trajectory
 */
using SlnParameters = std::vector<SlnStroke>;
// other parameters here

}  // namespace art_skills
