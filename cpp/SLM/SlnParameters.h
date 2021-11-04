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

#include <vector>

#include <gtsam/base/Vector.h>

namespace art_skills {

/**
 * SlnStroke defines a single stroke, the length of which is defined by D along the trajectory.
 * The D of each stroke cannot overlap that of another stroke.
 * Theta1 and 2 describe the angular evolution of the stroke
 * Sigma and mu describe the shape of the lognormal
 * t0 defines the start time of the stroke along the trajectory
 */
struct SlnStroke {
  gtsam::Vector2 xy; // point coordinate
  double t0; // start of impulse in global time
  double D; // amplitude of stroke (||P[i] - P[i-1]||)
  double theta1; // initial angular deviation
  double theta2; // initial angular deviation
  double sigma; // logresponse time
  double mu; // logtime delay
  };

/**
 * SlnParameters contains all the parameters needed to generate a SigmaLogNormal
 * trajectory
 */
using SlnParameters = std::vector<SlnStroke>;
// other parameters here

}  // namespace art_skills
