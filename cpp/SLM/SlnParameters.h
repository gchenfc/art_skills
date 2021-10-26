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
  static constexpr int SIZE = 6;
  // gtsam::Vector2 xy; // point coordinate
  double t0; // start of impulse in global time
  double D; // amplitude of stroke (||P[i] - P[i-1]||)
  double theta1; // initial angular deviation
  double theta2; // initial angular deviation
  double sigma; // logresponse time
  double mu; // logtime delay

  static SlnStroke fromVector(gtsam::Vector6 vector) {
    SlnStroke stroke;
    stroke.t0 = vector(1);
    stroke.D = vector(2);
    stroke.theta1 = vector(3);
    stroke.theta2 = vector(4);
    stroke.sigma = vector(5);
    stroke.mu = vector(6);
    return stroke;
  }
};

/**
 * SlnParameters contains all the parameters needed to generate a SigmaLogNormal
 * trajectory
 */
struct SlnParameters {

  gtsam::Vector2 xy_0; // p0 of the entire trajectory
  std::vector<SlnStroke> stroke;
  // other parameters here

  static SlnParameters fromVector(int num_strokes,
                                  const gtsam::Vector &vector) {
    // Size check
    int expected_vector_size =
        2 +                                    // for xy_0
        num_strokes * SlnStroke::SIZE;  // for strokes
    if (expected_vector_size != vector.size()) {
      throw std::runtime_error(
          "Vector size doesn't match number of strokes!");
    }
    // process and pull out params
    SlnParameters params;
    params.xy_0 = vector.head<2>();           // xy_0
    for (int i = 0; i < num_strokes; ++i) {   // stroke
      params.stroke.push_back(SlnStroke::fromVector(
          vector.segment<6>(2 + i * SlnStroke::SIZE)));
    }
    return params;
  }

  static gtsam::Vector toVector(const SlnParameters &params) {
    // TODO(JD): implement this
    gtsam::Vector vec(2 + params.stroke.size()*6); // idk

    return vec;
  }
};

}  // namespace art_skills
