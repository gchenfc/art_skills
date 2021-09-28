/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  SlnParameters.h
 *  @author JD
 *  @author Gerry
 **/

#pragma once

#include <vector>

#include <gtsam/base/Vector.h>

namespace art_skills {

/**
 * SlnControlPoint defines a single control point.  A stroke goes between two
 * SlnControlPoints.
 */
struct SlnControlPoint {
  static constexpr int SIZE = 5;
  gtsam::Vector2 xy;
  double delta_t;
  double t;
  double ac;
  // other parameters here

  static SlnControlPoint fromVector(gtsam::Vector5 vector) {
    SlnControlPoint ctrl_point;
    ctrl_point.xy = vector.head<2>();
    ctrl_point.delta_t = vector(2);
    ctrl_point.t = vector(3);
    ctrl_point.ac = vector(4);
    return ctrl_point;
  }
};

/**
 * SlnParameters contains all the parameters needed to generate a SigmaLogNormal
 * trajectory
 */
struct SlnParameters {

  gtsam::Vector2 xy_0;
  std::vector<SlnControlPoint> control_points;
  double period;

  // other parameters here

  static SlnParameters fromVector(int num_control_points,
                                  const gtsam::Vector &vector) {
    // Size check
    int expected_vector_size =
        2 +                                           // for xy_0
        num_control_points * SlnControlPoint::SIZE +  // for control_points
        1;                                            // for period
    if (expected_vector_size != vector.size()) {
      throw std::runtime_error(
          "Vector size doesn't match number of control points!");
    }

    SlnParameters params;
    // xy_0
    params.xy_0 = vector.head<2>();
    // control_points
    for (int i = 0; i < num_control_points; ++i) {
      params.control_points.push_back(SlnControlPoint::fromVector(
          vector.segment<5>(2 + i * SlnControlPoint::SIZE)));
    }
    // period
    params.period = vector.at(vector.size() - 1);

    return params;
  }

  static gtsam::Vector toVector(const SlnParameters &params) {
    // TODO(JD): implement this
    gtsam::Vector vec(2 + params.control_points.size()*5 + 1); // idk

    return vec;
  }
};

}  // namespace art_skills
