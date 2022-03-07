"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test python wrapper for SlnStrokeExpression
Author: Gerry Chen
"""

import unittest
import numpy as np
import art_skills
from art_skills import SlnStrokeExpression
import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P


class TestSlnStrokeExpression(GtsamTestCase):
    """Test SlnStrokeExpression wrapper."""

    def test_create_factor(self):
        """Test constructor and factor creation wrapping."""
        params_key = P(1)
        stroke = SlnStrokeExpression(params_key)
        factor = stroke.pos_integration_factor(12345, 0.01)

    def test_factor_correctness(self):
        params_key = P(1)
        stroke = SlnStrokeExpression(params_key)
        dt = 0.01
        x0 = np.array([0.00123, 0.00456])

        # Create graph
        graph = gtsam.NonlinearFactorGraph()
        graph.add(stroke.pos_integration_factor(0, dt))

        # Add priors
        #                                    t0 D th1 th2 sigma mu
        graph.addPriorVector(P(1), np.array([-2 + dt, 1, 0, 0, 1, 0]),
                             gtsam.noiseModel.Constrained.All(6))
        graph.addPriorPoint2(X(0), x0, gtsam.noiseModel.Constrained.All(2))

        # Initialization
        init = gtsam.Values()
        init.insert(P(1), np.array([0, 1, 0, 0, 1, 0]))
        init.insert(X(0), x0)
        init.insert(X(1), np.zeros(2))

        # Solve
        sol = gtsam.LevenbergMarquardtOptimizer(graph, init).optimize()

        expected = np.array([dt * 1 *
                             (1 / np.sqrt(2 * np.pi) / 2) * np.exp(-np.log(2)**2 / 2), 0]) + x0
        actual = sol.atPoint2(X(1))
        self.gtsamAssertEquals(actual, expected)


if __name__ == "__main__":
    unittest.main()
