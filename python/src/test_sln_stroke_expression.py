"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test python wrapper for SlnStrokeExpression
Author: Gerry Chen
"""

import itertools
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

    def test_query_speed(self):
        t0, D, th1, th2, sigma, mu = -0.1, 1.123, 9.876, 8.765, 1.432, 0.432
        stroke = SlnStrokeExpression(np.array([t0, D, th1, th2, sigma, mu]))
        t = 0.1
        expected_speed = D / (sigma * np.sqrt(2 * np.pi) *
                              (t - t0)) * np.exp(-np.square(np.log(t - t0) - mu) /
                                                 (2 * sigma * sigma))
        self.assertAlmostEqual(stroke.speed(t), expected_speed, 10)
        self.assertAlmostEqual(stroke.speed(t - t0, False), expected_speed, 10)

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

    def test_reparameterized(self):
        Reparameterized = SlnStrokeExpression.CreateSlnStrokeExpressionReparameterized

        t0, D, th1, th2, sigma, mu = -0.1, 1.123, 9.876, 8.765, 1.432, 0.432
        expected = SlnStrokeExpression(np.array([t0, D, th1, th2, sigma, mu]))
        actual = Reparameterized(np.array([t0, np.log(D), th1, th2, np.log(sigma), mu]))
        for t in [0.1, 0.5, 0.8]:
            self.assertEqual(actual.speed(t), expected.speed(t))
            self.gtsamAssertEquals(actual.displacement(t, 0.1), expected.displacement(t, 0.1))

        # Key version
        p_key, timestep = P(666), 666

        values = gtsam.Values()
        values.insert(p_key, np.array([t0, D, th1, th2, sigma, mu]))
        values.insert(X(timestep), np.array([1.1, 1.2]))
        values.insert(X(timestep + 1), np.array([2.1, 1.9]))

        for t, D, sigma in itertools.product([0.1, 0.5, 0.8], [0.4, 0.3, 1.5], [0.9, 0.7, 0.1]):
            values.update(p_key, np.array([t0, D, th1, th2, sigma, mu]))
            expected_error = SlnStrokeExpression(p_key).pos_integration_factor(timestep,
                                                                               0.01).error(values)
            values.update(p_key, np.array([t0, np.log(D), th1, th2, np.log(sigma), mu]))
            actual_error = Reparameterized(p_key).pos_integration_factor(timestep,
                                                                         0.01).error(values)

            self.assertEqual(actual_error, expected_error)


if __name__ == "__main__":
    unittest.main()
