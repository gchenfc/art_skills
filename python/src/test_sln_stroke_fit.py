"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test SLN stroke regression using python wrapper
Author: Gerry Chen
"""

import unittest
import numpy as np
import art_skills
# import .. import src
from sln_stroke_fit import SlnStrokeFit
# from . import src
# # from src.sln_stroke_fit import SlnStrokeFit
import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P


class TestSlnStrokeFit(GtsamTestCase):
    """Test SlnStrokeFit."""

    def test_create(self):
        """Test constructor and basic function calls."""
        graph = gtsam.NonlinearFactorGraph()
        fitter = SlnStrokeFit(0.01)
        self.assertEqual(fitter.t2k(0.05), 5, 'time to time index conversion failed')
        graph.push_back(fitter.stroke_factors(0, 0, 10))
        graph.push_back(fitter.data_prior_factors(np.zeros((10, 3))))
        fitter.create_initial_values(graph)

    def test_regress(self):
        """Test regression using 4 data points that should be exactly fit-able to a SLN curve"""
        #                     t0, D, th1, th2, sigma, mu
        pExpected = np.array([-0.8, 1, 0., 0., 0.2, 0.1])
        #           np.array([-1., 1., 0., 0., 0.2, 0.1]))
        data = np.array([
            [0.0, 0.0, 0.0],
            [0.2, 0.2608538815481531, 0.0],
            [0.4, 0.610783045144362, 0.0],
            [0.6, 0.8284735934786597, 0.0],
        ])

        graph = gtsam.NonlinearFactorGraph()
        # Since we expect a perfect fit to the data, any noise model should result in 0 error
        fitter = SlnStrokeFit(0.01,
                              integration_noise_model=gtsam.noiseModel.Unit.Create(2),
                              data_prior_noise_model=gtsam.noiseModel.Unit.Create(2))
        graph.push_back(fitter.stroke_factors(0, 0, int(0.6 / fitter.dt)))
        graph.push_back(fitter.data_prior_factors(data))
        print(graph)

        # Since we expect a perfect fit to the data, we want 0 error
        # TODO(gerry): play with initialization and how it affects optimization
        # TODO(gerry): understand loss landscape
        initial_values = fitter.create_initial_values(graph)
        # initial_values.update(P(0), np.array())
        sol = fitter.solve(graph,
                           initial_values=initial_values,
                           params=fitter.create_params(verbosityLM='SUMMARY',
                                                       maxIterations=300,
                                                       relativeErrorTol=0,
                                                       absoluteErrorTol=0,
                                                       errorTol=0))

        print(sol)
        # test fit quality
        for t, x, y in data:
            self.gtsamAssertEquals(fitter.query_estimate_at(sol, t), np.array([x, y]), 1e-4)
        # test fit parameters
        self.gtsamAssertEquals(fitter.query_parameters(sol, 0), pExpected)
        print(fitter.query_parameters(sol, 0).tolist())


if __name__ == "__main__":
    unittest.main()
