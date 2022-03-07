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
        data = np.array([
            [0.0, 0.0, 0.],
            [0.2, 1.0, 0.],
            [0.4, 2.0, 0.],
            [0.6, 3.0, 0.],
        ])

        graph = gtsam.NonlinearFactorGraph()
        fitter = SlnStrokeFit(0.01)
        graph.push_back(fitter.stroke_factors(0, 0, int(0.6 / fitter.dt)))
        graph.push_back(fitter.data_prior_factors(data))
        print(graph)
        
        sol = fitter.solve(graph)

        # test fit quality
        for t, x, y in data:
            self.gtsamAssertEquals(fitter.query_estimate_at(sol, t), np.array([x, y]))
        # expected_params = 


if __name__ == "__main__":
    unittest.main()
