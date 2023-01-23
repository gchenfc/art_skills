"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test Chebyshev letter regression
Author: Gerry Chen
"""

import unittest
import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P
import chebyshev_fitter
import loader


class TestChebyshevFitter(GtsamTestCase):
    """Test Chebyshev fitter."""

    def test_known_fit(self):
        # create data
        coeff_x = [6, 3, 1]
        coeff_y = [8.2, 1.3, 0.2]
        t = np.linspace(0, 3, 30)
        x = np.polyval(coeff_x, t)
        y = np.polyval(coeff_y, t)

        # do fit and extract parameters
        sol, _, _, _ = chebyshev_fitter.fit_trajectory([np.stack((t, x, y), axis=1)], p_order=2)
        actual_order, actual_cheb_x, actual_cheb_y = sol['params'][0]

        # check for fit equality
        self.assertEqual(actual_order, 2)
        def cheb2poly(cheb_coeff):
            return np.array([[0, 0, 1], [0, 1, 0], [2, 0, -1]]).T @ cheb_coeff
        np.testing.assert_allclose(cheb2poly(actual_cheb_x), coeff_x)
        np.testing.assert_allclose(cheb2poly(actual_cheb_y), coeff_y)

    def test_fit(self):
        """Test regression on the outline of the letter 'D'."""
        strokes = loader.load_segments('D', index=1)

        # do fit
        sol, _, _, stroke_indices = chebyshev_fitter.fit_trajectory(strokes, p_order=5)

        # test fit quality
        for stroke, (begin, end) in zip(strokes, stroke_indices.values()):
            np.testing.assert_almost_equal(sol['txy_from_params'][begin:end], stroke, 3)


if __name__ == "__main__":
    unittest.main()
