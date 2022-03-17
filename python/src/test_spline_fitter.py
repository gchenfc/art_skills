"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test Spline letter regression
Author: Gerry Chen
"""

import unittest
import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P
import spline_fitter
import loader


class TestSplineFitter(GtsamTestCase):
    """Test Spline fitter."""

    def test_fit(self):
        """Test regression on the outline of the letter 'D'."""
        strokes = loader.load_segments('D', index=1)

        # do fit
        sol, _, _, stroke_indices = spline_fitter.fit_trajectory(strokes, p_order=5)

        # test fit quality
        for stroke, (begin, end) in zip(strokes, stroke_indices.values()):
            np.testing.assert_almost_equal(sol['txy_from_params'][begin:end], stroke, 3)


if __name__ == "__main__":
    unittest.main()
