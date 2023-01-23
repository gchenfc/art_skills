"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test SLN letter regression
Author: Gerry Chen
"""

import unittest
import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P
from sln_stroke_fit import OptimizationLoggingParams
import sln_letter_fit


class TestSlnLetterFit(GtsamTestCase):
    """Test SlnLetterFit."""

    def test_fit(self):
        """Test regression on the outline of the letter 'D'.  Expect this test to take ~5s."""
        # parse the letter 'D' (mocap data)
        with np.load("data/all_letters_max.npz", allow_pickle=True) as data:
            trajectory = data['D'][1]
            dt = 1. / 120
            t = np.arange(0, trajectory.shape[0] * dt, dt).reshape(-1, 1)
            trajectory = np.hstack((t, trajectory))
        stroke1 = trajectory[0:31, :]  # Y = -0.34639319
        stroke2 = trajectory[31:57, :]  # X = 0.74592703
        stroke3 = trajectory[57:, :]  # to end
        strokes = [stroke1, stroke2, stroke3]

        # do fit
        sol, _, fitter, stroke_indices = sln_letter_fit.fit_trajectory(
            strokes,
            optimization_logging_params=OptimizationLoggingParams(print_progress=False,
                                                                  log_optimization_values=False))

        estimated_trajectory = sol['txy_from_params'][:, 1:]

        # debug
        if False:
            import matplotlib.pyplot as plt
            plt.plot(estimated_trajectory[:, 0], estimated_trajectory[:, 1])
            for stroke in strokes:
                plt.plot(stroke[:, 1], stroke[:, 2], '.')
            plt.axis('equal')
            plt.show()

        # test fit quality
        for stroke in strokes:
            for t, x, y in stroke:
                self.gtsamAssertEquals(estimated_trajectory[fitter.t2k(t)], np.array([x, y]), 0.03)


if __name__ == "__main__":
    unittest.main()
