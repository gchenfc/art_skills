"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test SLN incremental fitting
Author: Gerry Chen
"""

import unittest
from fit_incremental import FixedLagFitter

import numpy as np
import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P
import utils
import loader


class TestSlnFit(GtsamTestCase):
    """Test SlnFit."""

    def setUp(self) -> None:
        traj_data = loader.load_segments('A', index=1)
        self.data = np.vstack(traj_data)[::2]
        self.new_stroke_inds = [0] + np.cumsum([stroke.shape[0] for stroke in traj_data]) // 2

    def test_incremental(self):
        """Test incremental regression"""
        fitter = FixedLagFitter()
        fitter.initialize(self.data[:10])
        actual = fitter.query(self.data[:10, 0]).tolist()
        for i in range(10, self.data.shape[0]):
            actual.append(fitter.step(self.data[i], i in self.new_stroke_inds).tolist()[-1])

        actual = np.array(actual)
        # print(np.hstack((actual, self.data[:, 1:])))
        self.assertLess(np.mean(np.linalg.norm(actual[:, 1:] - self.data[:, 1:], axis=1)), 5e-2)
        np.testing.assert_allclose(actual, self.data, atol=0.2)


if __name__ == "__main__":
    unittest.main()
