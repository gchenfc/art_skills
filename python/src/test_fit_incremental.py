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
        self.new_stroke_inds = [0] + np.cumsum([stroke.shape[0] for stroke in traj_data])[:-1] // 2

    @unittest.skip
    def test_incremental(self):
        """Test incremental regression"""
        import matplotlib.pyplot as plt

        plt.plot(self.data[:, 1], self.data[:, 2], 'k-')
        plt.plot(self.data[self.new_stroke_inds, 1], self.data[self.new_stroke_inds, 2], 'r*')
        plt.savefig('tmp.png')

        fitter = FixedLagFitter()
        fitter.initialize(self.data[:10])
        actual = fitter.query(self.data[:10, 0]).tolist()
        for i in range(10, self.data.shape[0]):
            actual.append(fitter.step(self.data[i], i in self.new_stroke_inds).tolist()[-1])
            # actual.append(fitter.step(self.data[i])

        actual = np.array(actual)
        # print(np.hstack((actual, self.data[:, 1:])))
        self.assertLess(np.mean(np.linalg.norm(actual[:, 1:] - self.data[:, 1:], axis=1)), 6e-2)
        np.testing.assert_allclose(actual, self.data, atol=0.2)

    def test_incremental_D_with_segmentation(self):
        fitter = FixedLagFitter()
        fitter.initialize(self.data[:10])
        actual = fitter.query(self.data[:10, 0]).tolist()
        for i in range(10, self.data.shape[0]):
            actual.append(fitter.step(self.data[i]).tolist()[-1])

        actual = np.array(actual)
        # print(np.hstack((actual, self.data[:, 1:])))
        print(actual.shape, self.data.shape)
        self.assertLess(np.mean(np.linalg.norm(actual[:, 1:] - self.data[:, 1:], axis=1)), 6e-2)
        np.testing.assert_allclose(actual, self.data, atol=0.2)


    def test_timing(self):
        fitter = FixedLagFitter()
        # with utils.Time('initializing') as t:
        #     fitter.initialize(self.data[:10])
        #     assert t() < 10 / 120
        # actual = fitter.query(self.data[:10, 0]).tolist()
        # for i in range(10, self.data.shape[0]):
        #     actual.append(tmp := fitter.step(self.data[i]))

        # actual = np.array(actual)


if __name__ == "__main__":
    unittest.main()
