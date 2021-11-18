"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the entire project, end-end
Author: Frank Dellaert, Sang-Won Leigh, JD Florez-Castillo
"""

import unittest
from art_skills.capture_render_trajectory import CaptureRenderTrajectory
from art_skills.chebyshev_fitter import ChebyshevFit
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.function_base import linspace
import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestChebyshevFit(GtsamTestCase):
    """Test end-to-end for Chebyshev fitting."""

    def test_capture_trajectory(self):
        """Generate a trajectory."""
        caprend = CaptureRenderTrajectory()
        trajectory = caprend.parse_trajectory(["A"])
        plt.plot(trajectory[0][0][:, 0], trajectory[0][0][:, 1])
        plt.plot(trajectory[0][1][:, 0], trajectory[0][1][:, 1])
        plt.show()

    def test_chebyshev(self):
        """Test Chebyshev fitting"""
        x = np.array([0, 0.1, 1.5, 0.25, 0])
        t = np.array([0, 0.5,   1,  1.5, 2])
        order = 3
        zipx = zip(t, x)
        datax = dict(zipx)
        trajectory = ChebyshevFit(datax, t, order)
        plt.plot(t, x)
        plt.plot(t, trajectory)
        plt.show()

    def test_write_alphabet(self):
        """Capture/take input, perceive/process, and render the entire
        alphabet."""
        caprend = CaptureRenderTrajectory()
        traj = caprend.parse_trajectory(["Test"])
        xcoord, ycoord = caprend.render_trajectory(traj, 12)
        for stroke in range(len(xcoord)):
            plt.plot(xcoord[stroke], ycoord[stroke])
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    unittest.main()
